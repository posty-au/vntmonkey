// Wideband Autonomous Module - Firmware v1.2
// Teensy 4.1 - i.MX RT1062
// Date: 2026-01-08
// Status: Hardware-lock safe. Contract frozen for v1.0 silicon.

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <NativeEthernet.h>
#include <DS3231.h>
#include <MCP4725.h>

// =============================================================================
// PIN DEFINITIONS
// =============================================================================

#define PIN_HEATER_PWM      2
#define PIN_PUMP_ADC        A0
#define PIN_NERNST_ADC      A1
#define PIN_READY_GPIO      3
#define PIN_FAULT_GPIO      4

#define DAC_I2C_ADDR        0x60

// =============================================================================
// CONSTANTS & THRESHOLDS
// =============================================================================

constexpr float NERNST_IMPEDANCE_THRESHOLD_OHMS = 80.0f;
constexpr float FREE_AIR_SIGMA_MAX              = 0.002f;
constexpr float FREE_AIR_EXPECTED_BAND_LOW      = 1.50f;
constexpr float FREE_AIR_EXPECTED_BAND_HIGH     = 1.80f;
constexpr float FREE_AIR_PRECHECK_TOLERANCE_V   = 0.15f;

constexpr uint32_t PREHEAT_RAMP_TIME_MS         = 60000;
constexpr uint32_t FREE_AIR_WINDOW_START_MS     = 10000;
constexpr uint32_t FREE_AIR_WINDOW_DURATION_MS  = 30000;
constexpr uint16_t PWM_MAX_DUTY                 = 4095;
constexpr uint32_t FREE_AIR_SAMPLES             = 1000;
constexpr uint32_t OPERATION_LOOP_HZ            = 100;

// Physical sanity bounds for lambda (protects downstream consumers)
constexpr float LAMBDA_MIN = 0.65f;
constexpr float LAMBDA_MAX = 1.50f;

// =============================================================================
// STATE MACHINE
// =============================================================================

enum class SystemState : uint8_t {
  POWER_OFF = 0,
  BOOT,
  PREHEAT,
  THERMAL_OK,
  FREE_AIR_CALIBRATE,
  VALIDATE,
  OPERATION,
  FAULT_HOLD
};

volatile SystemState currentState = SystemState::BOOT;

// =============================================================================
// GLOBALS
// =============================================================================

struct SensorProfile {
  float pumpOffset_V = 1.65f;
  float heaterAgingFactor = 1.0f;
  uint32_t freeAirCount = 0;
};

SensorProfile profile;

float currentLambda = 1.000f;
bool lambdaValid = false;

volatile float latestPump_V = 1.65f;

float pumpSampleBuffer[FREE_AIR_SAMPLES];
uint32_t sampleIndex = 0;

const IPAddress monkeyIP(192, 168, 1, 100);  // Site-specific — change per install

// =============================================================================
// HARDWARE OBJECTS
// =============================================================================

DS3231 rtc;
MCP4725 dac(DAC_I2C_ADDR);

uint8_t mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(192, 168, 1, 177);
EthernetUDP udp;
const uint16_t UDP_PORT = 8888;

// File-scope timer — required for safe shutdown in fault
IntervalTimer adcTimer;

// =============================================================================
// FUNCTION PROTOTYPES
// =============================================================================

void enterFaultHold(const char* reason);
float readNernstImpedance();
void updateHeaterPWM(uint16_t duty);
float calculateMean(const float* data, uint32_t n);
float calculateStdDev(const float* data, uint32_t n, float mean);

// =============================================================================
// SETUP
// =============================================================================

void setup() {
  pinMode(PIN_HEATER_PWM, OUTPUT);
  pinMode(PIN_READY_GPIO, OUTPUT);
  pinMode(PIN_FAULT_GPIO, OUTPUT);
  digitalWrite(PIN_READY_GPIO, LOW);
  digitalWrite(PIN_FAULT_GPIO, LOW);
  analogWrite(PIN_HEATER_PWM, 0);

  analogReadResolution(12);
  analogWriteResolution(12);

  Serial.begin(115200);

  if (!SD.begin(BUILTIN_SDCARD)) {
    enterFaultHold("SD init fail");
    return;
  }

  File file = SD.open("sensor_profile.txt", FILE_READ);
  if (file) {
    profile.pumpOffset_V = file.parseFloat();
    profile.heaterAgingFactor = file.parseFloat();
    file.close();
  }

  Wire.begin();
  if (!rtc.begin()) {
    enterFaultHold("RTC fail");
    return;
  }

  dac.begin();
  Ethernet.begin(mac, ip);
  udp.begin(UDP_PORT);

  currentState = SystemState::PREHEAT;
  Serial.println("WAM v1.2 - PREHEAT - Contract locked");
}

// =============================================================================
// MAIN LOOP
// =============================================================================

void loop() {
  static uint32_t stateTimer = millis();
  static uint16_t preheatDuty = 0;

  switch (currentState) {

    case SystemState::PREHEAT: {
      uint32_t elapsed = millis() - stateTimer;
      if (elapsed < PREHEAT_RAMP_TIME_MS) {
        preheatDuty = (uint16_t)((elapsed / (float)PREHEAT_RAMP_TIME_MS) * 0.8f * PWM_MAX_DUTY);
        updateHeaterPWM(preheatDuty);
      } else {
        updateHeaterPWM(0.8f * PWM_MAX_DUTY);
      }

      static uint32_t checkTimer = 0;
      if (millis() - checkTimer > 2000) {
        checkTimer = millis();
        if (readNernstImpedance() < NERNST_IMPEDANCE_THRESHOLD_OHMS) {
          currentState = SystemState::THERMAL_OK;
          stateTimer = millis();
          Serial.println("THERMAL OK");
        }
      }
      break;
    }

    case SystemState::THERMAL_OK: {
      if (millis() - stateTimer > FREE_AIR_WINDOW_START_MS) {
        float currentPump = analogRead(PIN_PUMP_ADC) * (3.3f / 4095.0f);
        if (abs(currentPump - 1.65f) < FREE_AIR_PRECHECK_TOLERANCE_V) {
          currentState = SystemState::FREE_AIR_CALIBRATE;
          stateTimer = millis();
          sampleIndex = 0;
          Serial.println("FREE AIR CALIBRATE - conditions met");
        } else if (millis() - stateTimer > FREE_AIR_WINDOW_DURATION_MS + FREE_AIR_WINDOW_START_MS) {
          enterFaultHold("Free-air window expired - no stable ambient");
        }
      }
      break;
    }

    case SystemState::FREE_AIR_CALIBRATE: {
      if (sampleIndex < FREE_AIR_SAMPLES) {
        pumpSampleBuffer[sampleIndex++] = analogRead(PIN_PUMP_ADC) * (3.3f / 4095.0f);
        delay(1);
      } else {
        currentState = SystemState::VALIDATE;
      }
      break;
    }

    case SystemState::VALIDATE: {
      float mean = calculateMean(pumpSampleBuffer, FREE_AIR_SAMPLES);
      float sigma = calculateStdDev(pumpSampleBuffer, FREE_AIR_SAMPLES, mean);

      if (sigma < FREE_AIR_SIGMA_MAX &&
          mean > FREE_AIR_EXPECTED_BAND_LOW &&
          mean < FREE_AIR_EXPECTED_BAND_HIGH) {
        profile.pumpOffset_V = mean;

        SD.remove("sensor_profile.txt");
        File f = SD.open("sensor_profile.txt", FILE_WRITE);
        if (f) {
          f.println(profile.pumpOffset_V, 6);
          f.println(profile.heaterAgingFactor, 6);
          f.close();
        }

        currentState = SystemState::OPERATION;
        digitalWrite(PIN_READY_GPIO, HIGH);

        adcTimer.begin([]() {
          latestPump_V = analogRead(PIN_PUMP_ADC) * (3.3f / 4095.0f);
        }, 1000000 / OPERATION_LOOP_HZ);

        Serial.println("OPERATION - Lambda valid");
      } else {
        enterFaultHold("Free-air validation failed");
      }
      break;
    }

    case SystemState::OPERATION: {
      static uint32_t lastOutput = 0;
      if (millis() - lastOutput >= (1000 / OPERATION_LOOP_HZ)) {
        lastOutput = millis();

        float delta = latestPump_V - profile.pumpOffset_V;

        // Linear approximation — engineering telemetry only
        currentLambda = 1.0f + (delta / 1.5f);

        // Sanity clamp — protects downstream systems
        currentLambda = constrain(currentLambda, LAMBDA_MIN, LAMBDA_MAX);
        lambdaValid = true;

        // DAC: 0–3V output
        uint16_t dacValue = (uint16_t)(currentLambda * 1365.0f);  // 3.0V / 4095 * scaling
        dac.setValue(dacValue);

        // UDP telemetry
        udp.beginPacket(monkeyIP, UDP_PORT);
        udp.printf("LAMBDA=%.4f OFFSET=%.4f RAW=%.4f\n",
                   currentLambda, profile.pumpOffset_V, latestPump_V);
        udp.endPacket();
      }
      break;
    }

    case SystemState::FAULT_HOLD:
      // Quiescent — timer already stopped in enterFaultHold()
      break;

    default:
      break;
  }

  // Continuous Nernst health check
  if (currentState >= SystemState::THERMAL_OK && currentState < SystemState::FAULT_HOLD) {
    if (readNernstImpedance() > NERNST_IMPEDANCE_THRESHOLD_OHMS * 2) {
      enterFaultHold("Nernst impedance loss");
    }
  }
}

// =============================================================================
// HELPERS
// =============================================================================

void enterFaultHold(const char* reason) {
  Serial.print("FAULT HOLD: ");
  Serial.println(reason);

  updateHeaterPWM(0);
  adcTimer.end();                  // Critical: stop ISR
  digitalWrite(PIN_READY_GPIO, LOW);
  digitalWrite(PIN_FAULT_GPIO, HIGH);
  lambdaValid = false;
  currentState = SystemState::FAULT_HOLD;
}

void updateHeaterPWM(uint16_t duty) {
  analogWrite(PIN_HEATER_PWM, duty);
}

float readNernstImpedance() {
  // Hardware contract note:
  // Transimpedance stage must present < ~1kΩ source impedance to ADC
  // for accurate 100 Hz sampling (settling within conversion time).
  float raw = analogRead(PIN_NERNST_ADC) * (3.3f / 4095.0f);
  return raw * 100.0f; // Placeholder scaling
}

float calculateMean(const float* data, uint32_t n) {
  float sum = 0;
  for (uint32_t i = 0; i < n; i++) sum += data[i];
  return sum / n;
}

float calculateStdDev(const float* data, uint32_t n, float mean) {
  float sumSq = 0;
  for (uint32_t i = 0; i < n; i++) {
    float diff = data[i] - mean;
    sumSq += diff * diff;
  }
  return sqrtf(sumSq / n);
}
