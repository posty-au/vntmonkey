// Hungry Monkey Control System - Teensy 4.1
// Torque-on-demand compound turbo controller
// Version 1.8 - Added Watchdog Timer (WDT) hardware protection
// (c) 2026 - Built with love for the Monkey 🐒
// Open-source friendly: Configurable for various builds
// License: MIT - Feel free to fork, modify, and share!

#include <FlexCAN_T4.h>
#include <FreqMeasureMulti.h>
#include <SD.h>
#include <SPI.h>
#include <elapsedMillis.h>
#include <Watchdog_t4.h>        // Watchdog library

// === USER CALIBRATION SECTION ===

// === SYSTEM MODE TOGGLE ===
#define SINGLE_TURBO false

#define VNT_INVERTED false
#define USE_SENT_SENSORS false
#define LPG_SCALE_FACTOR 1.0
#define DRIVE_RATIO_LIMIT 1.5

// Actuator frequencies
#define VNT_FREQ 300.0
#define WG_FREQ 30.0
#define LPG_FREQ 10000.0

// Safety limits
#define COMP_SPEED_MAX 220000.0
#define EGT_MAX 900.0
#define EMP_MAX 4.5

// Control logic
#define HANDOVER_START_RPM 4200.0
#define HANDOVER_END_RPM 5000.0
#define LPG_MIN_TORQUE 400.0
#define LPG_MIN_RPM 1500.0
#define TABLETOP_START_RPM 2000.0
#define TABLETOP_END_RPM 4500.0
#define FEED_FORWARD_GAIN 0.05
#define MAP_RATE_MAX 5.0
#define COMP_SPEED_TRIM_THRESHOLD 0.9
#define COMP_SPEED_TRIM_AMOUNT 10.0
#define MAP_SPIKE_TRIM_AMOUNT 15.0
#define EMP_PANIC_TRIM_AMOUNT 25.0
#define WG_CRACK_ON_SPIKE 30.0
#define LPG_BASE_DUTY 0.10
#define LPG_TORQUE_SCALE 0.40
#define LPG_TABLETOP_BOOST 0.15
#define LPG_MAX_DUTY 0.85
#define MIN_ENGINE_RPM 800.0
#define MAP_MIN_FOR_EMP_CHECK 1.0

// Safety & sensor calibration
#define SAFETY_LATCH_TIME_MS 10000
#define MAP_SMOOTHING_ALPHA 0.1
#define EMP_SMOOTHING_ALPHA 0.1
#define EGT_SMOOTHING_ALPHA 0.15
#define COMP_PULSES_PER_REV 1.0
#define COMP_PULSE_TIMEOUT_MS 100

// === WATCHDOG CONFIGURATION ===
#define WATCHDOG_TIMEOUT_MS 100   // 100ms - aggressive for 500Hz loop safety

// === DIAGNOSTIC OPTIONS ===
#define ENABLE_BOOT_SWEEP true
#define ENABLE_SERIAL_DIAG true

// === PIN DEFINITIONS ===
#define PIN_VNT_LEFT      2
#define PIN_VNT_RIGHT     3
#define PIN_WASTEGATE     4
#define PIN_LPG_LEFT      5
#define PIN_LPG_RIGHT     8

#define PIN_COMP_SPEED_L  6
#define PIN_COMP_SPEED_R  7

#define PIN_MAP           A0
#define PIN_EGT           A1
#define PIN_EMP           A2

#define PIN_SD_CS         10

// === CONSTANTS ===
const uint32_t LOOP_RATE_HZ = 500;
const uint32_t LOOP_INTERVAL_US = 1000000 / LOOP_RATE_HZ;
const uint32_t TORQUE_MSG_ID = 0x200;

// === GLOBAL OBJECTS ===
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
FreqMeasureMulti compSpeedL;
FreqMeasureMulti compSpeedR;
WDT_T4<WDT1> wdt;  // Watchdog instance

File logFile;

// === MAPS ===
const int TORQUE_BINS = 11;
const int RPM_BINS = 9;
float torqueAxis[TORQUE_BINS] = {0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000};
float rpmAxis[RPM_BINS] = {1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000};
float vntBaseMap[RPM_BINS][TORQUE_BINS] = {
  {90, 85, 80, 75, 70, 65, 60, 55, 50, 45, 40},
  {85, 80, 75, 70, 65, 60, 55, 50, 45, 40, 35},
  {80, 75, 70, 65, 60, 55, 50, 45, 40, 35, 30},
  {75, 70, 65, 60, 55, 50, 45, 40, 35, 30, 25},
  {70, 65, 60, 55, 50, 45, 40, 35, 30, 25, 20},
  {65, 60, 55, 50, 45, 40, 35, 30, 25, 20, 15},
  {60, 55, 50, 45, 40, 35, 30, 25, 20, 15, 10},
  {55, 50, 45, 40, 35, 30, 25, 20, 15, 10, 5},
  {50, 45, 40, 35, 30, 25, 20, 15, 10, 5, 0}
};

// === STATE VARIABLES ===
float torqueRequest = 0.0;
float engineRPM = 0.0;
float compSpeedL_rpm = 0.0;
float compSpeedR_rpm = 0.0;
float mapPressure = 0.0;
float egtTemp = 0.0;
float empPressure = 0.0;
float torquePrev = 0.0;
unsigned long prevTime = 0;

bool systemHealthy = true;
unsigned long safetyTripTime = 0;

float mapSmoothed = 0.0;
float empSmoothed = 0.0;
float egtSmoothed = 0.0;

elapsedMillis timeSinceLastPulseL;
elapsedMillis timeSinceLastPulseR;

static int flushCounter = 0;
const int FLUSH_EVERY_N_LOOPS = 500;

bool diagnosticModeActive = false;

// === WATCHDOG SETUP ===
void watchdogSetup() {
  WDT_timings_t config;
  config.trigger = 5;       // Trigger reset after ~5 seconds of no feed (safety net)
  config.timeout = WATCHDOG_TIMEOUT_MS / 1000.0;  // Primary timeout in seconds (100ms)
  config.pin = 23;          // Optional: use external watchdog pin if wired
  wdt.begin(config);
}

void vntSweep() {
  Serial.println("DIAGNOSTIC: Starting VNT actuator sweep (Open → Closed → Open)");

  for (int pos = 0; pos <= 100; pos += 2) {
    float duty = map(pos, 0, 100, VNT_INVERTED ? 90 : 10, VNT_INVERTED ? 10 : 90);
    analogWrite(PIN_VNT_LEFT,  (uint16_t)(duty / 100.0 * 4095));
    analogWrite(PIN_VNT_RIGHT, (uint16_t)(duty / 100.0 * 4095));
    delay(50);
  }

  delay(1000);

  for (int pos = 100; pos >= 0; pos -= 2) {
    float duty = map(pos, 0, 100, VNT_INVERTED ? 90 : 10, VNT_INVERTED ? 10 : 90);
    analogWrite(PIN_VNT_LEFT,  (uint16_t)(duty / 100.0 * 4095));
    analogWrite(PIN_VNT_RIGHT, (uint16_t)(duty / 100.0 * 4095));
    delay(50);
  }

  Serial.println("DIAGNOSTIC: VNT sweep complete. System ready.");
}

void runDiagnostics() {
#if ENABLE_SERIAL_DIAG
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    if (cmd == 'T' || cmd == 't') {
      float testPos = Serial.parseFloat();
      if (testPos >= 0 && testPos <= 100) {
        testPos = constrain(testPos, 0, 100);
        float vntDuty = map(testPos, 0, 100, VNT_INVERTED ? 90 : 10, VNT_INVERTED ? 10 : 90);

        analogWrite(PIN_VNT_LEFT,  (uint16_t)(vntDuty / 100.0 * 4095));
        analogWrite(PIN_VNT_RIGHT, (uint16_t)(vntDuty / 100.0 * 4095));

        Serial.print("DIAGNOSTIC MODE: VNT forced to ");
        Serial.print(testPos);
        Serial.println("% closed");

        diagnosticModeActive = true;
      }
    }

    if (cmd == 'L' || cmd == 'l') {
      Serial.println("DIAGNOSTIC: LPG Pulse Test (10% Duty, 1s)");
      analogWrite(PIN_LPG_LEFT,  (uint16_t)(0.10 * 4095));
      analogWrite(PIN_LPG_RIGHT, (uint16_t)(0.10 * 4095));
      delay(1000);
      analogWrite(PIN_LPG_LEFT,  0);
      analogWrite(PIN_LPG_RIGHT, 0);
      Serial.println("DIAGNOSTIC: LPG test complete");
    }

    if (cmd == 'N' || cmd == 'n') {
      diagnosticModeActive = false;
      Serial.println("DIAGNOSTIC: Returned to normal control mode");
    }
  }
#endif
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  analogWriteFrequency(PIN_VNT_LEFT, VNT_FREQ);
  analogWriteFrequency(PIN_VNT_RIGHT, VNT_FREQ);
  analogWriteFrequency(PIN_WASTEGATE, WG_FREQ);
  analogWriteFrequency(PIN_LPG_LEFT, LPG_FREQ);
  analogWriteFrequency(PIN_LPG_RIGHT, LPG_FREQ);
  analogWriteResolution(12);

  compSpeedL.begin(PIN_COMP_SPEED_L);
  compSpeedR.begin(PIN_COMP_SPEED_R);

  Can0.begin();
  Can0.setBaudRate(500000);

  if (!SD.begin(PIN_SD_CS)) {
    Serial.println("SD card failed!");
  } else {
    logFile = SD.open("hungrylog.csv", FILE_WRITE);
    if (logFile) {
      logFile.println("time,torqueReq,RPM,compL,compR,MAP,EGT,EMP,vntDuty,wgDuty,lpgDuty,healthy");
    }
  }

  // Pre-seed filters
  float rawMap = analogRead(PIN_MAP) * (5.0 / 1023.0) * 0.8 + 0.2;
  mapSmoothed = rawMap;
  mapPressure = rawMap;

  float rawEmp = analogRead(PIN_EMP) * (5.0 / 1023.0) * 1.25;
  empSmoothed = rawEmp;
  empPressure = rawEmp;

  float rawEgt = analogRead(PIN_EGT) * (5.0 / 1023.0) * 200.0;
  egtSmoothed = rawEgt;
  egtTemp = rawEgt;

#if ENABLE_BOOT_SWEEP
  vntSweep();
#endif

  // === START WATCHDOG ONLY AFTER LONG SETUP TASKS ===
  watchdogSetup();
  Serial.println("Watchdog Active: 100ms Heartbeat required.");

  Serial.println("Hungry Monkey Awake! 🐒");
  Serial.println("Serial commands: T0-T100 (VNT position), L (LPG test), N (normal mode)");
  prevTime = micros();
}

float interpolate3D(float torque, float rpm) {
  // unchanged
  int tLow = 0, tHigh = TORQUE_BINS - 1;
  for (int i = 0; i < TORQUE_BINS - 1; i++) {
    if (torque < torqueAxis[i+1]) { tHigh = i+1; tLow = i; break; }
  }
  int rLow = 0, rHigh = RPM_BINS - 1;
  for (int i = 0; i < RPM_BINS - 1; i++) {
    if (rpm < rpmAxis[i+1]) { rHigh = i+1; rLow = i; break; }
  }

  float v11 = vntBaseMap[rLow][tLow];
  float v12 = vntBaseMap[rLow][tHigh];
  float v21 = vntBaseMap[rHigh][tLow];
  float v22 = vntBaseMap[rHigh][tHigh];

  float fracT = (torque - torqueAxis[tLow]) / (torqueAxis[tHigh] - torqueAxis[tLow]);
  float fracR = (rpm - rpmAxis[rLow]) / (rpmAxis[rHigh] - rpmAxis[rLow]);

  return v11 * (1-fracT)*(1-fracR) + v12 * fracT*(1-fracR) +
         v21 * (1-fracT)*fracR + v22 * fracT*fracR;
}

void loop() {
  wdt.feed();  // Critical: Feed the watchdog first - guarantees reset on any hang

  unsigned long now = micros();
  if (now - prevTime < LOOP_INTERVAL_US) {
    runDiagnostics();
    return;
  }
  prevTime = now;

  runDiagnostics();

  if (diagnosticModeActive) {
    return;
  }

  // === NORMAL CONTROL LOGIC ===
  CAN_message_t msg;
  if (Can0.read(msg)) {
    if (msg.id == TORQUE_MSG_ID) {
      torqueRequest = (msg.buf[0] << 8 | msg.buf[1]) * 0.1;
      engineRPM = (msg.buf[2] << 8 | msg.buf[3]);
    }
  }

  if (compSpeedL.available()) {
    float freqHz = compSpeedL.countToFrequency(compSpeedL.read());
    compSpeedL_rpm = freqHz * 60.0 / COMP_PULSES_PER_REV;
    timeSinceLastPulseL = 0;
  }
  if (timeSinceLastPulseL > COMP_PULSE_TIMEOUT_MS) {
    compSpeedL_rpm = 0.0;
  }

  if (compSpeedR.available()) {
    float freqHz = compSpeedR.countToFrequency(compSpeedR.read());
    compSpeedR_rpm = freqHz * 60.0 / COMP_PULSES_PER_REV;
    timeSinceLastPulseR = 0;
  }
  if (timeSinceLastPulseR > COMP_PULSE_TIMEOUT_MS) {
    compSpeedR_rpm = 0.0;
  }

#if USE_SENT_SENSORS
#else
  float rawMap = analogRead(PIN_MAP) * (5.0 / 1023.0) * 0.8 + 0.2;
  mapSmoothed = (mapSmoothed * (1.0 - MAP_SMOOTHING_ALPHA)) + (rawMap * MAP_SMOOTHING_ALPHA);
  mapPressure = mapSmoothed;

  float rawEmp = analogRead(PIN_EMP) * (5.0 / 1023.0) * 1.25;
  empSmoothed = (empSmoothed * (1.0 - EMP_SMOOTHING_ALPHA)) + (rawEmp * EMP_SMOOTHING_ALPHA);
  empPressure = empSmoothed;

  float rawEgt = analogRead(PIN_EGT) * (5.0 / 1023.0) * 200.0;
  egtSmoothed = (egtSmoothed * (1.0 - EGT_SMOOTHING_ALPHA)) + (rawEgt * EGT_SMOOTHING_ALPHA);
  egtTemp = egtSmoothed;
#endif

  // Safety checks...
  bool currentHealthy = true;
  if (compSpeedL_rpm > COMP_SPEED_MAX || compSpeedR_rpm > COMP_SPEED_MAX ||
      egtTemp > EGT_MAX || empPressure > EMP_MAX) {
    currentHealthy = false;
  }

  if (!currentHealthy) {
    if (systemHealthy) safetyTripTime = millis();
    systemHealthy = false;
  } else {
    if (SAFETY_LATCH_TIME_MS == 0) {
      systemHealthy = false;
    } else if ((millis() - safetyTripTime) > SAFETY_LATCH_TIME_MS) {
      systemHealthy = true;
      safetyTripTime = 0;
    }
  }

  float vntTargetPctClosed = 0.0;
  float wgDuty = 0.0;
  float lpgPulse = 0.0;

  if (systemHealthy && torqueRequest > 0 && engineRPM > MIN_ENGINE_RPM) {
    vntTargetPctClosed = interpolate3D(torqueRequest, engineRPM);

#if SINGLE_TURBO
    float avgCompSpeed = compSpeedL_rpm;
#else
    if (engineRPM > HANDOVER_START_RPM) {
        float handoverFactor = (engineRPM - HANDOVER_START_RPM) / (HANDOVER_END_RPM - HANDOVER_START_RPM);
        handoverFactor = constrain(handoverFactor, 0.0, 1.0);
        vntTargetPctClosed *= (1.0 - handoverFactor);
    }
    float avgCompSpeed = (compSpeedL_rpm + compSpeedR_rpm) / 2.0;
#endif

    float pedalRate = (torqueRequest - torquePrev) / (LOOP_INTERVAL_US / 1000000.0);
    vntTargetPctClosed += pedalRate * FEED_FORWARD_GAIN;
    vntTargetPctClosed = constrain(vntTargetPctClosed, 0, 100);

    if (avgCompSpeed > COMP_SPEED_MAX * COMP_SPEED_TRIM_THRESHOLD) {
      vntTargetPctClosed -= COMP_SPEED_TRIM_AMOUNT;
    }

    static float mapPrev = 0.0;
    float mapRate = (mapPressure - mapPrev) / (LOOP_INTERVAL_US / 1000000.0);
    if (mapRate > MAP_RATE_MAX) {
      vntTargetPctClosed -= MAP_SPIKE_TRIM_AMOUNT;
      wgDuty = WG_CRACK_ON_SPIKE;
    }
    mapPrev = mapPressure;

    if (empPressure > (mapPressure * DRIVE_RATIO_LIMIT) && mapPressure > MAP_MIN_FOR_EMP_CHECK) {
       vntTargetPctClosed -= EMP_PANIC_TRIM_AMOUNT;
       systemHealthy = false;
       safetyTripTime = millis();
    }

    if (torqueRequest > LPG_MIN_TORQUE && engineRPM > LPG_MIN_RPM) {
       float lpgDuty = LPG_BASE_DUTY + ((torqueRequest - LPG_MIN_TORQUE) / (1000.0 - LPG_MIN_TORQUE)) * LPG_TORQUE_SCALE;
       if (engineRPM >= TABLETOP_START_RPM && engineRPM <= TABLETOP_END_RPM) {
          lpgDuty += LPG_TABLETOP_BOOST;
       }
       lpgPulse = constrain(lpgDuty, 0.0, LPG_MAX_DUTY);
       lpgPulse *= LPG_SCALE_FACTOR;
       lpgPulse = constrain(lpgPulse, 0.0, 1.0);
    }
  } else {
    vntTargetPctClosed = 0;
    wgDuty = 100;
  }

  torquePrev = torqueRequest;

  float vntDuty = map(vntTargetPctClosed, 0, 100, VNT_INVERTED ? 90 : 10, VNT_INVERTED ? 10 : 90);

  analogWrite(PIN_VNT_LEFT,  (uint16_t)(vntDuty / 100.0 * 4095));
  analogWrite(PIN_VNT_RIGHT, (uint16_t)(vntDuty / 100.0 * 4095));
  analogWrite(PIN_WASTEGATE, (uint16_t)(wgDuty / 100.0 * 4095));
  analogWrite(PIN_LPG_LEFT,  (uint16_t)(lpgPulse * 4095));
  analogWrite(PIN_LPG_RIGHT, (uint16_t)(lpgPulse * 4095));

  // Logging
  Serial.printf("%.1f,%.0f,%.0f,%.0f,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f,%d\n",
                torqueRequest, engineRPM, compSpeedL_rpm, compSpeedR_rpm,
                mapPressure, egtTemp, empPressure, vntDuty, wgDuty, lpgPulse * 100.0, systemHealthy);

  if (logFile) {
    logFile.printf("%lu,%.1f,%.0f,%.0f,%.0f,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f,%d\n",
                   millis(), torqueRequest, engineRPM, compSpeedL_rpm, compSpeedR_rpm,
                   mapPressure, egtTemp, empPressure, vntDuty, wgDuty, lpgPulse * 100.0, systemHealthy);

    if (++flushCounter >= FLUSH_EVERY_N_LOOPS) {
      logFile.flush();
      flushCounter = 0;
    }
  }
}
