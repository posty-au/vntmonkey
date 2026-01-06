// Hungry Monkey Control System - Teensy 4.1
// Torque-on-demand compound turbo controller
// Version 1.5 - Critical real-world fixes: SD flush latency, safety latching, analog noise filtering
// (c) 2026 - Built with love for the Monkey 🐒
// Open-source friendly: Configurable for various builds (e.g., different actuators, sensors)
// License: MIT - Feel free to fork, modify, and share!

#include <FlexCAN_T4.h>          // CAN bus
#include <FreqMeasureMulti.h>    // Compressor speed
#include <SD.h>                  // Map storage & logging
#include <SPI.h>
#include <NativeEthernet.h>      // If Ethernet used for advanced logging (optional)

// === USER CALIBRATION SECTION ===
// Customize these defines for your specific hardware/build

// === SYSTEM MODE TOGGLE ===
#define SINGLE_TURBO false  // Set to TRUE for a single VNT setup, FALSE for Compound/Twin

#define VNT_INVERTED false // Set true if 90% PWM = Fully Open
#define USE_SENT_SENSORS false // Set true if using BMW B57 factory digital sensors
#define LPG_SCALE_FACTOR 1.0 // 1.0 = Default, 0.5 = Half cooling, 2.0 = Double
#define DRIVE_RATIO_LIMIT 1.5 // EMP/MAP safety threshold

// Additional calibrations
#define VNT_FREQ 300.0 // PWM frequency for VNT actuators (Hz; e.g., 140.0 or 300.0 for Hella)
#define WG_FREQ 30.0 // PWM frequency for wastegate solenoid (Hz; 25-35 typical)
#define LPG_FREQ 10000.0 // PWM frequency for LPG injectors (Hz; for peak-hold simulation)
#define COMP_SPEED_MAX 220000.0 // Max compressor speed (RPM; safety limit per turbo)
#define EGT_MAX 900.0 // Max exhaust gas temperature (°C)
#define EMP_MAX 4.5 // Max exhaust manifold pressure (bar absolute)
#define HANDOVER_START_RPM 4200.0 // RPM to start compound handover (force VNT open)
#define HANDOVER_END_RPM 5000.0 // RPM where VNT is fully open during handover
#define LPG_MIN_TORQUE 400.0 // Min torque (Nm) to activate LPG cooling
#define LPG_MIN_RPM 1500.0 // Min RPM to activate LPG cooling
#define TABLETOP_START_RPM 2000.0 // Start of high-heat "tabletop" RPM range for extra LPG
#define TABLETOP_END_RPM 4500.0 // End of high-heat "tabletop" RPM range
#define FEED_FORWARD_GAIN 0.05 // Gain for pedal rate feed-forward (tune for responsiveness)
#define MAP_RATE_MAX 5.0 // Max MAP change rate (bar/s) before damping
#define COMP_SPEED_TRIM_THRESHOLD 0.9 // Fraction of max speed to start trimming VNT (0.9 = 90%)
#define COMP_SPEED_TRIM_AMOUNT 10.0 // Amount to reduce VNT % closed on overspeed trim
#define MAP_SPIKE_TRIM_AMOUNT 15.0 // Amount to reduce VNT % closed on MAP spike
#define EMP_PANIC_TRIM_AMOUNT 25.0 // Amount to reduce VNT % closed on EMP ratio panic
#define WG_CRACK_ON_SPIKE 30.0 // Wastegate % open on MAP spike
#define LPG_BASE_DUTY 0.10 // Base LPG duty cycle
#define LPG_TORQUE_SCALE 0.40 // Scaling factor for torque-based LPG increase
#define LPG_TABLETOP_BOOST 0.15 // Extra duty during tabletop RPM
#define LPG_MAX_DUTY 0.85 // Max LPG duty to protect injectors
#define MIN_ENGINE_RPM 800.0 // Min RPM for active control (below = fail-safe)
#define MAP_MIN_FOR_EMP_CHECK 1.0 // Min MAP (bar) to enable EMP ratio check

// New safety calibration
#define SAFETY_LATCH_TIME_MS 10000  // Once tripped, stay in safe mode for 10 seconds (0 = latch until power cycle)
#define MAP_SMOOTHING_ALPHA 0.1     // EMA factor for MAP (0.05-0.2 typical; lower = smoother)
#define EMP_SMOOTHING_ALPHA 0.1     // EMA factor for EMP
#define EGT_SMOOTHING_ALPHA 0.15    // EMA factor for EGT (slightly slower)

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

File logFile;

// === CALIBRATION MAPS ===
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
float mapPressure = 0.0;      // Smoothed
float egtTemp = 0.0;         // Smoothed
float empPressure = 0.0;     // Smoothed
float torquePrev = 0.0;
unsigned long prevTime = 0;

// Safety state
bool systemHealthy = true;
unsigned long safetyTripTime = 0;  // Timestamp when safety tripped (0 = healthy)

// Noise filtering
float mapSmoothed = 0.0;
float empSmoothed = 0.0;
float egtSmoothed = 0.0;

// SD flush management
static int flushCounter = 0;
const int FLUSH_EVERY_N_LOOPS = 500;  // ~1 Hz at 500 Hz loop

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

  Serial.println("Hungry Monkey Awake! 🐒");
  prevTime = micros();
}

float interpolate3D(float torque, float rpm) {
  // (unchanged - same as before)
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
  unsigned long now = micros();
  if (now - prevTime < LOOP_INTERVAL_US) return;
  prevTime = now;

  // === READ CAN ===
  CAN_message_t msg;
  if (Can0.read(msg)) {
    if (msg.id == TORQUE_MSG_ID) {
      torqueRequest = (msg.buf[0] << 8 | msg.buf[1]) * 0.1;
      engineRPM = (msg.buf[2] << 8 | msg.buf[3]);
    }
  }

  // === READ SENSORS ===
  if (compSpeedL.available()) compSpeedL_rpm = compSpeedL.read() * 60.0;
  if (compSpeedR.available()) compSpeedR_rpm = compSpeedR.read() * 60.0;

#if USE_SENT_SENSORS
  // TODO: Implement SENT
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

  // === SAFETY CHECKS WITH LATCHING ===
  bool currentHealthy = true;
  if (compSpeedL_rpm > COMP_SPEED_MAX || compSpeedR_rpm > COMP_SPEED_MAX ||
      egtTemp > EGT_MAX || empPressure > EMP_MAX) {
    currentHealthy = false;
  }

  if (!currentHealthy) {
    if (systemHealthy) {  // First trip
      safetyTripTime = millis();
    }
    systemHealthy = false;
  } else {
    // Recovery logic
    if (SAFETY_LATCH_TIME_MS == 0) {
      // Permanent latch - requires power cycle
      systemHealthy = false;
    } else if ((millis() - safetyTripTime) > SAFETY_LATCH_TIME_MS) {
      systemHealthy = true;  // Auto-recovery after timer
      safetyTripTime = 0;
    }
  }

  // === CORE HUNGRY MONKEY LOGIC ===
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
        vntTargetPctClosed = vntTargetPctClosed * (1.0 - handoverFactor);
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

  // === LOGGING ===
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
