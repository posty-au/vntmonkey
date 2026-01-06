// Hungry Monkey Control System - Teensy 4.1
// Torque-on-demand compound turbo controller
// Version 1.4 - Added SINGLE_TURBO mode toggle for open-source flexibility
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

// === PIN DEFINITIONS ===
// NOTE: Teensy 4.1 is 3.3V logic. DO NOT connect PWM pins directly to actuators/injectors!
// Use level shifters (to 5V/12V) or MOSFET drivers for all outputs to handle current/voltage.
// VNT Actuators: Require PWM Low-Side Driver (ground switching) or 5V/12V signal.
// LPG Injectors: Require Peak-and-Hold driver board or robust MOSFET bank. Current draw ~15-20A total - split into Left/Right banks.
#define PIN_VNT_LEFT      2      // PWM capable
#define PIN_VNT_RIGHT     3
#define PIN_WASTEGATE     4
#define PIN_LPG_LEFT      5      // High-freq PWM for peak-hold simulation (Left bank)
#define PIN_LPG_RIGHT     8      // High-freq PWM for peak-hold simulation (Right bank)

#define PIN_COMP_SPEED_L  6      // FreqMeasureMulti input Left HP
#define PIN_COMP_SPEED_R  7      // Right HP

#define PIN_MAP           A0     // Analog 0-5V (e.g., 4-bar); if USE_SENT_SENSORS, reassign for digital
#define PIN_EGT           A1     // Thermocouple amp output (e.g., 0-5V = 0-1000°C)
#define PIN_EMP           A2     // Exhaust manifold pressure (optional, e.g., 5-bar sensor); if USE_SENT_SENSORS, reassign for digital

#define PIN_SD_CS         10     // SD card chip select (built-in)

// === CONSTANTS ===
const uint32_t LOOP_RATE_HZ = 500;      // Main control loop 500 Hz
const uint32_t LOOP_INTERVAL_US = 1000000 / LOOP_RATE_HZ;

// Placeholder CAN IDs - YOU MUST UPDATE THESE after sniffing!
const uint32_t TORQUE_MSG_ID = 0x200;  // Example - torque request + RPM common in Bosch

// === GLOBAL OBJECTS ===
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
FreqMeasureMulti compSpeedL;
FreqMeasureMulti compSpeedR;

File logFile;

// === CALIBRATION MAPS (loaded from SD) ===
// 3D: Torque (Nm) x RPM → VNT % closed (0% = fully open, 100% = fully closed)
// Edit these for your engine; load from SD for runtime tuning if desired
const int TORQUE_BINS = 11;
const int RPM_BINS = 9;
float torqueAxis[TORQUE_BINS] = {0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000};
float rpmAxis[RPM_BINS] = {1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000};
float vntBaseMap[RPM_BINS][TORQUE_BINS] = {
  // Fill with realistic values later - higher % closed at low RPM/high torque
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
float mapPressure = 0.0;      // bar absolute
float egtTemp = 0.0;
float empPressure = 0.0;      // bar absolute
float torquePrev = 0.0;
unsigned long prevTime = 0;

// Safety flag
bool systemHealthy = true;

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial monitor

  // PWM setup
  analogWriteFrequency(PIN_VNT_LEFT, VNT_FREQ);
  analogWriteFrequency(PIN_VNT_RIGHT, VNT_FREQ);
  analogWriteFrequency(PIN_WASTEGATE, WG_FREQ);
  analogWriteFrequency(PIN_LPG_LEFT, LPG_FREQ);
  analogWriteFrequency(PIN_LPG_RIGHT, LPG_FREQ);
  analogWriteResolution(12); // Fine control

  // Compressor speed inputs
  compSpeedL.begin(PIN_COMP_SPEED_L);
  compSpeedR.begin(PIN_COMP_SPEED_R);

  // CAN bus
  Can0.begin();
  Can0.setBaudRate(500000); // Common automotive CAN speed

  // SD card for maps & logging
  if (!SD.begin(PIN_SD_CS)) {
    Serial.println("SD card failed!");
  } else {
    logFile = SD.open("hungrylog.csv", FILE_WRITE);
    if (logFile) {
      logFile.println("time,torqueReq,RPM,compL,compR,MAP,EGT,EMP,vntDuty,wgDuty,lpgDuty");
    }
  }

  Serial.println("Hungry Monkey Awake! 🐒");
  prevTime = micros();
}

float interpolate3D(float torque, float rpm) {
  // Simple bilinear interpolation on 3D map
  // Find surrounding bins...
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

  // === READ CAN (torque request + RPM) ===
  CAN_message_t msg;
  if (Can0.read(msg)) {
    if (msg.id == TORQUE_MSG_ID) {
      // Placeholder parsing - adjust when real format known
      torqueRequest = (msg.buf[0] << 8 | msg.buf[1]) * 0.1; // example Nm
      engineRPM = (msg.buf[2] << 8 | msg.buf[3]);
    }
  }

  // === READ SENSORS ===
  if (compSpeedL.available()) compSpeedL_rpm = compSpeedL.read() * 60.0; // Freq -> RPM (adjust multiplier if blades ≠1)
  if (compSpeedR.available()) compSpeedR_rpm = compSpeedR.read() * 60.0;

#if USE_SENT_SENSORS
  // TODO: Implement SENT protocol reading for BMW B57 digital sensors (e.g., using bit-banging or library)
  // Placeholder: mapPressure = readSentSensor(PIN_MAP); // User to implement
  // empPressure = readSentSensor(PIN_EMP);
#else
  mapPressure = analogRead(PIN_MAP) * (5.0 / 1023.0) * 0.8 + 0.2; // example scaling to bar abs (adjust per sensor)
  empPressure = analogRead(PIN_EMP) * (5.0 / 1023.0) * 1.25; // Scale to e.g. 5 bar sensor
#endif

  egtTemp = analogRead(PIN_EGT) * (5.0 / 1023.0) * 200.0; // example scaling 0-5V = 0-1000°C (adjust per amp)

  // === SAFETY CHECKS ===
  systemHealthy = true;
  if (compSpeedL_rpm > COMP_SPEED_MAX || compSpeedR_rpm > COMP_SPEED_MAX ||
      egtTemp > EGT_MAX || empPressure > EMP_MAX) {
    systemHealthy = false;
  }

  // === CORE HUNGRY MONKEY LOGIC ===
  float vntTargetPctClosed = 0.0;
  float wgDuty = 0.0;
  float lpgPulse = 0.0;

  if (systemHealthy && torqueRequest > 0 && engineRPM > MIN_ENGINE_RPM) {
    // Base map
    vntTargetPctClosed = interpolate3D(torqueRequest, engineRPM);

    // === VNT LOGIC SELECTION ===
#if SINGLE_TURBO
    // SINGLE TURBO MODE: Disable handover logic
    float avgCompSpeed = compSpeedL_rpm; // Only uses Left sensor
#else
    // COMPOUND MODE: Active Handover to LP Charger
    if (engineRPM > HANDOVER_START_RPM) {
        float handoverFactor = (engineRPM - HANDOVER_START_RPM) / (HANDOVER_END_RPM - HANDOVER_START_RPM); 
        handoverFactor = constrain(handoverFactor, 0.0, 1.0);
        vntTargetPctClosed = vntTargetPctClosed * (1.0 - handoverFactor);
    }
    float avgCompSpeed = (compSpeedL_rpm + compSpeedR_rpm) / 2.0; 
#endif

    // Feed-forward: pedal velocity
    float pedalRate = (torqueRequest - torquePrev) / (LOOP_INTERVAL_US / 1000000.0);
    vntTargetPctClosed += pedalRate * FEED_FORWARD_GAIN; // tune gain
    vntTargetPctClosed = constrain(vntTargetPctClosed, 0, 100);

    // Feedback: compressor speed protection
    if (avgCompSpeed > COMP_SPEED_MAX * COMP_SPEED_TRIM_THRESHOLD) {
      vntTargetPctClosed -= COMP_SPEED_TRIM_AMOUNT; // soften quickly
    }

    // MAP rate damping (surge prevention)
    static float mapPrev = 0.0;
    float mapRate = (mapPressure - mapPrev) / (LOOP_INTERVAL_US / 1000000.0);
    if (mapRate > MAP_RATE_MAX) { // sudden spike
      vntTargetPctClosed -= MAP_SPIKE_TRIM_AMOUNT;
      wgDuty = WG_CRACK_ON_SPIKE; // crack open wastegate
    }
    mapPrev = mapPressure;

    // EMP Safety: Ratio Check - If Drive Pressure is > DRIVE_RATIO_LIMIT x Boost Pressure, we are choking.
    if (empPressure > (mapPressure * DRIVE_RATIO_LIMIT) && mapPressure > MAP_MIN_FOR_EMP_CHECK) {
       vntTargetPctClosed -= EMP_PANIC_TRIM_AMOUNT; // Panic open vanes
       systemHealthy = false;    // Log error but try to save engine
    }

    // NEW: Progressive Chemical Cooling Map
    // We want more LPG as the VGTs work harder (generating heat) 
    // and as RPM climbs (more air mass to cool).
    if (torqueRequest > LPG_MIN_TORQUE && engineRPM > LPG_MIN_RPM) {
       // Base duty cycle starts at LPG_BASE_DUTY and scales with Torque
       float lpgDuty = LPG_BASE_DUTY + ((torqueRequest - LPG_MIN_TORQUE) / (1000.0 - LPG_MIN_TORQUE)) * LPG_TORQUE_SCALE; 
       
       // Add extra cooling during the "Tabletop" high-heat zone
       if (engineRPM >= TABLETOP_START_RPM && engineRPM <= TABLETOP_END_RPM) {
          lpgDuty += LPG_TABLETOP_BOOST; 
       }
       
       lpgPulse = constrain(lpgDuty, 0.0, LPG_MAX_DUTY); // Cap at max duty to save injector coils
       lpgPulse *= LPG_SCALE_FACTOR; // Apply user scaling
       lpgPulse = constrain(lpgPulse, 0.0, 1.0); // Re-constrain after scaling
    } else {
       lpgPulse = 0.0;
    }
  } else {
    // Fail-safe
    vntTargetPctClosed = 0; // fully open vanes
    wgDuty = 100;           // fully open wastegate
  }

  torquePrev = torqueRequest;

  // Convert % closed to PWM duty (configurable inversion)
  // If VNT_INVERTED: high duty = open (90% open, 10% closed)
  // Else: low duty = open (10% open, 90% closed)
  float vntDuty = map(vntTargetPctClosed, 0, 100, VNT_INVERTED ? 90 : 10, VNT_INVERTED ? 10 : 90); // adjust per your actuator calibration

  // === OUTPUT ACTUATORS ===
  analogWrite(PIN_VNT_LEFT,  (uint16_t)(vntDuty / 100.0 * 4095));
  analogWrite(PIN_VNT_RIGHT, (uint16_t)(vntDuty / 100.0 * 4095));
  analogWrite(PIN_WASTEGATE, (uint16_t)(wgDuty / 100.0 * 4095));
  analogWrite(PIN_LPG_LEFT,  (uint16_t)(lpgPulse * 4095));
  analogWrite(PIN_LPG_RIGHT, (uint16_t)(lpgPulse * 4095));

  // === LOGGING ===
  Serial.printf("%.1f,%.0f,%.0f,%.0f,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f\n",
                torqueRequest, engineRPM, compSpeedL_rpm, compSpeedR_rpm,
                mapPressure, egtTemp, empPressure, vntDuty, wgDuty, lpgPulse * 100.0);

  if (logFile) {
    logFile.printf("%lu,%.1f,%.0f,%.0f,%.0f,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f\n",
                   millis(), torqueRequest, engineRPM, compSpeedL_rpm, compSpeedR_rpm,
                   mapPressure, egtTemp, empPressure, vntDuty, wgDuty, lpgPulse * 100.0);
    logFile.flush(); // every loop OK for tuning, reduce later
  }
}