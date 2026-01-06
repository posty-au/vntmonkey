// Hungry Monkey Control System - Teensy 4.1
// Torque-on-demand compound turbo controller
// Version 2.0 - User-definable SD write/flush interval + all previous features
// (c) 2026 - Built with love for the Monkey 🐒
// Open-source friendly: Fully configurable via SD maps + calibration defines
// License: MIT - Feel free to fork, modify, and share!

#include <FlexCAN_T4.h>
#include <FreqMeasureMulti.h>
#include <SD.h>
#include <SPI.h>
#include <elapsedMillis.h>
#include <Watchdog_t4.h>
#include <vector>

using std::vector;

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

// Watchdog
#define WATCHDOG_TIMEOUT_MS 100

// Diagnostics
#define ENABLE_BOOT_SWEEP true
#define ENABLE_SERIAL_DIAG true

// === NEW: USER-DEFINABLE SD LOGGING INTERVAL ===
#define SD_LOG_EVERY_N_LOOPS 50      // How often to write a line to SD (default 50 = 10 Hz at 500 Hz loop)
#define SD_FLUSH_EVERY_N_WRITES 20   // How many lines to buffer before flushing (default 20 = ~2 seconds at 10 Hz)

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
WDT_T4<WDT1> wdt;

File logFile;

// === DYNAMIC MAPS FROM SD ===
vector<float> torqueAxis;
vector<float> rpmAxis;
vector<vector<float>> vntBaseMap;

vector<float> lpgTorqueAxis;
vector<float> lpgRpmAxis;
vector<vector<float>> lpgMap;

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

// SD logging counters
static int logCounter = 0;
static int flushCounter = 0;

bool diagnosticModeActive = false;

// === MAP LOADING, INTERPOLATION, DIAGNOSTICS, WATCHDOG (unchanged from v1.9) ===
// ... [All functions from previous version remain identical] ...

// === MAP LOADING ===
bool loadCSVMap(const char* filename, vector<float>& tAxis, vector<float>& rAxis, vector<vector<float>>& mapOut) {
  File mapFile = SD.open(filename);
  if (!mapFile) {
    Serial.printf("ERROR: %s not found on SD card\n", filename);
    return false;
  }

  tAxis.clear();
  rAxis.clear();
  mapOut.clear();

  int rowIndex = 0;
  while (mapFile.available()) {
    String line = mapFile.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;

    vector<float> rowValues;
    int lastComma = 0;
    int commaPos = line.indexOf(',');
    while (commaPos != -1) {
      String cell = line.substring(lastComma, commaPos);
      cell.trim();
      rowValues.push_back(cell.toFloat());
      lastComma = commaPos + 1;
      commaPos = line.indexOf(',', lastComma);
    }
    String lastCell = line.substring(lastComma);
    lastCell.trim();
    if (lastCell.length() > 0) rowValues.push_back(lastCell.toFloat());

    if (rowIndex == 0) {
      for (size_t i = 1; i < rowValues.size(); i++) tAxis.push_back(rowValues[i]);
    } else {
      rAxis.push_back(rowValues[0]);
      vector<float> mapRow;
      for (size_t i = 1; i < rowValues.size(); i++) mapRow.push_back(rowValues[i]);
      mapOut.push_back(mapRow);
    }
    rowIndex++;
  }
  mapFile.close();
  Serial.printf("%s loaded: %d RPM x %d Torque bins\n", filename,
                (int)rAxis.size(), (int)tAxis.size());
  return true;
}

bool loadVNTMapFromSD() { return loadCSVMap("vnt_map.csv", torqueAxis, rpmAxis, vntBaseMap); }
bool loadLPGMapFromSD() { return loadCSVMap("lpg_map.csv", lpgTorqueAxis, lpgRpmAxis, lpgMap); }

// === INTERPOLATION ===
float interpolateMap(float torque, float rpm,
                     const vector<float>& tAxis, const vector<float>& rAxis,
                     const vector<vector<float>>& mapData) {
  if (tAxis.empty() || rAxis.empty() || mapData.empty()) return 0.0;

  size_t tLow = 0, tHigh = tAxis.size() - 1;
  for (size_t i = 0; i < tAxis.size() - 1; ++i) {
    if (torque < tAxis[i + 1]) { tLow = i; tHigh = i + 1; break; }
  }

  size_t rLow = 0, rHigh = rAxis.size() - 1;
  for (size_t i = 0; i < rAxis.size() - 1; ++i) {
    if (rpm < rAxis[i + 1]) { rLow = i; rHigh = i + 1; break; }
  }

  float v11 = mapData[rLow][tLow];
  float v12 = mapData[rLow][tHigh];
  float v21 = mapData[rHigh][tLow];
  float v22 = mapData[rHigh][tHigh];

  float fracT = (tAxis[tHigh] - tAxis[tLow] != 0)
                    ? (torque - tAxis[tLow]) / (tAxis[tHigh] - tAxis[tLow])
                    : 0.0;
  float fracR = (rAxis[rHigh] - rAxis[rLow] != 0)
                    ? (rpm - rAxis[rLow]) / (rAxis[rHigh] - rAxis[rLow])
                    : 0.0;

  return v11 * (1 - fracT) * (1 - fracR) + v12 * fracT * (1 - fracR) +
         v21 * (1 - fracT) * fracR + v22 * fracT * fracR;
}

float interpolateVNT(float torque, float rpm) {
  return interpolateMap(torque, rpm, torqueAxis, rpmAxis, vntBaseMap);
}

float interpolateLPG(float torque, float rpm) {
  return interpolateMap(torque, rpm, lpgTorqueAxis, lpgRpmAxis, lpgMap);
}

// === DIAGNOSTICS & WATCHDOG (unchanged) ===
// ... [vntSweep, checkSerialCommands, watchdogSetup unchanged] ...

void setup() {
  // ... [all previous setup code unchanged] ...

  if (SD.begin(PIN_SD_CS)) {
    loadVNTMapFromSD();
    loadLPGMapFromSD();
  } else {
    Serial.println("SD card failed!");
  }

  logFile = SD.open("hungrylog.csv", FILE_WRITE);
  if (logFile) {
    logFile.println("time,torqueReq,RPM,compL,compR,MAP,EGT,EMP,vntDuty,wgDuty,lpgDuty,healthy");
  }

  // ... [pre-seed filters, boot sweep, watchdog, welcome message unchanged] ...
}

void loop() {
  wdt.feed();

  unsigned long now = micros();
  if (now - prevTime < LOOP_INTERVAL_US) {
    checkSerialCommands();
    return;
  }
  prevTime = now;

  checkSerialCommands();

  if (diagnosticModeActive) return;

  // ... [all sensor reading, safety, control logic unchanged] ...

  // === LOGGING WITH USER-DEFINABLE INTERVAL ===
  Serial.printf("%.1f,%.0f,%.0f,%.0f,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f,%d\n",
                torqueRequest, engineRPM, compSpeedL_rpm, compSpeedR_rpm,
                mapPressure, egtTemp, empPressure, vntDuty, wgDuty, lpgPulse * 100.0, systemHealthy);

  if (logFile) {
    if (++logCounter >= SD_LOG_EVERY_N_LOOPS) {
      logFile.printf("%lu,%.1f,%.0f,%.0f,%.0f,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f,%d\n",
                     millis(), torqueRequest, engineRPM, compSpeedL_rpm, compSpeedR_rpm,
                     mapPressure, egtTemp, empPressure, vntDuty, wgDuty, lpgPulse * 100.0, systemHealthy);
      logCounter = 0;

      if (++flushCounter >= SD_FLUSH_EVERY_N_WRITES) {
        logFile.flush();
        flushCounter = 0;
      }
    }
  }
}
