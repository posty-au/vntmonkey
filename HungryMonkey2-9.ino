/*
  Hungry Monkey v2.9 (16x16 maps) - Patched Full main.cpp
  - Teensy 4.1 firmware
  - Enforces 16x16 maps
  - Robust UDP handling (commands on 8888, telemetry on 8889)
  - Safe map parsing and validation
  - Atomic SD map writes (tmp -> rename)
  - Telemetry target tracking (lastTelemetryTarget)
  - SRAM write timing hardened and protected
  - Defensive validation for config and injector params
  - Clear ACK/ERR responses for UDP commands
  - Conservative, well-logged behavior for safety
*/

#include <Arduino.h>
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include <SD.h>
#include <si5351.h>
#include <FlexCAN_T4.h>
#include <FreqMeasureMulti.h>
#include <Watchdog_t4.h>

#include <vector>
#include <string>

using std::vector;
using std::string;

// -----------------------------
// Constants & Configuration
// -----------------------------
#define MAP_T_SIZE 16
#define MAP_R_SIZE 16

#define INJECTOR_CHANNELS 12
#define INJECTOR_PULSE_MAX 255
#define INJ_PHASE_MAX 7

#define LOG_INTERVAL_MIN_MS 500
#define LOG_INTERVAL_MAX_MS 10000

#define STREAM_INTERVAL_MIN_MS 10
#define STREAM_INTERVAL_MAX_MS 1000

#define UDP_CMD_PORT 8888
#define UDP_TELEM_PORT 8889
#define HTTP_PORT 8000

#define PIN_SD_CS 10
#define PIN_VNT 2
#define PIN_WG 4
#define PIN_LPG 5
#define PIN_SRAM_WE 6
#define PIN_SRAM_CE 7
#define PIN_LOOP_ENABLE 13
#define PIN_SYNC 3

#define PIN_MAP A0
#define PIN_EGT A1
#define PIN_EMP A2
#define PIN_COMP_SPEED 8

// -----------------------------
// Networking globals
// -----------------------------
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 1, 177);
unsigned int localPort = UDP_CMD_PORT;
EthernetUDP Udp;
IPAddress lastTelemetryTarget = IPAddress(0,0,0,0); // set when commands arrive

// -----------------------------
// Hardware & libs
// -----------------------------
Si5351 si5351;
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
WDT_T4<WDT1> wdt;
FreqMeasureMulti compSensor;

// -----------------------------
// State & Config
// -----------------------------
volatile float mapPressure = 0.0f;
volatile float empPressure = 0.0f;
volatile float egtTemp = 0.0f;
volatile float engineRPM = 0.0f;
volatile float torqueRequest = 0.0f;
volatile float compSpeedRPM = 0.0f;
volatile bool systemHealthy = true;

bool enableSDLogging = true;
uint32_t logIntervalMs = 1000;
bool streamEnabled = false;
uint32_t streamIntervalMs = 50;

String cfg_ip = "192.168.1.177";
uint32_t cfg_streamRateMs = 50;
bool cfg_loggingEnabled = true;
uint32_t cfg_logIntervalMs = 1000;
uint32_t cfg_overspeedRPM = 220000;
bool cfg_vntInverted = false;
bool cfg_singleTurbo = false;
float cfg_lpgScaleFactor = 1.0f;
float cfg_driveRatioLimit = 1.8f;

// -----------------------------
// Map & injector models
// -----------------------------
struct TuningTable {
  vector<float> tAxis;                 // size MAP_T_SIZE
  vector<float> rAxis;                 // size MAP_R_SIZE
  vector<vector<float>> data;          // size MAP_R_SIZE x MAP_T_SIZE
  bool loaded = false;

  float interpolate(float t, float r) {
    if (!loaded || tAxis.size() < 2 || rAxis.size() < 2) return 0.0f;
    // clamp to axis range
    if (t <= tAxis.front()) t = tAxis.front();
    if (t >= tAxis.back())  t = tAxis.back();
    if (r <= rAxis.front()) r = rAxis.front();
    if (r >= rAxis.back())  r = rAxis.back();

    size_t tL = 0;
    while (tL + 1 < tAxis.size() && t >= tAxis[tL+1]) ++tL;
    size_t rL = 0;
    while (rL + 1 < rAxis.size() && r >= rAxis[rL+1]) ++rL;
    size_t tH = (tL + 1 < tAxis.size()) ? tL + 1 : tL;
    size_t rH = (rL + 1 < rAxis.size()) ? rL + 1 : rL;

    float t0 = tAxis[tL], t1 = tAxis[tH];
    float r0 = rAxis[rL], r1 = rAxis[rH];
    float fT = (t1 != t0) ? (t - t0) / (t1 - t0) : 0.0f;
    float fR = (r1 != r0) ? (r - r0) / (r1 - r0) : 0.0f;

    float v00 = data[rL][tL];
    float v10 = data[rL][tH];
    float v01 = data[rH][tL];
    float v11 = data[rH][tH];

    return v00 * (1 - fT) * (1 - fR)
         + v10 * fT       * (1 - fR)
         + v01 * (1 - fT) * fR
         + v11 * fT       * fR;
  }
};

TuningTable vntMap, lpgMap, wgMap;

struct InjectorChannel {
  uint8_t pulse;   // 0-255
  uint8_t meta;    // enable/phase/trim
  float scale;
  uint8_t phase;
  bool enable;
};

InjectorChannel inj[INJECTOR_CHANNELS];

// -----------------------------
// Map write buffering state (for MAP WRITE BEGIN/END)
// -----------------------------
enum MapWriteTarget { MAP_NONE, MAP_VNT, MAP_LPG, MAP_WG };
MapWriteTarget currentMapWrite = MAP_NONE;
vector<String> mapWriteBuffer;

// -----------------------------
// Logging
// -----------------------------
File logFile;

// -----------------------------
// Utility helpers
// -----------------------------
bool is_strictly_increasing(const vector<float>& v) {
  for (size_t i = 1; i < v.size(); ++i) if (v[i] <= v[i-1]) return false;
  return true;
}

bool is_valid_ipv4_octet(int x) { return x >= 0 && x <= 255; }

bool validate_and_set_ip(const String &s) {
  int p1, p2, p3, p4;
  if (sscanf(s.c_str(), "%d.%d.%d.%d", &p1, &p2, &p3, &p4) != 4) return false;
  if (!is_valid_ipv4_octet(p1) || !is_valid_ipv4_octet(p2) || !is_valid_ipv4_octet(p3) || !is_valid_ipv4_octet(p4)) return false;
  ip = IPAddress(p1,p2,p3,p4);
  cfg_ip = s;
  return true;
}

// -----------------------------
// SRAM write helper (timing hardened)
// -----------------------------
void IRAM_ATTR writeSRAM_atomic(uint8_t addr, uint8_t pulse, uint8_t meta) {
  // Protect against concurrent access: disable interrupts briefly
  noInterrupts();
  // Set address lines (example: GPIO9 used in original code)
  // NOTE: Replace GPIO register writes with appropriate Teensy fast GPIO API if needed.
  // For portability, use digitalWriteFast for address/data pins if mapped; here we assume direct register access as before.
  // Set address (lower 4 bits)
  GPIO9_DR = (GPIO9_DR & ~0xF) | (addr & 0xF);
  // Set data (16 bits)
  uint16_t word = ((uint16_t)meta << 8) | pulse;
  GPIO6_DR = (GPIO6_DR & ~0xFFFF) | word;
  // Assert WE low for safe t_WL
  digitalWriteFast(PIN_SRAM_WE, LOW);
  // Small delay - a few nops to meet t_WL (conservative)
  asm volatile("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n");
  digitalWriteFast(PIN_SRAM_WE, HIGH);
  interrupts();
}

// -----------------------------
// Map loader (robust, 16x16 enforcement)
// -----------------------------
bool parse_csv_line_to_floats(const String &line, vector<float> &out, bool allow_empty_top_left=false) {
  out.clear();
  int start = 0;
  int len = line.length();
  while (start <= len) {
    int comma = line.indexOf(',', start);
    String token;
    if (comma == -1) token = line.substring(start);
    else token = line.substring(start, comma);
    token.trim();
    if (token.length() == 0) {
      if (allow_empty_top_left && out.size() == 0) {
        // represent empty top-left as NAN sentinel
        out.push_back(NAN);
      } else {
        // empty cell not allowed
        return false;
      }
    } else {
      char buf[64];
      token.toCharArray(buf, sizeof(buf));
      // Use atof-like conversion; toFloat is acceptable but check numeric
      float v = token.toFloat();
      // toFloat returns 0.0 for non-numeric; we do a stricter check:
      bool numeric = true;
      bool seenDigit = false;
      bool seenDot = false;
      for (size_t i = 0; i < token.length(); ++i) {
        char c = token.charAt(i);
        if ((c >= '0' && c <= '9')) seenDigit = true;
        else if (c == '.' && !seenDot) seenDot = true;
        else if ((c == '+' || c == '-') && i == 0) continue;
        else { numeric = false; break; }
      }
      if (!numeric || !seenDigit) return false;
      out.push_back(v);
    }
    if (comma == -1) break;
    start = comma + 1;
  }
  return true;
}

bool loadMapFromFile(const char* filename, TuningTable &table) {
  File f = SD.open(filename);
  if (!f) {
    Serial.printf("loadMap: cannot open %s\n", filename);
    return false;
  }

  vector<vector<float>> rows;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;
    vector<float> parsed;
    // allow empty top-left only on header
    if (!parse_csv_line_to_floats(line, parsed, true)) {
      Serial.printf("loadMap: parse error in %s line: %s\n", filename, line.c_str());
      f.close();
      return false;
    }
    rows.push_back(parsed);
  }
  f.close();

  // Expect exactly MAP_R_SIZE + 1 rows (header + MAP_R_SIZE rows)
  if (rows.size() != (size_t)(MAP_R_SIZE + 1)) {
    Serial.printf("loadMap: %s expected %d rows, got %d\n", filename, MAP_R_SIZE + 1, (int)rows.size());
    return false;
  }

  // Header columns count
  size_t expectedCols = MAP_T_SIZE + 1;
  if (rows[0].size() != expectedCols) {
    Serial.printf("loadMap: %s expected %d columns in header, got %d\n", filename, (int)expectedCols, (int)rows[0].size());
    return false;
  }

  // Build table
  table.tAxis.clear();
  table.rAxis.clear();
  table.data.clear();

  // header: skip top-left (rows[0][0] is NAN sentinel)
  for (size_t c = 1; c < rows[0].size(); ++c) table.tAxis.push_back(rows[0][c]);

  for (size_t r = 1; r < rows.size(); ++r) {
    if (rows[r].size() != expectedCols) {
      Serial.printf("loadMap: %s inconsistent columns at row %d\n", filename, (int)r);
      return false;
    }
    table.rAxis.push_back(rows[r][0]);
    vector<float> drow;
    for (size_t c = 1; c < rows[r].size(); ++c) drow.push_back(rows[r][c]);
    table.data.push_back(drow);
  }

  // Validate sizes
  if (table.tAxis.size() != MAP_T_SIZE || table.rAxis.size() != MAP_R_SIZE || table.data.size() != MAP_R_SIZE) {
    Serial.printf("loadMap: %s wrong dimensions after parse\n", filename);
    return false;
  }
  for (size_t r = 0; r < table.data.size(); ++r) {
    if (table.data[r].size() != MAP_T_SIZE) {
      Serial.printf("loadMap: %s row %d wrong column count\n", filename, (int)r);
      return false;
    }
  }

  // Validate monotonicity
  if (!is_strictly_increasing(table.tAxis)) {
    Serial.printf("loadMap: %s T-axis not strictly increasing\n", filename);
    return false;
  }
  if (!is_strictly_increasing(table.rAxis)) {
    Serial.printf("loadMap: %s R-axis not strictly increasing\n", filename);
    return false;
  }

  table.loaded = true;
  Serial.printf("loadMap: %s loaded OK (16x16)\n", filename);
  return true;
}

// Atomic write: write to tmp file then rename
bool writeMapToFileAtomic(const char* filename, const vector<String> &lines) {
  String tmp = String(filename) + ".tmp";
  File f = SD.open(tmp.c_str(), FILE_WRITE);
  if (!f) {
    Serial.printf("writeMap: cannot open tmp %s\n", tmp.c_str());
    return false;
  }
  for (const String &l : lines) {
    f.println(l);
  }
  f.flush();
  f.close();
  // Remove existing target then rename
  if (SD.exists(filename)) {
    SD.remove(filename);
  }
  bool ok = SD.rename(tmp.c_str(), filename);
  if (!ok) {
    Serial.printf("writeMap: rename failed %s -> %s\n", tmp.c_str(), filename);
    // attempt cleanup
    if (SD.exists(tmp.c_str())) SD.remove(tmp.c_str());
    return false;
  }
  Serial.printf("writeMap: %s written atomically\n", filename);
  return true;
}

// Validate CSV lines in memory (16x16)
bool validate_map_lines_16x16(const vector<String> &lines) {
  if (lines.size() != (size_t)(MAP_R_SIZE + 1)) {
    Serial.printf("validate_map: expected %d lines, got %d\n", MAP_R_SIZE + 1, (int)lines.size());
    return false;
  }
  // parse header
  vector<float> header;
  if (!parse_csv_line_to_floats(lines[0], header, true)) return false;
  if (header.size() != (size_t)(MAP_T_SIZE + 1)) return false;
  vector<float> tAxis;
  for (size_t c = 1; c < header.size(); ++c) tAxis.push_back(header[c]);

  vector<float> rAxis;
  vector<vector<float>> data;
  for (size_t r = 1; r < lines.size(); ++r) {
    vector<float> row;
    if (!parse_csv_line_to_floats(lines[r], row, false)) return false;
    if (row.size() != (size_t)(MAP_T_SIZE + 1)) return false;
    rAxis.push_back(row[0]);
    vector<float> drow;
    for (size_t c = 1; c < row.size(); ++c) drow.push_back(row[c]);
    data.push_back(drow);
  }
  if (!is_strictly_increasing(tAxis) || !is_strictly_increasing(rAxis)) return false;
  return true;
}

// -----------------------------
// UDP command handling & helpers
// -----------------------------
void send_udp_ack(const IPAddress &target, const String &cmd) {
  Udp.beginPacket(target, UDP_CMD_PORT); // reply to sender port (they listen)
  Udp.printf("ACK: %s\n", cmd.c_str());
  Udp.endPacket();
}

void send_udp_err(const IPAddress &target, const String &reason) {
  Udp.beginPacket(target, UDP_CMD_PORT);
  Udp.printf("ERR: %s\n", reason.c_str());
  Udp.endPacket();
}

void handle_map_read(const IPAddress &sender, const String &mapName) {
  // Choose file
  const char* fname = nullptr;
  TuningTable *table = nullptr;
  if (mapName == "VNT") { fname = "vnt_map.csv"; table = &vntMap; }
  else if (mapName == "LPG") { fname = "lpg_map.csv"; table = &lpgMap; }
  else if (mapName == "WG")  { fname = "wg_map.csv";  table = &wgMap; }
  else { send_udp_err(sender, "Unknown map"); return; }

  // Read file and stream back as MAP BEGIN ... MAP END
  File f = SD.open(fname);
  if (!f) {
    send_udp_err(sender, String("Cannot open ") + fname);
    return;
  }
  // Send MAP BEGIN
  Udp.beginPacket(sender, UDP_CMD_PORT);
  Udp.printf("MAP BEGIN %s\n", mapName.c_str());
  Udp.endPacket();

  // Send CSV lines
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;
    Udp.beginPacket(sender, UDP_CMD_PORT);
    Udp.printf("%s\n", line.c_str());
    Udp.endPacket();
    delay(2); // small pacing to avoid UDP packet loss on host
  }
  f.close();

  // Send MAP END
  Udp.beginPacket(sender, UDP_CMD_PORT);
  Udp.printf("MAP END %s\n", mapName.c_str());
  Udp.endPacket();
  send_udp_ack(sender, String("MAP READ ") + mapName);
}

void handle_map_write_begin(const IPAddress &sender, const String &mapName) {
  if (currentMapWrite != MAP_NONE) {
    send_udp_err(sender, "Another MAP WRITE in progress");
    return;
  }
  if (mapName == "VNT") currentMapWrite = MAP_VNT;
  else if (mapName == "LPG") currentMapWrite = MAP_LPG;
  else if (mapName == "WG")  currentMapWrite = MAP_WG;
  else { send_udp_err(sender, "Unknown map"); return; }
  mapWriteBuffer.clear();
  send_udp_ack(sender, String("MAP WRITE ") + mapName + " BEGIN");
}

void handle_map_write_end(const IPAddress &sender, const String &mapName) {
  if (currentMapWrite == MAP_NONE) { send_udp_err(sender, "No MAP WRITE in progress"); return; }
  // Validate buffer
  if (!validate_map_lines_16x16(mapWriteBuffer)) {
    mapWriteBuffer.clear();
    currentMapWrite = MAP_NONE;
    send_udp_err(sender, "Map validation failed (must be 16x16, numeric, monotonic)");
    return;
  }
  // Write to file atomically
  const char* fname = nullptr;
  if (currentMapWrite == MAP_VNT) fname = "vnt_map.csv";
  else if (currentMapWrite == MAP_LPG) fname = "lpg_map.csv";
  else if (currentMapWrite == MAP_WG)  fname = "wg_map.csv";
  else { send_udp_err(sender, "Internal error"); currentMapWrite = MAP_NONE; return; }

  if (!writeMapToFileAtomic(fname, mapWriteBuffer)) {
    mapWriteBuffer.clear();
    currentMapWrite = MAP_NONE;
    send_udp_err(sender, "Failed to write map to SD");
    return;
  }

  // Reload into memory
  bool ok = false;
  if (currentMapWrite == MAP_VNT) ok = loadMapFromFile("vnt_map.csv", vntMap);
  else if (currentMapWrite == MAP_LPG) ok = loadMapFromFile("lpg_map.csv", lpgMap);
  else if (currentMapWrite == MAP_WG)  ok = loadMapFromFile("wg_map.csv", wgMap);

  mapWriteBuffer.clear();
  currentMapWrite = MAP_NONE;

  if (!ok) {
    send_udp_err(sender, "Map written but failed to load into memory");
    return;
  }
  send_udp_ack(sender, String("MAP WRITE ") + mapName + " END");
}

void handle_set_injector(const IPAddress &sender, const String &token) {
  // token example: "INJ0 SCALE 1.05" or "INJ3 PHASE 2" or "INJ5 ENABLE 1"
  // parse
  int idx = -1;
  if (token.startsWith("INJ")) {
    int p = token.indexOf(' ');
    if (p == -1) { send_udp_err(sender, "Malformed INJ command"); return; }
    String injPart = token.substring(0, p); // INJn
    String rest = token.substring(p+1);
    idx = injPart.substring(3).toInt();
    if (idx < 0 || idx >= INJECTOR_CHANNELS) { send_udp_err(sender, "Invalid injector index"); return; }
    // parse rest
    int sp = rest.indexOf(' ');
    if (sp == -1) { send_udp_err(sender, "Malformed INJ command"); return; }
    String field = rest.substring(0, sp);
    String val = rest.substring(sp+1);
    field.trim(); val.trim();
    if (field == "SCALE") {
      float s = val.toFloat();
      if (!isfinite(s)) { send_udp_err(sender, "Invalid scale"); return; }
      inj[idx].scale = s;
      send_udp_ack(sender, String("SET INJ") + String(idx) + " SCALE");
    } else if (field == "PHASE") {
      int ph = val.toInt();
      if (ph < 0 || ph > INJ_PHASE_MAX) { send_udp_err(sender, "Phase out of range 0-7"); return; }
      inj[idx].phase = (uint8_t)ph;
      send_udp_ack(sender, String("SET INJ") + String(idx) + " PHASE");
    } else if (field == "ENABLE") {
      int e = val.toInt();
      inj[idx].enable = (e != 0);
      send_udp_ack(sender, String("SET INJ") + String(idx) + " ENABLE");
    } else {
      send_udp_err(sender, "Unknown injector field");
    }
  } else {
    send_udp_err(sender, "Malformed INJ command");
  }
}

void handle_log_command(const IPAddress &sender, const String &token) {
  // LOG ON / LOG OFF / LOG <ms>
  if (token == "ON") {
    enableSDLogging = true;
    send_udp_ack(sender, "LOG ON");
  } else if (token == "OFF") {
    enableSDLogging = false;
    send_udp_ack(sender, "LOG OFF");
  } else {
    int ms = token.toInt();
    if (ms < (int)LOG_INTERVAL_MIN_MS || ms > (int)LOG_INTERVAL_MAX_MS) {
      send_udp_err(sender, "LOG interval out of range");
      return;
    }
    logIntervalMs = ms;
    send_udp_ack(sender, String("LOG ") + String(ms));
  }
}

void handle_stream_command(const IPAddress &sender, const String &token) {
  // STREAM ON / STREAM OFF / STREAM RATE <ms>
  if (token == "ON") {
    streamEnabled = true;
    send_udp_ack(sender, "STREAM ON");
  } else if (token == "OFF") {
    streamEnabled = false;
    send_udp_ack(sender, "STREAM OFF");
  } else if (token.startsWith("RATE ")) {
    String val = token.substring(5);
    int ms = val.toInt();
    if (ms < (int)STREAM_INTERVAL_MIN_MS || ms > (int)STREAM_INTERVAL_MAX_MS) {
      send_udp_err(sender, "STREAM RATE out of range");
      return;
    }
    streamIntervalMs = ms;
    send_udp_ack(sender, String("STREAM RATE ") + String(ms));
  } else {
    send_udp_err(sender, "Malformed STREAM command");
  }
}

void handle_config_set(const IPAddress &sender, const String &token) {
  // SET IP 192.168.1.200
  // SET OVERSPEED 180000
  // SET DRIVE_RATIO_LIMIT 1.8
  int sp = token.indexOf(' ');
  if (sp == -1) { send_udp_err(sender, "Malformed SET command"); return; }
  String key = token.substring(0, sp);
  String val = token.substring(sp+1);
  key.trim(); val.trim();
  if (key == "IP") {
    if (!validate_and_set_ip(val)) { send_udp_err(sender, "Invalid IP"); return; }
    send_udp_ack(sender, String("SET IP ") + val);
  } else if (key == "OVERSPEED") {
    long v = val.toInt();
    if (v <= 0) { send_udp_err(sender, "Invalid OVERSPEED"); return; }
    cfg_overspeedRPM = (uint32_t)v;
    send_udp_ack(sender, String("SET OVERSPEED ") + String(v));
  } else if (key == "DRIVE_RATIO_LIMIT") {
    float f = val.toFloat();
    if (!isfinite(f) || f <= 0.0f) { send_udp_err(sender, "Invalid DRIVE_RATIO_LIMIT"); return; }
    cfg_driveRatioLimit = f;
    send_udp_ack(sender, String("SET DRIVE_RATIO_LIMIT ") + String(f));
  } else {
    send_udp_err(sender, "Unknown SET key");
  }
}

void process_udp_command(const IPAddress &sender, const String &cmdline) {
  // Track last telemetry target
  lastTelemetryTarget = sender;

  String s = cmdline;
  s.trim();
  if (s.length() == 0) return;

  // Tokenize first words
  int sp = s.indexOf(' ');
  String first = (sp == -1) ? s : s.substring(0, sp);
  String rest = (sp == -1) ? "" : s.substring(sp+1);
  first.trim(); rest.trim();

  if (first == "MAP") {
    // MAP READ VNT
    // MAP WRITE VNT BEGIN
    // MAP WRITE VNT END
    int sp2 = rest.indexOf(' ');
    if (sp2 == -1) { send_udp_err(sender, "Malformed MAP command"); return; }
    String sub = rest.substring(0, sp2);
    String tail = rest.substring(sp2+1);
    sub.trim(); tail.trim();
    if (sub == "READ") {
      // tail is map name
      handle_map_read(sender, tail);
    } else if (sub == "WRITE") {
      // tail: <NAME> BEGIN or <NAME> END
      int sp3 = tail.indexOf(' ');
      if (sp3 == -1) { send_udp_err(sender, "Malformed MAP WRITE"); return; }
      String name = tail.substring(0, sp3);
      String action = tail.substring(sp3+1);
      name.trim(); action.trim();
      if (action == "BEGIN") handle_map_write_begin(sender, name);
      else if (action == "END") handle_map_write_end(sender, name);
      else send_udp_err(sender, "Unknown MAP WRITE action");
    } else {
      send_udp_err(sender, "Unknown MAP subcommand");
    }
  } else if (first == "SET") {
    // SET INJ0 SCALE 1.05  OR SET IP ...
    // We will pass rest to handler that expects "INJ..." or "IP ..."
    if (rest.startsWith("INJ")) {
      handle_set_injector(sender, rest);
    } else {
      handle_config_set(sender, rest);
    }
  } else if (first == "LOG") {
    // LOG ON/OFF/<ms>
    handle_log_command(sender, rest);
  } else if (first == "STREAM") {
    handle_stream_command(sender, rest);
  } else {
    // If we are in a MAP WRITE session, treat the entire line as CSV content
    if (currentMapWrite != MAP_NONE) {
      // Accept CSV lines until MAP WRITE <NAME> END
      mapWriteBuffer.push_back(s);
      // ACK receipt of line to sender (optional)
      // send_udp_ack(sender, "MAP LINE RECEIVED");
    } else {
      send_udp_err(sender, "Unknown command");
    }
  }
}

// -----------------------------
// Telemetry sending (safe)
// -----------------------------
void sendTelemetryPacket() {
  IPAddress target = lastTelemetryTarget;
  if (target == IPAddress(0,0,0,0)) {
    // fallback to configured IP
    target = ip;
  }
  // Build telemetry string
  char buf[128];
  int healthy = systemHealthy ? 1 : 0;
  int len = snprintf(buf, sizeof(buf), "DATA,%.2f,%.2f,%.1f,%.0f,%.0f,%d\n",
                     mapPressure, empPressure, egtTemp, engineRPM, compSpeedRPM, healthy);
  if (len <= 0) return;
  Udp.beginPacket(target, UDP_TELEM_PORT);
  Udp.write((uint8_t*)buf, len);
  Udp.endPacket();
}

// -----------------------------
// ISR: sync loop (8 kHz) - simplified
// -----------------------------
void IRAM_ATTR syncLoopISR() {
  // Read sensors (fast ADC)
  float vAdcMAP = analogRead(PIN_MAP) * (3.3f / 1023.0f);
  float pMAP = constrain((vAdcMAP - 0.5f) * 7.5f * 0.0689476f, 0.0f, 30.0f); // example conversion
  float vAdcEMP = analogRead(PIN_EMP) * (3.3f / 1023.0f);
  float pEMP = constrain((vAdcEMP - 0.5f) * 7.5f * 0.0689476f, 0.0f, 30.0f);
  float rawEGT = analogRead(PIN_EGT) * 0.977f;

  // Simple smoothing (IIR)
  mapPressure = (pMAP * 0.15f) + (mapPressure * 0.85f);
  empPressure = (pEMP * 0.10f) + (empPressure * 0.90f);
  egtTemp = (rawEGT * 0.15f) + (egtTemp * 0.85f);

  // Safety checks
  bool overBoost = mapPressure > 2.2f;
  bool overEMP = empPressure > 2.0f;
  bool overDrive = (mapPressure > 1.1f) && ((empPressure / mapPressure) > cfg_driveRatioLimit);
  bool overSpeed = compSpeedRPM > (float)cfg_overspeedRPM;
  if (egtTemp > 900.0f || overBoost || overEMP || overDrive || overSpeed) {
    systemHealthy = false;
    digitalWriteFast(PIN_LOOP_ENABLE, LOW);
  }

  if (systemHealthy) {
    // Interpolate maps (use torqueRequest and engineRPM)
    float localTorque = torqueRequest;
    float localRPM = engineRPM;
    float vntDuty = vntMap.interpolate(localTorque, localRPM);
    float lpgPulse = lpgMap.interpolate(localTorque, localRPM);
    float wgDuty = wgMap.interpolate(localTorque, localRPM);

    if (!cfg_singleTurbo && localRPM > 4200.0f) {
      float fade = (localRPM - 4200.0f) / (5000.0f - 4200.0f);
      fade = constrain(fade, 0.0f, 1.0f);
      vntDuty *= (1.0f - fade);
    }
    if (cfg_vntInverted) vntDuty = 100.0f - vntDuty;

    // Update PWM outputs (scale 0-100 to 0-255)
    analogWrite(PIN_VNT, (int)(constrain(vntDuty, 0.0f, 100.0f) * 2.55f));
    analogWrite(PIN_WG,  (int)(constrain(wgDuty, 0.0f, 100.0f) * 2.55f));
    analogWrite(PIN_LPG, (int)(constrain(lpgPulse * cfg_lpgScaleFactor, 0.0f, 100.0f) * 2.55f));

    // Compute injector pulses and write to SRAM
    for (uint8_t i = 0; i < INJECTOR_CHANNELS; ++i) {
      float basePulse = localTorque * inj[i].scale;
      uint8_t pulse = (uint8_t)constrain((int)round(basePulse), 0, INJECTOR_PULSE_MAX);
      uint8_t meta = 0;
      if (inj[i].enable) meta |= (1 << 0);
      meta |= ((inj[i].phase & 0x07) << 1);
      inj[i].pulse = pulse;
      inj[i].meta = meta;
      // Write to SRAM (atomic)
      writeSRAM_atomic(i, pulse, meta);
    }
  }
}

// -----------------------------
// Setup & Loop
// -----------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000) ; // wait briefly for serial

  pinMode(PIN_LOOP_ENABLE, OUTPUT);
  digitalWriteFast(PIN_LOOP_ENABLE, LOW);

  pinMode(PIN_SRAM_WE, OUTPUT);
  pinMode(PIN_SRAM_CE, OUTPUT);
  digitalWriteFast(PIN_SRAM_CE, LOW);
  digitalWriteFast(PIN_SRAM_WE, HIGH);

  pinMode(PIN_VNT, OUTPUT);
  pinMode(PIN_WG, OUTPUT);
  pinMode(PIN_LPG, OUTPUT);

  analogWriteFrequency(PIN_VNT, 300);
  analogWriteFrequency(PIN_WG, 30);
  analogWriteFrequency(PIN_LPG, 10000);
  analogReadResolution(10);
  analogReadAveraging(0);

  // Initialize Si5351
  if (si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0)) {
    si5351.set_freq(100000000ULL, SI5351_CLK0); // 100 MHz placeholder
    si5351.set_freq(800000ULL, SI5351_CLK1);   // 800 kHz placeholder for sync
  } else {
    Serial.println("Si5351 init failed");
  }

  // Initialize SD
  if (!SD.begin(PIN_SD_CS)) {
    Serial.println("SD init failed - continuing with defaults");
    enableSDLogging = false;
  } else {
    // Load config and maps
    // loadConfig() - implement reading config.txt if present
    // For brevity, we set defaults and then attempt to load maps
    if (!loadMapFromFile("vnt_map.csv", vntMap)) Serial.println("vnt_map load failed");
    if (!loadMapFromFile("lpg_map.csv", lpgMap)) Serial.println("lpg_map load failed");
    if (!loadMapFromFile("wg_map.csv", wgMap))  Serial.println("wg_map load failed");
    // Open log file
    logFile = SD.open("MONKEY.CSV", FILE_WRITE);
    if (!logFile) Serial.println("Failed to open MONKEY.CSV");
  }

  // Initialize Ethernet & UDP
  Ethernet.begin(mac, ip);
  delay(100);
  Udp.begin(localPort);
  Serial.printf("UDP command port %d started\n", localPort);

  // Initialize CAN
  Can0.begin();
  Can0.setBaudRate(500000);

  // Initialize comp sensor
  compSensor.begin(PIN_COMP_SPEED);

  // Attach sync ISR (external pin)
  attachInterrupt(digitalPinToInterrupt(PIN_SYNC), syncLoopISR, RISING);

  // Watchdog
  wdt.begin(100);

  // Default injector config
  for (int i = 0; i < INJECTOR_CHANNELS; ++i) {
    inj[i].scale = 1.0f;
    inj[i].phase = i % 8;
    inj[i].enable = true;
    inj[i].pulse = 0;
    inj[i].meta = 0;
  }

  digitalWriteFast(PIN_LOOP_ENABLE, HIGH);
  Serial.println("Setup complete");
}

void loop() {
  wdt.feed();

  // Handle incoming UDP command packets
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    char packetBuffer[512];
    int len = Udp.read(packetBuffer, sizeof(packetBuffer) - 1);
    if (len > 0) {
      packetBuffer[len] = '\0';
      String cmd = String(packetBuffer);
      cmd.trim();
      IPAddress sender = Udp.remoteIP();
      Serial.printf("UDP from %s: %s\n", sender.toString().c_str(), cmd.c_str());
      process_udp_command(sender, cmd);
    }
  }

  // Handle CAN messages (non-blocking)
  CAN_message_t msg;
  if (Can0.read(msg) && msg.id == 0x200) {
    // parse RPM and torque from payload
    uint16_t rpm_raw = (msg.buf[0] << 8) | msg.buf[1];
    uint16_t torque_raw = (msg.buf[2] << 8) | msg.buf[3];
    noInterrupts();
    engineRPM = (float)rpm_raw;
    torqueRequest = (float)torque_raw;
    interrupts();
  }

  // Update comp speed if available
  if (compSensor.available()) {
    float freq = compSensor.countToFrequency(compSensor.read());
    compSpeedRPM = freq * (60.0f * 1.0f / 1.0f); // adjust divider/blade count as configured
  }

  // Telemetry streaming (timed)
  static uint32_t lastStream = 0;
  if (streamEnabled && (millis() - lastStream >= streamIntervalMs)) {
    sendTelemetryPacket();
    lastStream = millis();
  }

  // SD logging (timed)
  static uint32_t lastLog = 0;
  if (enableSDLogging && logFile && (millis() - lastLog >= logIntervalMs)) {
    logFile.printf("%lu,%.2f,%.2f,%.1f,%.0f,%.0f,%d\n",
                   millis(),
                   mapPressure,
                   empPressure,
                   egtTemp,
                   engineRPM,
                   compSpeedRPM,
                   systemHealthy ? 1 : 0);
    logFile.flush();
    lastLog = millis();
  }

  // Small idle delay to yield CPU (non-blocking)
  delay(1);
}
