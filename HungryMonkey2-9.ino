/*
  Hungry Monkey v2.9 (16x16 maps) - Final
  - Version print at startup
  - Dynamic telemetry broadcast via getBroadcastIP()
  - Config persistence: load/save config.txt on SD
  - Heartbeat LED (LED_BUILTIN)
  - Full 16x16 map validation (axes + data cells)
  - UDP buffer 2048 bytes with safe multi-line parsing
  - Per-line ACK during MAP WRITE
  - Safety hysteresis with recovery logic
  - Watchdog 500 ms
  - Atomic SD writes (tmp -> rename)
  - Hardened SRAM timing via GPIO9_DR / GPIO6_DR (board-specific)
  - All markdown/link artifacts removed
  - Ready for Teensy 4.1 deployment (verify GPIO mapping)
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
IPAddress lastTelemetryTarget = IPAddress(0,0,0,0);

// -----------------------------
// Hardware & libs
// -----------------------------
Si5351 si5351;
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
WDT_T4<WDT1> wdt;
FreqMeasureMulti compSensor;

// -----------------------------
// State & Config (defaults)
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
uint32_t cfg_overspeedRPM = 220000;
bool cfg_vntInverted = false;
bool cfg_singleTurbo = false;
float cfg_lpgScaleFactor = 1.0f;
float cfg_driveRatioLimit = 1.8f;

// -----------------------------
// Map & injector models
// -----------------------------
struct TuningTable {
  vector<float> tAxis;
  vector<float> rAxis;
  vector<vector<float>> data;
  bool loaded = false;

  float interpolate(float t, float r) {
    if (!loaded || tAxis.size() < 2 || rAxis.size() < 2) return 0.0f;
    t = constrain(t, tAxis.front(), tAxis.back());
    r = constrain(r, rAxis.front(), rAxis.back());

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

    return v00 * (1 - fT) * (1 - fR) + v10 * fT * (1 - fR) +
           v01 * (1 - fT) * fR + v11 * fT * fR;
  }
};

TuningTable vntMap, lpgMap, wgMap;

struct InjectorChannel {
  uint8_t pulse = 0;
  uint8_t meta = 0;
  float scale = 1.0f;
  uint8_t phase = 0;
  bool enable = true;
};
InjectorChannel inj[INJECTOR_CHANNELS];

// -----------------------------
// Map write state
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
  for (size_t i = 1; i < v.size(); ++i)
    if (v[i] <= v[i-1]) return false;
  return true;
}

bool is_valid_ipv4_octet(int x) { return x >= 0 && x <= 255; }

bool validate_and_set_ip(const String &s) {
  int p1, p2, p3, p4;
  if (sscanf(s.c_str(), "%d.%d.%d.%d", &p1, &p2, &p3, &p4) != 4) return false;
  if (!is_valid_ipv4_octet(p1) || !is_valid_ipv4_octet(p2) ||
      !is_valid_ipv4_octet(p3) || !is_valid_ipv4_octet(p4)) return false;
  ip = IPAddress(p1, p2, p3, p4);
  cfg_ip = s;
  return true;
}

// -----------------------------
// Config persistence (config.txt)
// -----------------------------
const char* CONFIG_FILE = "config.txt";

void saveConfigToSD() {
  if (!SD.begin(PIN_SD_CS)) return;
  // Remove existing file to ensure clean overwrite
  if (SD.exists(CONFIG_FILE)) SD.remove(CONFIG_FILE);
  File f = SD.open(CONFIG_FILE, FILE_WRITE);
  if (!f) return;
  f.printf("IP=%s\n", cfg_ip.c_str());
  f.printf("OVERSPEED=%u\n", (unsigned)cfg_overspeedRPM);
  f.printf("DRIVE_RATIO_LIMIT=%.3f\n", cfg_driveRatioLimit);
  f.printf("VNT_INVERTED=%d\n", cfg_vntInverted ? 1 : 0);
  f.printf("SINGLE_TURBO=%d\n", cfg_singleTurbo ? 1 : 0);
  f.printf("LPG_SCALE=%.3f\n", cfg_lpgScaleFactor);
  f.close();
  Serial.println("Config saved to SD");
}

void loadConfigFromSD() {
  if (!SD.begin(PIN_SD_CS)) {
    Serial.println("SD not available for config load");
    return;
  }
  if (!SD.exists(CONFIG_FILE)) {
    Serial.println("No config.txt found, using defaults");
    return;
  }
  File f = SD.open(CONFIG_FILE);
  if (!f) {
    Serial.println("Failed to open config.txt");
    return;
  }
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0 || line.startsWith("#")) continue;
    int eq = line.indexOf('=');
    if (eq == -1) continue;
    String key = line.substring(0, eq);
    String val = line.substring(eq + 1);
    key.trim(); val.trim();
    if (key == "IP") validate_and_set_ip(val);
    else if (key == "OVERSPEED") cfg_overspeedRPM = (uint32_t)val.toInt();
    else if (key == "DRIVE_RATIO_LIMIT") cfg_driveRatioLimit = val.toFloat();
    else if (key == "VNT_INVERTED") cfg_vntInverted = (val.toInt() != 0);
    else if (key == "SINGLE_TURBO") cfg_singleTurbo = (val.toInt() != 0);
    else if (key == "LPG_SCALE") cfg_lpgScaleFactor = val.toFloat();
  }
  f.close();
  Serial.println("Config loaded from SD");
}

// -----------------------------
// Dynamic broadcast helper
// -----------------------------
IPAddress getBroadcastIP() {
  return IPAddress(ip[0], ip[1], ip[2], 255);
}

// -----------------------------
// SRAM write (timing hardened)
// -----------------------------
// NOTE: Ensure your PCB maps address/data lines to these GPIO registers.
// If not, replace with digitalWriteFast() mapped to your pins.
void IRAM_ATTR writeSRAM_atomic(uint8_t addr, uint8_t pulse, uint8_t meta) {
  noInterrupts();
  GPIO9_DR = (GPIO9_DR & ~0xF) | (addr & 0xF);
  uint16_t word = ((uint16_t)meta << 8) | pulse;
  GPIO6_DR = (GPIO6_DR & ~0xFFFF) | word;
  digitalWriteFast(PIN_SRAM_WE, LOW);
  asm volatile("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n");
  digitalWriteFast(PIN_SRAM_WE, HIGH);
  interrupts();
}

// -----------------------------
// CSV parsing
// -----------------------------
bool parse_csv_line_to_floats(const String &line, vector<float> &out, bool allow_empty_top_left = false) {
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
      if (allow_empty_top_left && out.empty()) out.push_back(NAN);
      else return false;
    } else {
      bool numeric = true, seenDigit = false, seenDot = false;
      for (size_t i = 0; i < token.length(); ++i) {
        char c = token.charAt(i);
        if (c >= '0' && c <= '9') seenDigit = true;
        else if (c == '.' && !seenDot) seenDot = true;
        else if ((c == '+' || c == '-') && i == 0) continue;
        else { numeric = false; break; }
      }
      if (!numeric || !seenDigit) return false;
      out.push_back(token.toFloat());
    }
    if (comma == -1) break;
    start = comma + 1;
  }
  return true;
}

// -----------------------------
// Map load/write/validate
// -----------------------------
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
    if (!parse_csv_line_to_floats(line, parsed, true)) {
      Serial.printf("loadMap: parse error in %s: %s\n", filename, line.c_str());
      f.close();
      return false;
    }
    rows.push_back(parsed);
  }
  f.close();

  if (rows.size() != MAP_R_SIZE + 1) {
    Serial.printf("loadMap: %s expected %d rows, got %d\n", filename, MAP_R_SIZE + 1, (int)rows.size());
    return false;
  }
  if (rows[0].size() != MAP_T_SIZE + 1) {
    Serial.printf("loadMap: %s header wrong columns\n", filename);
    return false;
  }

  table.tAxis.clear();
  table.rAxis.clear();
  table.data.clear();

  for (size_t c = 1; c < rows[0].size(); ++c) table.tAxis.push_back(rows[0][c]);
  for (size_t r = 1; r < rows.size(); ++r) {
    if (rows[r].size() != MAP_T_SIZE + 1) return false;
    table.rAxis.push_back(rows[r][0]);
    vector<float> drow;
    for (size_t c = 1; c < rows[r].size(); ++c) drow.push_back(rows[r][c]);
    table.data.push_back(drow);
  }

  if (table.tAxis.size() != MAP_T_SIZE || table.rAxis.size() != MAP_R_SIZE ||
      table.data.size() != MAP_R_SIZE) return false;

  for (auto& row : table.data)
    if (row.size() != MAP_T_SIZE) return false;

  if (!is_strictly_increasing(table.tAxis) || !is_strictly_increasing(table.rAxis)) return false;

  table.loaded = true;
  Serial.printf("loadMap: %s loaded OK (16x16)\n", filename);
  return true;
}

bool writeMapToFileAtomic(const char* filename, const vector<String> &lines) {
  String tmp = String(filename) + ".tmp";
  File f = SD.open(tmp.c_str(), FILE_WRITE);
  if (!f) {
    Serial.printf("writeMap: cannot open tmp %s\n", tmp.c_str());
    return false;
  }
  for (const String &l : lines) f.println(l);
  f.flush();
  f.close();
  if (SD.exists(filename)) SD.remove(filename);
  bool ok = SD.rename(tmp.c_str(), filename);
  if (!ok) {
    Serial.printf("writeMap: rename failed %s -> %s\n", tmp.c_str(), filename);
    if (SD.exists(tmp.c_str())) SD.remove(tmp.c_str());
  }
  return ok;
}

bool validate_map_lines_16x16(const vector<String> &lines) {
  if (lines.size() != MAP_R_SIZE + 1) return false;

  vector<float> header;
  if (!parse_csv_line_to_floats(lines[0], header, true) || header.size() != MAP_T_SIZE + 1) return false;

  vector<float> tAxis, rAxis;
  vector<vector<float>> data;
  for (size_t c = 1; c < header.size(); ++c) tAxis.push_back(header[c]);

  for (size_t r = 1; r < lines.size(); ++r) {
    vector<float> row;
    if (!parse_csv_line_to_floats(lines[r], row, false) || row.size() != MAP_T_SIZE + 1) return false;
    rAxis.push_back(row[0]);
    vector<float> drow;
    for (size_t c = 1; c < row.size(); ++c) drow.push_back(row[c]);
    data.push_back(drow);
  }

  if (data.size() != MAP_R_SIZE) return false;
  for (auto &dr : data) if (dr.size() != MAP_T_SIZE) return false;
  return is_strictly_increasing(tAxis) && is_strictly_increasing(rAxis);
}

// -----------------------------
// UDP helpers & command handling
// -----------------------------
void send_udp_ack(const IPAddress &target, const String &cmd) {
  Udp.beginPacket(target, UDP_CMD_PORT);
  Udp.printf("ACK: %s\n", cmd.c_str());
  Udp.endPacket();
}

void send_udp_err(const IPAddress &target, const String &reason) {
  Udp.beginPacket(target, UDP_CMD_PORT);
  Udp.printf("ERR: %s\n", reason.c_str());
  Udp.endPacket();
}

void handle_map_read(const IPAddress &sender, const String &mapName) {
  const char* fname = nullptr;
  if (mapName == "VNT") fname = "vnt_map.csv";
  else if (mapName == "LPG") fname = "lpg_map.csv";
  else if (mapName == "WG")  fname = "wg_map.csv";
  else { send_udp_err(sender, "Unknown map"); return; }

  File f = SD.open(fname);
  if (!f) { send_udp_err(sender, String("Cannot open ") + fname); return; }

  Udp.beginPacket(sender, UDP_CMD_PORT);
  Udp.printf("MAP BEGIN %s\n", mapName.c_str());
  Udp.endPacket();

  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;
    Udp.beginPacket(sender, UDP_CMD_PORT);
    Udp.printf("%s\n", line.c_str());
    Udp.endPacket();
    delay(2);
  }
  f.close();

  Udp.beginPacket(sender, UDP_CMD_PORT);
  Udp.printf("MAP END %s\n", mapName.c_str());
  Udp.endPacket();
  send_udp_ack(sender, String("MAP READ ") + mapName);
}

void handle_map_write_begin(const IPAddress &sender, const String &mapName) {
  if (currentMapWrite != MAP_NONE) { send_udp_err(sender, "Another MAP WRITE in progress"); return; }
  if (mapName == "VNT") currentMapWrite = MAP_VNT;
  else if (mapName == "LPG") currentMapWrite = MAP_LPG;
  else if (mapName == "WG")  currentMapWrite = MAP_WG;
  else { send_udp_err(sender, "Unknown map"); return; }
  mapWriteBuffer.clear();
  send_udp_ack(sender, String("MAP WRITE ") + mapName + " BEGIN");
}

void handle_map_write_end(const IPAddress &sender, const String &mapName) {
  if (currentMapWrite == MAP_NONE) { send_udp_err(sender, "No MAP WRITE in progress"); return; }
  if (!validate_map_lines_16x16(mapWriteBuffer)) {
    mapWriteBuffer.clear();
    currentMapWrite = MAP_NONE;
    send_udp_err(sender, "Map validation failed (must be 16x16, numeric, monotonic)");
    return;
  }
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

  bool ok = false;
  if (currentMapWrite == MAP_VNT) ok = loadMapFromFile("vnt_map.csv", vntMap);
  else if (currentMapWrite == MAP_LPG) ok = loadMapFromFile("lpg_map.csv", lpgMap);
  else if (currentMapWrite == MAP_WG)  ok = loadMapFromFile("wg_map.csv", wgMap);

  mapWriteBuffer.clear();
  currentMapWrite = MAP_NONE;

  if (!ok) { send_udp_err(sender, "Map written but failed to load into memory"); return; }
  send_udp_ack(sender, String("MAP WRITE ") + mapName + " END");
}

void handle_set_injector(const IPAddress &sender, const String &token) {
  int p = token.indexOf(' ');
  if (p == -1) { send_udp_err(sender, "Malformed INJ command"); return; }
  String injPart = token.substring(0, p);
  String rest = token.substring(p+1);
  if (!injPart.startsWith("INJ")) { send_udp_err(sender, "Malformed INJ command"); return; }
  int idx = injPart.substring(3).toInt();
  if (idx < 0 || idx >= INJECTOR_CHANNELS) { send_udp_err(sender, "Invalid injector index"); return; }
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
}

void handle_log_command(const IPAddress &sender, const String &token) {
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
  int sp = token.indexOf(' ');
  if (sp == -1) { send_udp_err(sender, "Malformed SET command"); return; }
  String key = token.substring(0, sp);
  String val = token.substring(sp+1);
  key.trim(); val.trim();
  if (key == "IP") {
    if (!validate_and_set_ip(val)) { send_udp_err(sender, "Invalid IP"); return; }
    saveConfigToSD();
    send_udp_ack(sender, String("SET IP ") + val);
  } else if (key == "OVERSPEED") {
    long v = val.toInt();
    if (v <= 0) { send_udp_err(sender, "Invalid OVERSPEED"); return; }
    cfg_overspeedRPM = (uint32_t)v;
    saveConfigToSD();
    send_udp_ack(sender, String("SET OVERSPEED ") + String(v));
  } else if (key == "DRIVE_RATIO_LIMIT") {
    float f = val.toFloat();
    if (!isfinite(f) || f <= 0.0f) { send_udp_err(sender, "Invalid DRIVE_RATIO_LIMIT"); return; }
    cfg_driveRatioLimit = f;
    saveConfigToSD();
    send_udp_ack(sender, String("SET DRIVE_RATIO_LIMIT ") + String(f));
  } else {
    send_udp_err(sender, "Unknown SET key");
  }
}

void process_udp_command(const IPAddress &sender, const String &cmdline) {
  lastTelemetryTarget = sender;
  String s = cmdline;
  s.trim();
  if (s.length() == 0) return;

  int sp = s.indexOf(' ');
  String first = (sp == -1) ? s : s.substring(0, sp);
  String rest = (sp == -1) ? "" : s.substring(sp + 1);
  first.trim(); rest.trim();

  if (first == "MAP") {
    int sp2 = rest.indexOf(' ');
    if (sp2 == -1) { send_udp_err(sender, "Malformed MAP command"); return; }
    String sub = rest.substring(0, sp2);
    String tail = rest.substring(sp2+1);
    sub.trim(); tail.trim();
    if (sub == "READ") {
      handle_map_read(sender, tail);
    } else if (sub == "WRITE") {
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
    if (rest.startsWith("INJ")) handle_set_injector(sender, rest);
    else handle_config_set(sender, rest);
  } else if (first == "LOG") {
    handle_log_command(sender, rest);
  } else if (first == "STREAM") {
    handle_stream_command(sender, rest);
  } else {
    if (currentMapWrite != MAP_NONE) {
      mapWriteBuffer.push_back(s);
      Udp.beginPacket(sender, UDP_CMD_PORT);
      Udp.print("MAP LINE OK\n");
      Udp.endPacket();
    } else {
      send_udp_err(sender, "Unknown command");
    }
  }
}

// -----------------------------
// Telemetry
// -----------------------------
void sendTelemetryPacket() {
  IPAddress target = (lastTelemetryTarget != IPAddress(0,0,0,0)) ? lastTelemetryTarget : getBroadcastIP();
  char buf[128];
  int healthy = systemHealthy ? 1 : 0;
  int len = snprintf(buf, sizeof(buf), "DATA,%.2f,%.2f,%.1f,%.0f,%.0f,%d\n",
                     mapPressure, empPressure, egtTemp, engineRPM, compSpeedRPM, healthy);
  if (len > 0) {
    Udp.beginPacket(target, UDP_TELEM_PORT);
    Udp.write((uint8_t*)buf, len);
    Udp.endPacket();
  }
}

// -----------------------------
// Sync ISR (8 kHz)
// -----------------------------
void IRAM_ATTR syncLoopISR() {
  float vAdcMAP = analogRead(PIN_MAP) * (3.3f / 1023.0f);
  float pMAP = constrain((vAdcMAP - 0.5f) * 7.5f * 0.0689476f, 0.0f, 30.0f);
  float vAdcEMP = analogRead(PIN_EMP) * (3.3f / 1023.0f);
  float pEMP = constrain((vAdcEMP - 0.5f) * 7.5f * 0.0689476f, 0.0f, 30.0f);
  float rawEGT = analogRead(PIN_EGT) * 0.977f;

  mapPressure = mapPressure * 0.85f + pMAP * 0.15f;
  empPressure = empPressure * 0.90f + pEMP * 0.10f;
  egtTemp = egtTemp * 0.85f + rawEGT * 0.15f;

  bool overBoost = mapPressure > 2.2f;
  bool overEMP = empPressure > 2.0f;
  bool overDrive = (mapPressure > 1.1f) && ((empPressure / mapPressure) > cfg_driveRatioLimit);
  bool overSpeed = compSpeedRPM > cfg_overspeedRPM;

  static bool wasUnhealthy = false;
  if (egtTemp > 900.0f || overBoost || overEMP || overDrive || overSpeed) {
    systemHealthy = false;
    digitalWriteFast(PIN_LOOP_ENABLE, LOW);
    wasUnhealthy = true;
  } else if (wasUnhealthy && egtTemp < 800.0f && mapPressure < 1.8f && compSpeedRPM < cfg_overspeedRPM * 0.9f) {
    systemHealthy = true;
    digitalWriteFast(PIN_LOOP_ENABLE, HIGH);
    wasUnhealthy = false;
  }

  if (systemHealthy) {
    float localTorque = torqueRequest;
    float localRPM = engineRPM;

    float vntDuty = vntMap.interpolate(localTorque, localRPM);
    float lpgPulse = lpgMap.interpolate(localTorque, localRPM);
    float wgDuty = wgMap.interpolate(localTorque, localRPM);

    if (!cfg_singleTurbo && localRPM > 4200.0f) {
      float fade = constrain((localRPM - 4200.0f) / 800.0f, 0.0f, 1.0f);
      vntDuty *= (1.0f - fade);
    }
    if (cfg_vntInverted) vntDuty = 100.0f - vntDuty;

    analogWrite(PIN_VNT, (int)(constrain(vntDuty, 0.0f, 100.0f) * 2.55f));
    analogWrite(PIN_WG, (int)(constrain(wgDuty, 0.0f, 100.0f) * 2.55f));
    analogWrite(PIN_LPG, (int)(constrain(lpgPulse * cfg_lpgScaleFactor, 0.0f, 100.0f) * 2.55f));

    for (uint8_t i = 0; i < INJECTOR_CHANNELS; ++i) {
      float base = localTorque * inj[i].scale;
      uint8_t pulse = (uint8_t)constrain((int)round(base), 0, INJECTOR_PULSE_MAX);
      uint8_t meta = inj[i].enable ? 1 : 0;
      meta |= (inj[i].phase & 0x07) << 1;
      inj[i].pulse = pulse;
      inj[i].meta = meta;
      writeSRAM_atomic(i, pulse, meta);
    }
  }
}

// -----------------------------
// Setup & Loop
// -----------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000);

  // Version print
  Serial.println("Hungry Monkey v2.9 - (c) 2026 posty-au");

  // Heartbeat LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWriteFast(LED_BUILTIN, LOW);

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

  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  si5351.set_freq(100000000ULL, SI5351_CLK0);
  si5351.set_freq(800000ULL, SI5351_CLK1);

  // Load config and maps if SD available
  if (SD.begin(PIN_SD_CS)) {
    loadConfigFromSD();
    loadMapFromFile("vnt_map.csv", vntMap);
    loadMapFromFile("lpg_map.csv", lpgMap);
    loadMapFromFile("wg_map.csv", wgMap);
    logFile = SD.open("MONKEY.CSV", FILE_WRITE);
  } else {
    Serial.println("SD init failed - continuing with defaults");
    enableSDLogging = false;
  }

  Ethernet.begin(mac, ip);
  delay(100);
  Udp.begin(localPort);

  Can0.begin();
  Can0.setBaudRate(500000);
  compSensor.begin(PIN_COMP_SPEED);
  attachInterrupt(digitalPinToInterrupt(PIN_SYNC), syncLoopISR, RISING);

  wdt.begin(500);

  for (int i = 0; i < INJECTOR_CHANNELS; ++i) {
    inj[i].scale = 1.0f;
    inj[i].phase = i % 8;
    inj[i].enable = true;
  }

  digitalWriteFast(PIN_LOOP_ENABLE, HIGH);
  Serial.println("Hungry Monkey v2.9 ready");
}

void loop() {
  wdt.feed();

  // Heartbeat LED (500 ms)
  static bool ledState = false;
  static uint32_t lastBlink = 0;
  if (millis() - lastBlink > 500) {
    ledState = !ledState;
    digitalWriteFast(LED_BUILTIN, ledState ? HIGH : LOW);
    lastBlink = millis();
  }

  const int UDP_BUF_SZ = 2048;
  static char packetBuffer[UDP_BUF_SZ];
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    int len = Udp.read(packetBuffer, min(packetSize, UDP_BUF_SZ - 1));
    if (len > 0) {
      packetBuffer[len] = '\0';
      String full = String(packetBuffer);
      IPAddress sender = Udp.remoteIP();

      int start = 0;
      while (true) {
        int nl = full.indexOf('\n', start);
        if (nl == -1) break;
        String cmd = full.substring(start, nl);
        cmd.trim();
        if (cmd.length() > 0) {
          Serial.printf("UDP from %s: %s\n", sender.toString().c_str(), cmd.c_str());
          process_udp_command(sender, cmd);
        }
        start = nl + 1;
      }
      if (start < full.length()) {
        String tail = full.substring(start);
        tail.trim();
        if (tail.length() > 0) process_udp_command(sender, tail);
      }
    }
  }

  CAN_message_t msg;
  if (Can0.read(msg) && msg.id == 0x200) {
    uint16_t rpm_raw = (msg.buf[0] << 8) | msg.buf[1];
    uint16_t torque_raw = (msg.buf[2] << 8) | msg.buf[3];
    noInterrupts();
    engineRPM = (float)rpm_raw;
    torqueRequest = (float)torque_raw;
    interrupts();
  }

  if (compSensor.available()) {
    float freq = compSensor.countToFrequency(compSensor.read());
    compSpeedRPM = freq * 60.0f;
  }

  static uint32_t lastStream = 0;
  if (streamEnabled && millis() - lastStream >= streamIntervalMs) {
    sendTelemetryPacket();
    lastStream = millis();
  }

  static uint32_t lastLog = 0;
  if (enableSDLogging && logFile && millis() - lastLog >= logIntervalMs) {
    logFile.printf("%lu,%.2f,%.2f,%.1f,%.0f,%.0f,%d\n",
                   millis(), mapPressure, empPressure, egtTemp,
                   engineRPM, compSpeedRPM, systemHealthy ? 1 : 0);
    logFile.flush();
    lastLog = millis();
  }

  delay(1);
}
