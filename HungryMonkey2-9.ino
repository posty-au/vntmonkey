/*
 * Hungry Monkey v2.9 (16x16 maps)
 * Teensy 4.1 + Si5351 + IS62WV25616 + 74HC161/688/74 + LMG3411 GaN
 * 8 kHz control loop + hardware-timed 12-channel sequential injection
 * SD-configured, UDP-tunable, 16x16 multi-map turbo control + telemetry
 */

#include <FlexCAN_T4.h>
#include <SD.h>
#include <si5351.h>
#include <Watchdog_t4.h>
#include <vector>
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include <FreqMeasureMulti.h>

using std::vector;

// ==========================================
// === Tier 1: Static Constants =============
// ==========================================

#define EGT_MAX             900.0f
#define EMP_MAX_SAFETY      2.0f
#define MAP_MAX_SAFETY      2.0f
#define MAP_SMOOTHING_ALPHA 0.15f
#define EMP_SMOOTHING_ALPHA 0.10f
#define EGT_SMOOTHING_ALPHA 0.15f

#define HANDOVER_START_RPM 4200.0f
#define HANDOVER_END_RPM   5000.0f

#define ADC_REF_V      3.3f
#define ADC_COUNTS     1023.0f
#define NPA_DIVIDER    0.733f
#define PSI_TO_BAR     0.0689476f

#define VNT_FREQ 300.0f
#define WG_FREQ  30.0f
#define LPG_FREQ 10000.0f

#define LOG_INTERVAL_MIN_MS  500
#define LOG_INTERVAL_MAX_MS  10000

#define STREAM_INTERVAL_MIN_MS 10
#define STREAM_INTERVAL_MAX_MS 1000

#define INJECTOR_CHANNELS     12
#define INJECTOR_PULSE_MAX    255

#define INJ_META_ENABLE_BIT   0
#define INJ_META_PHASE_SHIFT  1
#define INJ_META_PHASE_MASK   0x0E
#define INJ_META_TRIM_SHIFT   4
#define INJ_META_TRIM_MASK    0xF0

// 16x16 map constraints
#define MAP_T_SIZE 16
#define MAP_R_SIZE 16

// ==========================================
// === Tier 2: Pins & Hardware ==============
// ==========================================
#define SYNC_PIN    3
#define LOOP_ENABLE 13
#define PIN_VNT     2
#define PIN_WG      4
#define PIN_LPG     5
#define SRAM_WE     6
#define SRAM_CE     7
#define PIN_COMP_SPEED 8
#define PIN_SD_CS   10
#define PIN_MAP     A0
#define PIN_EGT     A1
#define PIN_EMP     A2

// ==========================================
// === Tier 3: Global Objects & State =======
// ==========================================
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
Si5351 si5351;
WDT_T4<WDT1> wdt;
File logFile;
FreqMeasureMulti compSensor;

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 1, 177);
unsigned int localPort = 8888;
EthernetUDP Udp;

String   cfg_ip              = "192.168.1.177";
uint32_t cfg_streamRateMs    = 50;
bool     cfg_loggingEnabled  = true;
uint32_t cfg_logIntervalMs   = 1000;
uint8_t  cfg_bladeCount      = 14;
uint8_t  cfg_sensorDivider   = 8;
uint32_t cfg_overspeedRPM    = 220000;
bool     cfg_vntInverted     = false;
bool     cfg_singleTurbo     = false;
float    cfg_lpgScaleFactor  = 1.0f;
float    cfg_driveRatioLimit = 1.8f;

volatile float torqueRequest = 0.0f, engineRPM = 0.0f;
volatile float mapPressure   = 0.0f;
volatile float egtTemp       = 0.0f;
volatile float empPressure   = 0.0f;
volatile float vntDuty       = 0.0f;
volatile float wgDuty        = 0.0f;
volatile float lpgPulse      = 0.0f;
volatile bool  systemHealthy = true;
volatile float compSpeedRPM  = 0.0f;

volatile bool     enableSDLogging   = true;
volatile uint32_t logIntervalMs     = 1000;
volatile bool     streamEnabled     = false;
volatile uint32_t streamIntervalMs  = 50;

struct TuningTable {
    vector<float> tAxis;
    vector<float> rAxis;
    vector<vector<float>> data;
    bool loaded = false;

    float interpolate(float t, float r) {
        if (!loaded || data.empty() || tAxis.size() < 2 || rAxis.size() < 2) return 0.0f;
        size_t tL = 0, rL = 0;
        for (size_t i = 0; i < tAxis.size() - 1; i++) {
            if (t >= tAxis[i + 1]) tL = i;
        }
        for (size_t i = 0; i < rAxis.size() - 1; i++) {
            if (r >= rAxis[i + 1]) rL = i;
        }
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
    uint8_t pulse;
    uint8_t meta;
};

InjectorChannel inj[INJECTOR_CHANNELS];

float injFuelScale[INJECTOR_CHANNELS];
uint8_t injPhase[INJECTOR_CHANNELS];
bool injEnable[INJECTOR_CHANNELS];

enum MapWriteTarget { MAP_NONE, MAP_VNT, MAP_LPG, MAP_WG };
MapWriteTarget currentMapWrite = MAP_NONE;
vector<String> mapWriteBuffer;

// ==========================================
// === Helper: SRAM Bus =====================
// ==========================================
void IRAM_ATTR writeSRAM(uint8_t addr, uint8_t pulse, uint8_t meta) {
    GPIO9_DR = (GPIO9_DR & ~0xF) | (addr & 0xF);
    uint16_t word = ((uint16_t)meta << 8) | pulse;
    GPIO6_DR = (GPIO6_DR & ~0xFFFF) | word;
    digitalWriteFast(SRAM_WE, LOW);
    asm volatile("nop\n\tnop\n\tnop");
    digitalWriteFast(SRAM_WE, HIGH);
}

// ==========================================
// === Helper: Map Loader (16x16) ===========
// ==========================================
bool loadMap(const char* filename, TuningTable& table) {
    File f = SD.open(filename);
    if (!f) return false;
    table.tAxis.clear();
    table.rAxis.clear();
    table.data.clear();
    table.loaded = false;

    int rIdx = 0;
    while (f.available()) {
        String l = f.readStringUntil('\n');
        l.trim();
        if (l.length() == 0) continue;
        vector<float> row;
        int last = 0, comma = l.indexOf(',');
        while (comma != -1) {
            row.push_back(l.substring(last, comma).toFloat());
            last = comma + 1;
            comma = l.indexOf(',', last);
        }
        row.push_back(l.substring(last).toFloat());
        if (rIdx == 0) {
            for (size_t i = 1; i < row.size(); i++) table.tAxis.push_back(row[i]);
        } else {
            table.rAxis.push_back(row[0]);
            vector<float> dRow;
            for (size_t i = 1; i < row.size(); i++) dRow.push_back(row[i]);
            table.data.push_back(dRow);
        }
        rIdx++;
    }
    f.close();

    // Enforce 16x16
    if (table.tAxis.size() != MAP_T_SIZE) return false;
    if (table.rAxis.size() != MAP_R_SIZE) return false;
    if (table.data.size() != MAP_R_SIZE) return false;
    for (size_t i = 0; i < table.data.size(); i++) {
        if (table.data[i].size() != MAP_T_SIZE) return false;
    }

    table.loaded = true;
    return true;
}

// ==========================================
// === Helper: Config Loader ================
// ==========================================
void loadConfig() {
    File f = SD.open("config.txt");
    if (!f) {
        enableSDLogging  = cfg_loggingEnabled;
        logIntervalMs    = cfg_logIntervalMs;
        streamIntervalMs = cfg_streamRateMs;
        for (int i = 0; i < INJECTOR_CHANNELS; i++) {
            injFuelScale[i] = 1.0f;
            injPhase[i]     = i % 8;
            injEnable[i]    = true;
        }
        return;
    }

    for (int i = 0; i < INJECTOR_CHANNELS; i++) {
        injFuelScale[i] = 1.0f;
        injPhase[i]     = i % 8;
        injEnable[i]    = true;
    }

    while (f.available()) {
        String line = f.readStringUntil('\n');
        line.trim();
        if (line.length() == 0 || line.startsWith("#")) continue;

        int eq = line.indexOf('=');
        if (eq < 0) continue;

        String key = line.substring(0, eq);
        String val = line.substring(eq + 1);
        key.trim(); val.trim();

        if (key == "IP") cfg_ip = val;
        else if (key == "STREAM_RATE") cfg_streamRateMs = val.toInt();
        else if (key == "LOGGING") cfg_loggingEnabled = (val.equalsIgnoreCase("ON"));
        else if (key == "LOG_INTERVAL") cfg_logIntervalMs = val.toInt();
        else if (key == "COMP_BLADE_COUNT") cfg_bladeCount = val.toInt();
        else if (key == "COMP_SENSOR_DIVIDER") cfg_sensorDivider = val.toInt();
        else if (key == "COMP_OVERSPEED_RPM") cfg_overspeedRPM = val.toInt();
        else if (key == "VNT_INVERTED") cfg_vntInverted = (val.toInt() == 1);
        else if (key == "SINGLE_TURBO") cfg_singleTurbo = (val.toInt() == 1);
        else if (key == "LPG_SCALE_FACTOR") cfg_lpgScaleFactor = val.toFloat();
        else if (key == "DRIVE_RATIO_LIMIT") cfg_driveRatioLimit = val.toFloat();
        else if (key.startsWith("INJ") && key.endsWith("_SCALE")) {
            int idx = key.substring(3, key.indexOf('_')).toInt();
            if (idx >= 0 && idx < INJECTOR_CHANNELS) injFuelScale[idx] = val.toFloat();
        } else if (key.startsWith("INJ") && key.endsWith("_PHASE")) {
            int idx = key.substring(3, key.indexOf('_')).toInt();
            if (idx >= 0 && idx < INJECTOR_CHANNELS) injPhase[idx] = (uint8_t)val.toInt();
        } else if (key.startsWith("INJ") && key.endsWith("_ENABLE")) {
            int idx = key.substring(3, key.indexOf('_')).toInt();
            if (idx >= 0 && idx < INJECTOR_CHANNELS) injEnable[idx] = (val.toInt() == 1);
        }
    }
    f.close();

    enableSDLogging  = cfg_loggingEnabled;
    logIntervalMs    = constrain(cfg_logIntervalMs, (uint32_t)LOG_INTERVAL_MIN_MS, (uint32_t)LOG_INTERVAL_MAX_MS);
    streamIntervalMs = constrain(cfg_streamRateMs, (uint32_t)STREAM_INTERVAL_MIN_MS, (uint32_t)STREAM_INTERVAL_MAX_MS);

    int p1, p2, p3, p4;
    if (sscanf(cfg_ip.c_str(), "%d.%d.%d.%d", &p1, &p2, &p3, &p4) == 4) {
        ip = IPAddress(p1, p2, p3, p4);
    }
}

// ==========================================
// === 8 kHz ISR ============================
// ==========================================
void IRAM_ATTR syncLoopISR() {
    float vAdcMAP = analogRead(PIN_MAP) * (ADC_REF_V / ADC_COUNTS);
    float vOutMAP = vAdcMAP / NPA_DIVIDER;
    float psiMAP  = (vOutMAP - 0.5f) * 7.5f;
    psiMAP = constrain(psiMAP, 0.0f, 30.0f);
    float pMAP = psiMAP * PSI_TO_BAR;

    float vAdcEMP = analogRead(PIN_EMP) * (ADC_REF_V / ADC_COUNTS);
    float vOutEMP = vAdcEMP / NPA_DIVIDER;
    float psiEMP  = (vOutEMP - 0.5f) * 7.5f;
    psiEMP = constrain(psiEMP, 0.0f, 30.0f);
    float pEMP = psiEMP * PSI_TO_BAR;

    float rawEGT = analogRead(PIN_EGT) * 0.977f;

    mapPressure = (pMAP * MAP_SMOOTHING_ALPHA) + (mapPressure * (1.0f - MAP_SMOOTHING_ALPHA));
    empPressure = (pEMP * EMP_SMOOTHING_ALPHA) + (empPressure * (1.0f - EMP_SMOOTHING_ALPHA));
    egtTemp     = (rawEGT * EGT_SMOOTHING_ALPHA) + (egtTemp * (1.0f - EGT_SMOOTHING_ALPHA));

    bool overBoost = mapPressure > MAP_MAX_SAFETY;
    bool overEMP   = empPressure > EMP_MAX_SAFETY;
    bool overDrive = (mapPressure > 1.1f) && ((empPressure / mapPressure) > cfg_driveRatioLimit);
    bool overSpeed = compSpeedRPM > (float)cfg_overspeedRPM;

    if (egtTemp > EGT_MAX || overBoost || overEMP || overDrive || overSpeed) {
        systemHealthy = false;
        digitalWriteFast(LOOP_ENABLE, LOW);
    }

    if (systemHealthy) {
        float localTorque = torqueRequest;
        float localRPM    = engineRPM;

        vntDuty  = vntMap.interpolate(localTorque, localRPM);
        lpgPulse = lpgMap.interpolate(localTorque, localRPM);
        wgDuty   = wgMap.interpolate(localTorque, localRPM);

        if (!cfg_singleTurbo && localRPM > HANDOVER_START_RPM) {
            float fade = (localRPM - HANDOVER_START_RPM) /
                         (HANDOVER_END_RPM - HANDOVER_START_RPM);
            fade = constrain(fade, 0.0f, 1.0f);
            vntDuty *= (1.0f - fade);
        }

        if (cfg_vntInverted) vntDuty = 100.0f - vntDuty;

        for (uint8_t i = 0; i < INJECTOR_CHANNELS; i++) {
            float basePulse = localTorque * injFuelScale[i];
            uint8_t pulse = (uint8_t)constrain(basePulse, 0.0f, (float)INJECTOR_PULSE_MAX);

            uint8_t meta = 0;
            if (injEnable[i]) meta |= (1 << INJ_META_ENABLE_BIT);
            meta |= ((injPhase[i] & 0x07) << INJ_META_PHASE_SHIFT);

            inj[i].pulse = pulse;
            inj[i].meta  = meta;

            writeSRAM(i, pulse, meta);
        }

        analogWrite(PIN_VNT, (int)(constrain(vntDuty, 0.0f, 100.0f) * 2.55f));
        analogWrite(PIN_WG,  (int)(constrain(wgDuty,  0.0f, 100.0f) * 2.55f));
        analogWrite(PIN_LPG, (int)(constrain(lpgPulse * cfg_lpgScaleFactor, 0.0f, 100.0f) * 2.55f));
    }
}

// ==========================================
// === Setup & Loop (unchanged except maps) =
// ==========================================
void setup() {
    Serial.begin(115200);

    pinMode(LOOP_ENABLE, OUTPUT);
    digitalWrite(LOOP_ENABLE, LOW);

    pinMode(PIN_VNT, OUTPUT);
    pinMode(PIN_WG,  OUTPUT);
    pinMode(PIN_LPG, OUTPUT);

    GPIO6_GDIR |= 0xFFFF;
    GPIO9_GDIR |= 0xF;
    pinMode(SRAM_WE, OUTPUT);
    pinMode(SRAM_CE, OUTPUT);
    digitalWrite(SRAM_CE, LOW);
    digitalWrite(SRAM_WE, HIGH);

    if (si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0)) {
        si5351.set_freq(100000000ULL, SI5351_CLK0);
        si5351.set_freq(800000ULL,    SI5351_CLK1);
    }

    if (SD.begin(PIN_SD_CS)) {
        loadConfig();
        loadMap("vnt_map.csv", vntMap);
        loadMap("lpg_map.csv", lpgMap);
        loadMap("wg_map.csv",  wgMap);
        logFile = SD.open("MONKEY.CSV", FILE_WRITE);
    } else {
        loadMap("vnt_map.csv", vntMap);
        loadMap("lpg_map.csv", lpgMap);
        loadMap("wg_map.csv",  wgMap);
        enableSDLogging = false;
    }

    Ethernet.begin(mac, ip);
    Udp.begin(localPort);

    analogWriteFrequency(PIN_VNT, VNT_FREQ);
    analogWriteFrequency(PIN_WG,  WG_FREQ);
    analogWriteFrequency(PIN_LPG, LPG_FREQ);
    analogReadResolution(10);
    analogReadAveraging(0);

    Can0.begin();
    Can0.setBaudRate(500000);

    compSensor.begin(PIN_COMP_SPEED);

    attachInterrupt(digitalPinToInterrupt(SYNC_PIN), syncLoopISR, RISING);

    wdt.begin(100);

    digitalWrite(LOOP_ENABLE, HIGH);
}

void loop() {
    wdt.feed();

    int packetSize = Udp.parsePacket();
    if (packetSize) {
        char buf[256];
        int len = Udp.read(buf, sizeof(buf) - 1);
        buf[len] = '\0';

        String cmd = String(buf);
        cmd.trim();

        // (same UDP command handling as previous v2.9, unchanged)
        // ... omitted here for brevity in this answer, but in your actual code
        // keep the full command handler you already approved.
        // Only map loading/enforcement changed.
    }

    if (compSensor.available()) {
        float freq = compSensor.countToFrequency(compSensor.read());
        compSpeedRPM = freq * (60.0f * (float)cfg_sensorDivider / (float)cfg_bladeCount);
    }

    static uint32_t lastStream = 0;
    if (streamEnabled && millis() - lastStream >= streamIntervalMs) {
        Udp.beginPacket(Udp.remoteIP(), 8889);
        Udp.printf("DATA,%.2f,%.2f,%.1f,%.0f,%.0f,%d\n",
                   mapPressure,
                   empPressure,
                   egtTemp,
                   engineRPM,
                   compSpeedRPM,
                   systemHealthy ? 1 : 0);
        Udp.endPacket();
        lastStream = millis();
    }

    CAN_message_t msg;
    if (Can0.read(msg) && msg.id == 0x200) {
        float r = (float)((msg.buf[0] << 8) | msg.buf[1]);
        float t = (float)((msg.buf[2] << 8) | msg.buf[3]);
        noInterrupts();
        engineRPM     = r;
        torqueRequest = t;
        interrupts();
    }

    static uint32_t lastLog = 0;
    if (enableSDLogging && logFile && millis() - lastLog >= logIntervalMs) {
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
}
