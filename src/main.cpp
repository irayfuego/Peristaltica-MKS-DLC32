#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Update.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <WiFiManager.h>
#include <ESPAsyncWebServer.h>
#include "stepper595.h"
#include <NimBLEDevice.h>
#include <time.h>
extern "C" {
    #include "freertos/FreeRTOS.h"
    #include "freertos/timers.h"
    #include "freertos/semphr.h"
    #include "esp_task_wdt.h"
}
#include <AsyncMqttClient.h>

#ifndef BUILD_VERSION
#define BUILD_VERSION "dlc32-1.0"
#endif

#define NUM_CH 3

// =========================================================
// MUTEXES
// =========================================================
SemaphoreHandle_t prefsMutex   = nullptr;
SemaphoreHandle_t sensorMutex  = nullptr;
SemaphoreHandle_t fsMutex      = nullptr;  // guards LittleFS history writes
volatile bool otaRestartPending = false;   // set true after successful HTTP OTA
#define PREFS_LOCK()    xSemaphoreTake(prefsMutex,  portMAX_DELAY)
#define PREFS_UNLOCK()  xSemaphoreGive(prefsMutex)
#define SENSOR_LOCK()   xSemaphoreTake(sensorMutex, portMAX_DELAY)
#define SENSOR_UNLOCK() xSemaphoreGive(sensorMutex)


// =========================================================
// PREFERENCES + MQTT CONFIG
// =========================================================
Preferences prefs;

#define MQTT_PORT_DEFAULT 1883
String mqttServer;
int    mqttPort = MQTT_PORT_DEFAULT;
String mqttUser;
String mqttPass;
bool   mqttEnabled = false;

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

void loadMqttConfig() {
    PREFS_LOCK();
    prefs.begin("peristaltica", true);
    mqttServer = prefs.getString("mqtt_srv", "");
    mqttPort   = prefs.getInt("mqtt_port", MQTT_PORT_DEFAULT);
    mqttUser   = prefs.getString("mqtt_usr", "");
    mqttPass   = prefs.getString("mqtt_pwd", "");
    prefs.end();
    PREFS_UNLOCK();
    mqttEnabled = mqttServer.length() > 0;
}

void saveMqttConfig(const String& srv, int port, const String& usr, const String& pwd) {
    PREFS_LOCK();
    prefs.begin("peristaltica", false);
    prefs.putString("mqtt_srv", srv);
    prefs.putInt("mqtt_port", port);
    prefs.putString("mqtt_usr", usr);
    prefs.putString("mqtt_pwd", pwd);
    prefs.end();
    PREFS_UNLOCK();
    mqttServer  = srv;  mqttPort = port;
    mqttUser    = usr;  mqttPass = pwd;
    mqttEnabled = srv.length() > 0;
}


// =========================================================
// WEB SERVER
// =========================================================
AsyncWebServer webServer(80);


// =========================================================
// MOTORS — array-indexed (DLC32 v2.1: driven via 74HC595)
// =========================================================
const int SPEED_IN_STEPS_PER_SECOND        = 2000;
const int ACCELERATION_IN_STEPS_PER_SECOND = 800;
const int DECELERATION_IN_STEPS_PER_SECOND = 800;

Stepper595 motors;

long StepsPerMili[NUM_CH];
long StepsToMove[NUM_CH];
long SpeedToMove[NUM_CH];
volatile bool StepperRunning[NUM_CH] = { false, false, false };
int  Progress[NUM_CH] = { 0, 0, 0 };

// "run for duration" support: when set, target the steps but the time is
// computed from a duration (s), not a volume.
bool   DurationMode[NUM_CH] = { false, false, false };
time_t LastRunStart[NUM_CH] = { 0, 0, 0 };

// Daily totals (mL dispensed today). Reset at local midnight.
float    DailyVolume[NUM_CH] = { 0, 0, 0 };
uint16_t DailyVolumeCap[NUM_CH] = { 0, 0, 0 };  // 0 = no cap
int      LastDayOfYear = -1;

const char* CAL_KEY[NUM_CH] = { "spm1", "spm2", "spm3" };
const char* CAP_KEY[NUM_CH] = { "cap1", "cap2", "cap3" };

static bool anyRunning() {
    for (int i = 0; i < NUM_CH; i++) if (StepperRunning[i]) return true;
    return false;
}

// Stepper task callbacks defer publishing to loop() to keep the stepper
// task tiny.
struct PendingMqttEvent {
    bool     pending;
    char     event;     // 'D' = run done, 'S' = stop (emergency)
    uint8_t  channel;
    int      progress;
};
volatile PendingMqttEvent pendingMqtt[NUM_CH] = {{false,0,0,0},{false,0,0,0},{false,0,0,0}};

static void queueEvent(int ch1based, char ev, int ps) {
    int i = ch1based - 1;
    pendingMqtt[i].channel  = ch1based;
    pendingMqtt[i].event    = ev;
    pendingMqtt[i].progress = ps;
    pendingMqtt[i].pending  = true;
}

// Single channel-aware completion callback (Stepper595 passes 1-based ch).
static void onTargetReached(int ch1based) {
    int i = ch1based - 1;
    Progress[i] = 100;
    StepperRunning[i] = false;
    queueEvent(ch1based, 'D', 100);
}

static void onEmergencyStop(int ch1based) {
    int i = ch1based - 1;
    int ps = 0;
    long total = StepsToMove[i];
    if (total > 0) {
        long done = total - motors.stepsRemaining(i);
        if (done < 0) done = 0;
        ps = round(100.0f * done / (float)total);
        Progress[i] = ps;
    }
    queueEvent(ch1based, 'S', ps);
}


// =========================================================
// CALIBRATION + CAP STORAGE
// =========================================================
void loadCalibration() {
    PREFS_LOCK();
    prefs.begin("peristaltica", true);
    for (int i = 0; i < NUM_CH; i++) {
        StepsPerMili[i]   = prefs.getLong(CAL_KEY[i], 1600);
        DailyVolumeCap[i] = prefs.getUShort(CAP_KEY[i], 0);
        if (StepsPerMili[i] <= 0) StepsPerMili[i] = 1600;
    }
    prefs.end();
    PREFS_UNLOCK();
}

void saveCalibration(int ch1based, long spm) {
    if (ch1based < 1 || ch1based > NUM_CH || spm <= 0) return;
    StepsPerMili[ch1based - 1] = spm;
    PREFS_LOCK();
    prefs.begin("peristaltica", false);
    prefs.putLong(CAL_KEY[ch1based - 1], spm);
    prefs.end();
    PREFS_UNLOCK();
}

void saveDailyCap(int ch1based, uint16_t cap) {
    if (ch1based < 1 || ch1based > NUM_CH) return;
    DailyVolumeCap[ch1based - 1] = cap;
    PREFS_LOCK();
    prefs.begin("peristaltica", false);
    prefs.putUShort(CAP_KEY[ch1based - 1], cap);
    prefs.end();
    PREFS_UNLOCK();
}


// =========================================================
// IRRIGATION HISTORY  (LittleFS CSV — last 100 events)
// =========================================================
#define HIST_FILE "/hist.csv"
#define HIST_MAX  100

struct HistEvent {
    uint32_t ts;       // unix timestamp
    uint8_t  ch;       // 1-3
    float    vol;      // mL
    char     trig[8];  // "manual","sched","mqtt"
};
static HistEvent histBuf[HIST_MAX];
static int histCount = 0;
static int histHead  = 0;   // next write slot (circular)

#define FS_LOCK()   xSemaphoreTake(fsMutex, portMAX_DELAY)
#define FS_UNLOCK() xSemaphoreGive(fsMutex)

// Write current buffer to file — caller must hold fsMutex.
static void saveHistory() {
    File f = LittleFS.open(HIST_FILE, "w");
    if (!f) return;
    int n = min(histCount, HIST_MAX);
    for (int i = 0; i < n; i++) {
        int idx = ((histHead - n + i) % HIST_MAX + HIST_MAX) % HIST_MAX;
        f.printf("%u,%d,%.1f,%s\n",
                 histBuf[idx].ts, (int)histBuf[idx].ch,
                 histBuf[idx].vol, histBuf[idx].trig);
    }
    f.close();
}

void loadHistory() {
    File f = LittleFS.open(HIST_FILE, "r");
    if (!f) { Serial.println("History: no file yet"); return; }
    histCount = 0; histHead = 0;
    char line[48];
    while (f.available() && histCount < HIST_MAX) {
        int len = f.readBytesUntil('\n', line, 47);
        line[len] = 0;
        if (len < 5) continue;
        HistEvent e = {};
        unsigned long ts; int ch; float vol; char trig[8] = "?";
        if (sscanf(line, "%lu,%d,%f,%7s", &ts, &ch, &vol, trig) >= 3) {
            e.ts = (uint32_t)ts; e.ch = (uint8_t)ch; e.vol = vol;
            strlcpy(e.trig, trig, 8);
            histBuf[histHead] = e;
            histHead = (histHead + 1) % HIST_MAX;
            histCount++;
        }
    }
    f.close();
    Serial.printf("History: loaded %d events\n", histCount);
}

void logHistoryEvent(int ch1based, float vol, const char* trig) {
    if (!trig || strcmp(trig, "cal") == 0 || vol < 0.01f) return;
    if (!fsMutex) return;
    FS_LOCK();
    histBuf[histHead].ts  = (uint32_t)time(nullptr);
    histBuf[histHead].ch  = (uint8_t)ch1based;
    histBuf[histHead].vol = vol;
    strlcpy(histBuf[histHead].trig, trig, 8);
    histHead = (histHead + 1) % HIST_MAX;
    if (histCount < HIST_MAX) histCount++;
    saveHistory();
    FS_UNLOCK();
}

void clearHistory() {
    if (!fsMutex) return;
    FS_LOCK();
    histCount = 0; histHead = 0;
    LittleFS.remove(HIST_FILE);
    FS_UNLOCK();
}


// =========================================================
// VACATION MODE
// =========================================================
bool   vacationMode  = false;
time_t vacationUntil = 0;   // 0 = indefinite; >0 = auto-resume timestamp

void loadVacationMode() {
    PREFS_LOCK();
    prefs.begin("peristaltica", true);
    vacationMode  = prefs.getBool("vacMode",  false);
    vacationUntil = (time_t)prefs.getUInt("vacUntil", 0);
    prefs.end();
    PREFS_UNLOCK();
}

void saveVacationMode() {
    PREFS_LOCK();
    prefs.begin("peristaltica", false);
    prefs.putBool("vacMode",  vacationMode);
    prefs.putUInt("vacUntil", (uint32_t)vacationUntil);
    prefs.end();
    PREFS_UNLOCK();
}


// =========================================================
// BLE SENSORS — Mi Flora
// =========================================================
#define MAX_SCAN_RESULTS 20
#define SENSOR_OFFLINE_HOURS 12     // schedule treats sensor as offline after this

struct SensorConfig {
    char    mac[18];
    char    name[40];
    uint8_t threshold;   // moisture % — 0 = disabled
    bool    assigned;
};

struct SensorReading {
    float    temperature;
    uint8_t  moisture;
    uint32_t light;
    uint16_t conductivity;
    uint8_t  battery;     // 0-100
    bool     valid;
    time_t   timestamp;
};

struct BLEScanResult {
    char addr[18];
    char name[48];
    int  rssi;
};

SensorConfig  sensorCfg[NUM_CH];
SensorReading sensorData[NUM_CH];

BLEScanResult scanResults[MAX_SCAN_RESULTS];
volatile int  scanResultCount = 0;
volatile bool bleScanning     = false;
volatile bool bleScanReq      = false;
volatile bool bleReadReq      = false;

void saveSensorConfig(int ch) {
    char key[12]; sprintf(key, "sensor_%d", ch);
    StaticJsonDocument<192> doc;
    doc["mac"]  = sensorCfg[ch].mac;
    doc["name"] = sensorCfg[ch].name;
    doc["thr"]  = sensorCfg[ch].threshold;
    doc["asgn"] = sensorCfg[ch].assigned;
    char buf[192]; serializeJson(doc, buf);
    PREFS_LOCK();
    prefs.begin("peristaltica", false);
    prefs.putString(key, buf);
    prefs.end();
    PREFS_UNLOCK();
}

void loadSensorConfigs() {
    PREFS_LOCK();
    prefs.begin("peristaltica", true);
    for (int i = 0; i < NUM_CH; i++) {
        sensorCfg[i]  = {{0}, {0}, 0, false};
        sensorData[i] = {0, 0, 0, 0, 0, false, 0};
        char key[12]; sprintf(key, "sensor_%d", i);
        String s = prefs.getString(key, "");
        if (s.length() == 0) continue;
        StaticJsonDocument<192> doc;
        if (deserializeJson(doc, s)) continue;
        strlcpy(sensorCfg[i].mac,  doc["mac"]  | "", 18);
        strlcpy(sensorCfg[i].name, doc["name"] | "", 40);
        sensorCfg[i].threshold = doc["thr"]  | 0;
        sensorCfg[i].assigned  = doc["asgn"] | false;
    }
    prefs.end();
    PREFS_UNLOCK();
}

void deleteSensorConfig(int ch) {
    sensorCfg[ch] = {{0}, {0}, 0, false};
    char key[12]; sprintf(key, "sensor_%d", ch);
    PREFS_LOCK();
    prefs.begin("peristaltica", false);
    prefs.remove(key);
    prefs.end();
    PREFS_UNLOCK();
}

// Read sensor data + battery from a Mi Flora device.
bool readMiFlora(const char* macStr, SensorReading& r) {
    NimBLEAddress addr(macStr);
    NimBLEClient* client = NimBLEDevice::createClient();
    client->setConnectionParams(16, 16, 0, 60);

    if (!client->connect(addr)) {
        NimBLEDevice::deleteClient(client);
        return false;
    }

    NimBLERemoteService* svc =
        client->getService("00001204-0000-1000-8000-00805f9b34fb");
    if (!svc) { client->disconnect(); NimBLEDevice::deleteClient(client); return false; }

    NimBLERemoteCharacteristic* wc =
        svc->getCharacteristic("00001a00-0000-1000-8000-00805f9b34fb");
    if (wc && wc->canWrite()) {
        uint8_t cmd[] = {0xA0, 0x1F};
        wc->writeValue(cmd, 2, false);
        delay(200);
    }

    NimBLERemoteCharacteristic* rc =
        svc->getCharacteristic("00001a01-0000-1000-8000-00805f9b34fb");
    if (!rc || !rc->canRead()) {
        client->disconnect(); NimBLEDevice::deleteClient(client); return false;
    }
    std::string val = rc->readValue();
    if (val.length() < 8) {
        client->disconnect(); NimBLEDevice::deleteClient(client); return false;
    }
    const uint8_t* d = (const uint8_t*)val.data();

    uint8_t battery = 0;
    NimBLERemoteCharacteristic* bc =
        svc->getCharacteristic("00001a02-0000-1000-8000-00805f9b34fb");
    if (bc && bc->canRead()) {
        std::string bv = bc->readValue();
        if (bv.length() >= 1) battery = (uint8_t)bv[0];
    }

    SENSOR_LOCK();
    r.temperature   = (int16_t)(d[0] | (d[1] << 8)) / 10.0f;
    r.moisture      = d[3];
    r.light         = (uint32_t)(d[4] | (d[5] << 8));
    r.conductivity  = (uint16_t)(d[6] | (d[7] << 8));
    r.battery       = battery;
    r.valid         = true;
    r.timestamp     = time(nullptr);
    SENSOR_UNLOCK();

    client->disconnect();
    NimBLEDevice::deleteClient(client);
    return true;
}

void doBleScan() {
    scanResultCount = 0;
    bleScanning = true;

    NimBLEScan* scan = NimBLEDevice::getScan();
    scan->setActiveScan(true);
    scan->setInterval(100);
    scan->setWindow(99);

    NimBLEScanResults results = scan->start(10, false);

    SENSOR_LOCK();
    int count = 0;
    for (int i = 0; i < results.getCount() && count < MAX_SCAN_RESULTS; i++) {
        NimBLEAdvertisedDevice dev = results.getDevice(i);
        std::string name = dev.getName();
        std::string addr = dev.getAddress().toString();
        strlcpy(scanResults[count].addr, addr.c_str(), 18);
        strlcpy(scanResults[count].name, name.length() ? name.c_str() : "?", 48);
        scanResults[count].rssi = dev.getRSSI();
        count++;
    }
    scanResultCount = count;
    SENSOR_UNLOCK();
    scan->clearResults();
    bleScanning = false;
}

void doReadAllSensors() {
    for (int i = 0; i < NUM_CH; i++) {
        if (!sensorCfg[i].assigned) continue;
        Serial.printf("Reading sensor ch%d (%s)...\n", i+1, sensorCfg[i].mac);
        if (!readMiFlora(sensorCfg[i].mac, sensorData[i])) {
            Serial.printf("  Failed\n");
        } else {
            Serial.printf("  moisture=%d%% temp=%.1fC bat=%d%%\n",
                          sensorData[i].moisture, sensorData[i].temperature,
                          sensorData[i].battery);
        }
    }
}

bool anySensorAssigned() {
    for (int i = 0; i < NUM_CH; i++) if (sensorCfg[i].assigned) return true;
    return false;
}

bool isValidMac(const char* m) {
    if (!m || strlen(m) != 17) return false;
    for (int i = 0; i < 17; i++) {
        char c = m[i];
        if ((i % 3) == 2) { if (c != ':') return false; }
        else { if (!isxdigit((int)c)) return false; }
    }
    return true;
}


// =========================================================
// SCHEDULES + QUIET HOURS
// =========================================================
#define MAX_SCHEDULES 16

struct Schedule {
    bool    enabled;
    uint8_t channel;
    uint8_t days;       // bit0=Mon  bit6=Sun
    uint8_t hour;
    uint8_t minute;
    float   volume;
    float   speed;
    char    direction[4];
    uint8_t moistureThreshold;
};

Schedule schedules[MAX_SCHEDULES];

uint8_t quietStartHour = 0, quietEndHour = 0;

static void resetSchedule(Schedule& s) {
    s.enabled = false; s.channel = 1; s.days = 0;
    s.hour = 0; s.minute = 0; s.volume = 0; s.speed = 10;
    strlcpy(s.direction, "cw", sizeof(s.direction));
    s.moistureThreshold = 0;
}

void saveSchedule(int idx) {
    char key[12]; sprintf(key, "sched_%d", idx);
    StaticJsonDocument<256> doc;
    doc["en"]  = schedules[idx].enabled;
    doc["ch"]  = schedules[idx].channel;
    doc["dy"]  = schedules[idx].days;
    doc["hr"]  = schedules[idx].hour;
    doc["mn"]  = schedules[idx].minute;
    doc["vol"] = schedules[idx].volume;
    doc["spd"] = schedules[idx].speed;
    doc["dir"] = schedules[idx].direction;
    doc["mth"] = schedules[idx].moistureThreshold;
    char buf[256]; serializeJson(doc, buf);
    PREFS_LOCK();
    prefs.begin("peristaltica", false);
    prefs.putString(key, buf);
    prefs.end();
    PREFS_UNLOCK();
}

void loadSchedules() {
    PREFS_LOCK();
    prefs.begin("peristaltica", true);
    for (int i = 0; i < MAX_SCHEDULES; i++) {
        resetSchedule(schedules[i]);
        char key[12]; sprintf(key, "sched_%d", i);
        String s = prefs.getString(key, "");
        if (s.length() == 0) continue;
        StaticJsonDocument<256> doc;
        if (deserializeJson(doc, s)) continue;
        schedules[i].enabled           = doc["en"]  | false;
        schedules[i].channel           = doc["ch"]  | 1;
        schedules[i].days              = doc["dy"]  | 0;
        schedules[i].hour              = doc["hr"]  | 0;
        schedules[i].minute            = doc["mn"]  | 0;
        schedules[i].volume            = doc["vol"] | 0.0f;
        schedules[i].speed             = doc["spd"] | 10.0f;
        schedules[i].moistureThreshold = doc["mth"] | 0;
        strlcpy(schedules[i].direction, doc["dir"] | "cw", 4);
    }
    quietStartHour = prefs.getUChar("quiet_s", 0);
    quietEndHour   = prefs.getUChar("quiet_e", 0);
    prefs.end();
    PREFS_UNLOCK();
}

void deleteSchedule(int idx) {
    resetSchedule(schedules[idx]);
    char key[12]; sprintf(key, "sched_%d", idx);
    PREFS_LOCK();
    prefs.begin("peristaltica", false);
    prefs.remove(key);
    prefs.end();
    PREFS_UNLOCK();
}

void saveQuietHours(uint8_t s, uint8_t e) {
    quietStartHour = s; quietEndHour = e;
    PREFS_LOCK();
    prefs.begin("peristaltica", false);
    prefs.putUChar("quiet_s", s);
    prefs.putUChar("quiet_e", e);
    prefs.end();
    PREFS_UNLOCK();
}

static bool inQuietHours(uint8_t hour) {
    if (quietStartHour == quietEndHour) return false;
    if (quietStartHour < quietEndHour) {
        return hour >= quietStartHour && hour < quietEndHour;
    }
    return hour >= quietStartHour || hour < quietEndHour;
}


// =========================================================
// NTP
// =========================================================
String ntpServer1 = "pool.ntp.org";
String ntpServer2 = "time.cloudflare.com";

void applyTimezone() {
    PREFS_LOCK();
    prefs.begin("peristaltica", true);
    String tz = prefs.getString("timezone", "CET-1CEST,M3.5.0,M10.5.0/3");
    ntpServer1 = prefs.getString("ntp1", "pool.ntp.org");
    ntpServer2 = prefs.getString("ntp2", "time.cloudflare.com");
    prefs.end();
    PREFS_UNLOCK();
    setenv("TZ", tz.c_str(), 1);
    tzset();
}

static bool ntpSynced() {
    struct tm t;
    return getLocalTime(&t, 0) && (t.tm_year + 1900) >= 2024;
}

void setupNTP() {
    applyTimezone();
    configTime(0, 0, ntpServer1.c_str(), ntpServer2.c_str());
}


// =========================================================
// DECLARATIONS NEEDED BELOW
// =========================================================
void doRunVolume(int ch1based, float vol, float spd, const char* dir, const char* trig = "manual");
void doRunDuration(int ch1based, uint32_t durSec, float spd, const char* dir, const char* trig = "manual");
void doStop(int ch1based);
void publishHaDiscovery();
void publishChannelState(int ch1based);
void publishSensorState(int ch1based);


void checkSchedules() {
    struct tm t;
    if (!getLocalTime(&t, 0)) return;

    static int  lastCheckedMinute = -1;
    static bool firstTickConsumed = false;

    if (t.tm_min == lastCheckedMinute) return;
    lastCheckedMinute = t.tm_min;

    if (!firstTickConsumed) { firstTickConsumed = true; return; }

    // Auto-deactivate vacation mode when the resume date has passed
    if (vacationMode && vacationUntil > 0 && time(nullptr) >= vacationUntil) {
        vacationMode = false; vacationUntil = 0;
        saveVacationMode();
        Serial.println("Vacation mode auto-deactivated");
    }
    if (vacationMode) return;

    if (inQuietHours(t.tm_hour)) return;

    int dow    = (t.tm_wday == 0) ? 6 : t.tm_wday - 1;
    uint8_t db = 1 << dow;

    for (int i = 0; i < MAX_SCHEDULES; i++) {
        if (!schedules[i].enabled)              continue;
        if (!(schedules[i].days & db))           continue;
        if (schedules[i].hour   != t.tm_hour)    continue;
        if (schedules[i].minute != t.tm_min)     continue;

        if (schedules[i].moistureThreshold > 0) {
            int ch = schedules[i].channel - 1;
            SENSOR_LOCK();
            bool assigned = sensorCfg[ch].assigned;
            bool valid    = sensorData[ch].valid;
            uint8_t m     = sensorData[ch].moisture;
            time_t  age   = time(nullptr) - sensorData[ch].timestamp;
            SENSOR_UNLOCK();

            if (assigned && (!valid || age > SENSOR_OFFLINE_HOURS * 3600)) {
                Serial.printf("Schedule %d: sensor offline (age=%ld s), running anyway\n",
                              i, (long)age);
            } else if (assigned && valid && m >= schedules[i].moistureThreshold) {
                Serial.printf("Schedule %d skipped: moisture %d%% >= %d%%\n",
                              i, m, schedules[i].moistureThreshold);
                continue;
            }
        }

        doRunVolume(schedules[i].channel, schedules[i].volume,
                    schedules[i].speed,   schedules[i].direction, "sched");
    }
}

void rolloverDailyTotals() {
    struct tm t;
    if (!getLocalTime(&t, 0)) return;
    if (LastDayOfYear == -1) { LastDayOfYear = t.tm_yday; return; }
    if (t.tm_yday != LastDayOfYear) {
        for (int i = 0; i < NUM_CH; i++) DailyVolume[i] = 0;
        LastDayOfYear = t.tm_yday;
    }
}


// =========================================================
// HTML UI (PROGMEM)
// =========================================================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Perist&aacute;ltica</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:system-ui,sans-serif;background:#f0f2f5;color:#333}
header{background:#1565c0;color:#fff;padding:14px 20px;display:flex;justify-content:space-between;align-items:center;flex-wrap:wrap;gap:8px}
header h1{font-size:1.1rem;font-weight:600}
.sb{font-size:.78rem;display:flex;gap:14px;align-items:center}
.dot{width:9px;height:9px;border-radius:50%;display:inline-block;margin-right:5px;vertical-align:middle}
.on{background:#69f0ae}.off{background:#ef5350}
main{max-width:980px;margin:20px auto;padding:0 14px;display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:14px}
.card{background:#fff;border-radius:10px;padding:18px;box-shadow:0 1px 5px rgba(0,0,0,.1)}
.card h2{font-size:.95rem;margin-bottom:14px;color:#1565c0;font-weight:600;border-bottom:1px solid #e3e8f0;padding-bottom:8px}
label{display:block;font-size:.76rem;color:#666;margin:9px 0 3px}
input,select{width:100%;padding:7px 9px;border:1px solid #ddd;border-radius:6px;font-size:.88rem;background:#fafafa}
input:focus,select:focus{outline:none;border-color:#1565c0;background:#fff}
.row{display:flex;gap:8px;margin-top:14px}
button{flex:1;padding:9px 6px;border:none;border-radius:6px;font-size:.88rem;cursor:pointer;font-weight:500;transition:opacity .15s,transform .1s}
button:active{opacity:.75;transform:scale(.97)}
.run{background:#1565c0;color:#fff}.stp{background:#e53935;color:#fff}
.sav{background:#2e7d32;color:#fff}.rst{background:#bf360c;color:#fff}
.sec{background:#546e7a;color:#fff}
.pw{margin-top:12px}
.pl{font-size:.73rem;color:#888;display:flex;justify-content:space-between;margin-bottom:4px}
.pb{height:8px;background:#e0e0e0;border-radius:4px;overflow:hidden}
.pf{height:100%;background:#1565c0;border-radius:4px;transition:width .6s ease}
.full{grid-column:1/-1}.two{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.msg{font-size:.76rem;margin-top:8px;color:#2e7d32;min-height:1.1em}
.daily{font-size:.72rem;color:#888;margin-top:6px}
.tabs{display:flex;gap:4px;margin-bottom:8px}
.tab{flex:1;padding:6px;background:#e8eaf6;color:#3949ab;border-radius:5px;cursor:pointer;text-align:center;font-size:.8rem}
.tab.act{background:#1565c0;color:#fff}
.schd-form{display:grid;grid-template-columns:repeat(auto-fit,minmax(130px,1fr));gap:8px;margin-top:10px}
.days{display:flex;gap:5px;margin-top:4px;flex-wrap:wrap}
.db{width:30px;height:30px;border-radius:50%;padding:0;flex:none;background:#e8eaf6;color:#3949ab;font-size:.78rem;font-weight:700;border:2px solid transparent}
.db.sel{background:#1565c0;color:#fff;border-color:#0d47a1}
table{width:100%;border-collapse:collapse;font-size:.81rem;margin-top:8px}
th{color:#888;font-weight:500;text-align:left;padding:5px 4px;border-bottom:1px solid #e8eaf6}
td{padding:5px 4px;border-bottom:1px solid #f5f5f5;vertical-align:middle}
.tz-row{display:flex;gap:8px;align-items:flex-end;margin-bottom:12px;flex-wrap:wrap}
.tz-row input{flex:1;min-width:180px}
.tz-row button{flex:none;padding:7px 14px}
.time-badge{font-size:.95rem;font-weight:600;color:#fff;font-variant-numeric:tabular-nums}
.slots{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:10px;margin-bottom:14px}
.slot{border:2px dashed #dce3f0;border-radius:8px;padding:12px;font-size:.82rem}
.slot.asgn{border:2px solid #1565c0;border-style:solid}
.slot-hd{font-weight:600;color:#1565c0;margin-bottom:6px;font-size:.88rem}
.reading{display:grid;grid-template-columns:1fr 1fr;gap:4px;margin:8px 0;font-size:.8rem}
.reading span{color:#666}
.reading b{color:#333}
.bat{display:inline-block;font-size:.7rem;color:#888;margin-left:6px}
.bat.lo{color:#e53935;font-weight:600}
.scan-item{display:flex;justify-content:space-between;align-items:center;padding:8px 4px;border-bottom:1px solid #f0f0f0;font-size:.82rem}
.scan-item .flora{color:#2e7d32;font-size:.72rem;font-weight:600;margin-left:4px}
.afrm{background:#f8f9ff;border:1px solid #c5cae9;border-radius:8px;padding:12px;margin-top:8px;display:none}
.afrm-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.sys{font-size:.78rem;color:#666;display:grid;grid-template-columns:1fr 1fr;gap:4px}
.sys b{color:#333}
.cal-step{font-size:.85rem;color:#444;line-height:1.4}
</style>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js"></script>
</head>
<body>
<header>
  <h1>Bomba Perist&aacute;ltica (MKS DLC32 v2.1)</h1>
  <div class="sb">
    <span><span class="dot off" id="dW"></span><span id="lW">WiFi</span></span>
    <span><span class="dot off" id="dM"></span>MQTT</span>
    <span class="time-badge" id="clock">--:--:--</span>
    <span id="vacBadge" style="display:none;background:#ff8f00;border-radius:10px;padding:2px 8px;font-size:.75rem;font-weight:600">&#127958; Vacaciones</span>
  </div>
</header>
<main>

  <div id="channels" class="full" style="display:contents"></div>

  <div class="card full">
    <h2>&#128202; Historial de riegos</h2>
    <div style="position:relative;max-height:220px"><canvas id="histChart"></canvas></div>
    <div style="display:flex;justify-content:space-between;align-items:center;margin:14px 0 6px">
      <span style="font-size:.86rem;font-weight:600;color:#1565c0">&#128336; &Uacute;ltimos riegos</span>
      <button class="stp" style="padding:4px 14px;font-size:.78rem;flex:none" onclick="if(confirm('&#191;Borrar todo el historial?'))clearHist()">&#128465; Borrar historial</button>
    </div>
    <table>
      <thead><tr><th>Fecha / Hora</th><th>Canal</th><th>Volumen</th><th>Origen</th></tr></thead>
      <tbody id="histBody"><tr><td colspan="4" style="color:#aaa;text-align:center;padding:12px">Cargando...</td></tr></tbody>
    </table>
  </div>

  <div class="card full" style="display:flex;align-items:center;gap:20px;flex-wrap:wrap">
    <h2 style="border:none;padding:0;margin:0;flex:none">Control global</h2>
    <button class="stp" style="max-width:200px;flex:unset;padding:10px 24px" onclick="stp(0)">&#9632; Parar todo</button>
  </div>

  <div class="card full">
    <h2>&#127807; Sensores Mi Flora</h2>
    <div class="slots" id="sensorSlots"></div>
    <div class="row" style="max-width:280px;margin-bottom:10px">
      <button class="sec" id="scanBtn" onclick="startScan()">&#128268; Buscar dispositivos BLE</button>
    </div>
    <div id="scanStatus" style="font-size:.8rem;color:#666;margin-bottom:8px"></div>
    <div id="scanList"></div>
    <div class="afrm" id="assignForm">
      <b style="font-size:.88rem">Asignar sensor</b>
      <div class="afrm-grid" style="margin-top:10px">
        <div><label>Canal</label>
          <select id="aCh"><option value="0">Canal 1</option><option value="1">Canal 2</option><option value="2">Canal 3</option></select>
        </div>
        <div><label>Umbral humedad (%)</label>
          <input type="number" id="aTh" value="40" min="0" max="100">
        </div>
      </div>
      <small style="color:#888;display:block;margin-top:4px">0 = ignorar sensor en schedules</small>
      <div class="row" style="margin-top:10px">
        <button class="sav" onclick="doAssign()">Confirmar</button>
        <button class="sec" onclick="hideAssign()">Cancelar</button>
      </div>
    </div>
    <div class="row" style="max-width:220px;margin-top:12px">
      <button class="sec" onclick="readNow()">&#8635; Leer sensores ahora</button>
    </div>
    <div class="msg" id="sensorMsg"></div>
  </div>

  <div class="card full">
    <h2>&#128197; Programaci&oacute;n semanal</h2>
    <div class="tz-row">
      <div style="flex:1"><label>Zona horaria (POSIX)</label>
        <input id="tzIn" placeholder="CET-1CEST,M3.5.0,M10.5.0/3">
      </div>
      <button class="sav" onclick="saveTz()">Guardar TZ</button>
    </div>
    <div class="tz-row">
      <div style="flex:1"><label>NTP servidor 1</label><input id="ntp1"></div>
      <div style="flex:1"><label>NTP servidor 2</label><input id="ntp2"></div>
      <button class="sav" onclick="saveNtp()">Guardar NTP</button>
    </div>
    <div class="tz-row">
      <div><label>Hora silencio inicio</label><input type="number" id="qS" min="0" max="23" value="0" style="width:80px"></div>
      <div><label>Fin</label><input type="number" id="qE" min="0" max="23" value="0" style="width:80px"></div>
      <button class="sav" onclick="saveQuiet()">Guardar silencio</button>
    </div>
    <small style="color:#888">Inicio==fin desactiva. Soporta cruzar medianoche (ej. 22-7).</small>
    <table>
      <thead><tr>
        <th>Canal</th><th>D&iacute;as</th><th>Hora</th>
        <th>Volumen</th><th>Vel.</th><th>Dir.</th>
        <th>Umbral</th><th>Pr&oacute;xima</th><th>Activo</th><th></th>
      </tr></thead>
      <tbody id="schedBody"></tbody>
    </table>
    <h3 style="font-size:.88rem;color:#1565c0;margin:16px 0 6px">Nuevo horario</h3>
    <div class="schd-form">
      <div><label>Canal</label>
        <select id="nch"><option value="1">Canal 1</option><option value="2">Canal 2</option><option value="3">Canal 3</option></select>
      </div>
      <div><label>Hora</label><input type="time" id="nhm" value="08:00"></div>
      <div><label>Volumen (mL)</label><input type="number" id="nvol" value="100" min="1"></div>
      <div><label>Velocidad (mL/min)</label><input type="number" id="nspd" value="10" min="0.1" step="0.1"></div>
      <div><label>Direcci&oacute;n</label>
        <select id="ndir"><option value="cw">CW &#8635;</option><option value="ccw">CCW &#8634;</option></select>
      </div>
      <div><label>Umbral humedad (%)</label>
        <input type="number" id="nmth" value="0" min="0" max="100">
        <small style="color:#aaa">0=sin sensor</small>
      </div>
    </div>
    <div style="margin:10px 0">
      <label>D&iacute;as de la semana</label>
      <div class="days" id="dayPicker">
        <button class="db" data-d="0" onclick="toggleDay(0)">L</button>
        <button class="db" data-d="1" onclick="toggleDay(1)">M</button>
        <button class="db" data-d="2" onclick="toggleDay(2)">X</button>
        <button class="db" data-d="3" onclick="toggleDay(3)">J</button>
        <button class="db" data-d="4" onclick="toggleDay(4)">V</button>
        <button class="db" data-d="5" onclick="toggleDay(5)">S</button>
        <button class="db" data-d="6" onclick="toggleDay(6)">D</button>
      </div>
    </div>
    <div class="row" style="max-width:200px">
      <button class="sav" onclick="addSched()">+ A&ntilde;adir horario</button>
    </div>
    <div class="msg" id="schedMsg"></div>
  </div>

  <div class="card">
    <h2>Calibraci&oacute;n &amp; Tope diario</h2>
    <label>Canal</label>
    <select id="cc"><option value="1">Canal 1</option><option value="2">Canal 2</option><option value="3">Canal 3</option></select>
    <div class="tabs">
      <div class="tab act" onclick="calTab('m')" id="tbM">Manual</div>
      <div class="tab" onclick="calTab('w')" id="tbW">Asistente</div>
      <div class="tab" onclick="calTab('p')" id="tbP">B&aacute;scula</div>
    </div>

    <div id="calM">
      <label>Pasos / mL</label><input type="number" id="cs" value="1600" min="1">
      <div class="row"><button class="sav" onclick="calSave()">Guardar pasos/mL</button></div>
    </div>

    <div id="calW" style="display:none">
      <div class="cal-step" id="calWStep">1) Coloca un recipiente graduado en la salida.</div>
      <label>Segundos a bombear</label><input type="number" id="cwSec" value="30" min="5" max="120">
      <div class="row">
        <button class="run" onclick="calStart()">&#9654; Bombear</button>
      </div>
      <label>mL recogidos</label><input type="number" id="cwMl" value="" step="0.1" min="0.1">
      <div class="row">
        <button class="sav" onclick="calCalc()">Calcular y guardar</button>
      </div>
    </div>

    <div id="calP" style="display:none">
      <div class="cal-step">1) Pon un vaso en la b&aacute;scula y tara a 0.<br>2) Pulsa Bombear, espera que acabe.<br>3) Pesa el l&iacute;quido y escribe los gramos.</div>
      <label>Segundos a bombear</label><input type="number" id="cpSec" value="30" min="5" max="120">
      <div class="row"><button class="run" onclick="calStartPeso()">&#9654; Bombear</button></div>
      <label>Gramos medidos en la b&aacute;scula</label>
      <input type="number" id="cpGr" value="" step="0.1" min="0.1" placeholder="ej. 47.3">
      <label>Densidad del l&iacute;quido (g/mL)</label>
      <input type="number" id="cpDens" value="1.00" step="0.01" min="0.01">
      <small style="color:#aaa;display:block;margin-bottom:4px">Agua = 1.00 &bull; Soluciones nutritivas &asymp; 1.01-1.05</small>
      <div class="row"><button class="sav" onclick="calCalcPeso()">Calcular y guardar</button></div>
    </div>

    <hr style="border:none;border-top:1px solid #eee;margin:14px 0">
    <label>Tope diario (mL/d&iacute;a, 0=sin l&iacute;mite)</label>
    <input type="number" id="capV" value="0" min="0">
    <div class="row"><button class="sav" onclick="capSave()">Guardar tope</button></div>
    <div class="msg" id="cm"></div>
  </div>

  <div class="card">
    <h2>Configuraci&oacute;n MQTT</h2>
    <label>Servidor</label><input type="text" id="ms" placeholder="192.168.1.100">
    <div class="two">
      <div><label>Puerto</label><input type="number" id="mp" value="1883"></div>
      <div><label>Usuario</label><input type="text" id="mu" autocomplete="off"></div>
    </div>
    <label>Contrase&ntilde;a</label><input type="password" id="mw" autocomplete="new-password">
    <div class="row">
      <button class="sav" onclick="saveCfg()">Guardar</button>
      <button class="rst" onclick="rstWifi()">Reset WiFi</button>
    </div>
    <div class="msg" id="mm"></div>
  </div>

  <div class="card full">
    <h2>&#9881; Sistema</h2>
    <div class="sys" id="sysInfo">Cargando...</div>
    <hr style="border:none;border-top:1px solid #eee;margin:14px 0">
    <label style="display:flex;align-items:center;gap:8px;cursor:pointer;width:auto;font-size:.9rem;font-weight:500;margin-bottom:4px">
      <input type="checkbox" id="vacMode" onchange="setVacation(this.checked)" style="width:auto;cursor:pointer">
      &#127958; Modo vacaciones
    </label>
    <small style="color:#888;font-size:.75rem">Suspende todos los horarios autom&aacute;ticos mientras est&aacute;s fuera</small>
    <div style="margin-top:8px;display:flex;align-items:center;gap:8px;flex-wrap:wrap">
      <label style="width:auto;font-size:.76rem;color:#666;margin:0">Reanudar el:</label>
      <input type="date" id="vacUntil" style="width:160px">
      <small style="color:#aaa;font-size:.74rem">vac&iacute;o = indefinido</small>
    </div>
    <div class="row" style="max-width:400px;margin-top:12px;gap:10px">
      <button class="sec" onclick="api('/api/system/reboot',{}).then(()=>alert('Reiniciando...'))">Reiniciar</button>
      <button class="sav" onclick="location.href='/update'">&#11014; Actualizar firmware</button>
    </div>
  </div>

</main>
<script>
const $ = id => document.getElementById(id);
const DAY_NAMES = ['L','M','X','J','V','S','D'];
let selDays = 0;
let assigningMAC = '', assigningName = '';
let sensors = [{},{},{}];
let calMode = 'm';
let histChart = null;

// ---- History ----
function loadHistory() {
  api('/api/history').then(d => {
    if (!d) return;
    renderHistTable(d.events);
    renderHistChart(computeChartData(d.events));
  });
}
function computeChartData(events) {
  const days=[], c1=[], c2=[], c3=[];
  const DOW=['D','L','M','X','J','V','S'];
  const now=new Date(); now.setHours(23,59,59,999);
  for (let d=6; d>=0; d--) {
    const day=new Date(now); day.setDate(day.getDate()-d);
    days.push(DOW[day.getDay()]+' '+day.getDate()+'/'+(day.getMonth()+1));
    const ds=new Date(day); ds.setHours(0,0,0,0);
    const t0=ds.getTime()/1000, t1=t0+86400;
    let v1=0, v2=0, v3=0;
    (events||[]).forEach(e=>{
      if(e.ts>=t0 && e.ts<t1){
        if(e.ch===1) v1+=e.v; else if(e.ch===2) v2+=e.v; else v3+=e.v;
      }
    });
    c1.push(+v1.toFixed(1)); c2.push(+v2.toFixed(1)); c3.push(+v3.toFixed(1));
  }
  return {days,c1,c2,c3};
}
function renderHistTable(events) {
  const tb=$('histBody'); if(!tb) return;
  if(!events||!events.length){
    tb.innerHTML='<tr><td colspan="4" style="color:#aaa;text-align:center;padding:12px">Sin eventos registrados</td></tr>'; return;
  }
  const recent=[...events].reverse().slice(0,30);
  const tL={manual:'Manual',sched:'Programado',mqtt:'MQTT'};
  tb.innerHTML=recent.map(e=>{
    const d=new Date(e.ts*1000);
    const dt=d.toLocaleDateString('es',{day:'2-digit',month:'2-digit'})+' '+
             d.toLocaleTimeString('es',{hour:'2-digit',minute:'2-digit'});
    return `<tr><td>${dt}</td><td>Canal ${e.ch}</td><td>${e.v.toFixed(1)} mL</td><td>${tL[e.t]||e.t}</td></tr>`;
  }).join('');
}
function renderHistChart(data) {
  const canvas=$('histChart'); if(!canvas) return;
  if(histChart){histChart.destroy(); histChart=null;}
  if(typeof Chart==='undefined') return;
  histChart=new Chart(canvas.getContext('2d'),{
    type:'bar',
    data:{labels:data.days, datasets:[
      {label:'Canal 1', data:data.c1, backgroundColor:'rgba(21,101,192,0.75)'},
      {label:'Canal 2', data:data.c2, backgroundColor:'rgba(46,125,50,0.75)'},
      {label:'Canal 3', data:data.c3, backgroundColor:'rgba(230,81,0,0.75)'}
    ]},
    options:{responsive:true,
      plugins:{legend:{position:'bottom'}},
      scales:{y:{beginAtZero:true, title:{display:true, text:'mL'}}}}
  });
}
function clearHist() {
  api('/api/history', null, 'DELETE').then(()=>loadHistory());
}

// ---- Vacation mode ----
function loadVacation() {
  api('/api/vacation').then(d=>{
    if(!d) return;
    const cb=$('vacMode'); if(cb) cb.checked=d.enabled;
    const vb=$('vacBadge'); if(vb) vb.style.display=d.enabled?'':'none';
    if(d.until>0 && $('vacUntil')){
      const dt=new Date(d.until*1000);
      $('vacUntil').value=dt.toISOString().split('T')[0];
    }
  });
}
function setVacation(en) {
  const dateStr=($('vacUntil')||{}).value||'';
  let until=0;
  if(dateStr){ const d=new Date(dateStr+'T23:59:59'); until=Math.floor(d.getTime()/1000); }
  api('/api/vacation',{enabled:en,until}).then(d=>{
    if(d?.ok){ const vb=$('vacBadge'); if(vb) vb.style.display=en?'':'none'; }
  });
}

async function api(path, body, method) {
  try {
    const m = method || (body ? 'POST' : 'GET');
    const opts = body ? {method:m, headers:{'Content-Type':'application/json'}, body:JSON.stringify(body)} : {method:m};
    return (await fetch(path, opts)).json();
  } catch(e) { return null; }
}

function renderChannels() {
  const html = [1,2,3].map(ch => `
    <div class="card">
      <h2>Canal ${ch}</h2>
      <div class="tabs">
        <div class="tab act" onclick="modeTab(${ch},'v')" id="tv${ch}">Volumen</div>
        <div class="tab" onclick="modeTab(${ch},'d')" id="td${ch}">Duraci&oacute;n</div>
      </div>
      <div id="mv${ch}">
        <label>Volumen (mL)</label><input type="number" id="v${ch}" value="100" min="0.1" step="0.1">
      </div>
      <div id="md${ch}" style="display:none">
        <label>Duraci&oacute;n (s)</label><input type="number" id="dur${ch}" value="60" min="1">
      </div>
      <label>Velocidad (mL/min)</label><input type="number" id="s${ch}" value="10" min="0.1" step="0.1">
      <label>Direcci&oacute;n</label>
      <select id="d${ch}"><option value="cw">CW &#8635;</option><option value="ccw">CCW &#8634;</option></select>
      <div class="row">
        <button class="run" onclick="run(${ch})">&#9654; Iniciar</button>
        <button class="stp" onclick="stp(${ch})">&#9632; Parar</button>
      </div>
      <div class="pw"><div class="pl"><span>Progreso</span><span id="p${ch}t">0%</span></div>
      <div class="pb"><div class="pf" id="p${ch}" style="width:0%"></div></div></div>
      <div class="daily" id="da${ch}">Hoy: 0 mL</div>
    </div>
  `).join('');
  $('channels').innerHTML = html;
}
window.runMode = window.runMode || {1:'v',2:'v',3:'v'};
function modeTab(ch, m) {
  runMode[ch] = m;
  $('tv'+ch).className = 'tab' + (m==='v'?' act':'');
  $('td'+ch).className = 'tab' + (m==='d'?' act':'');
  $('mv'+ch).style.display = m==='v'?'':'none';
  $('md'+ch).style.display = m==='d'?'':'none';
}

function run(ch) {
  if (runMode[ch] === 'd') {
    api('/api/run', {channel:ch, duration:+$('dur'+ch).value, speed:+$('s'+ch).value, direction:$('d'+ch).value});
  } else {
    api('/api/run', {channel:ch, volume:+$('v'+ch).value, speed:+$('s'+ch).value, direction:$('d'+ch).value});
  }
}
function stp(ch) { api('/api/stop', {channel:ch}); }

function calTab(m) {
  calMode = m;
  $('tbM').className = 'tab' + (m==='m'?' act':'');
  $('tbW').className = 'tab' + (m==='w'?' act':'');
  $('tbP').className = 'tab' + (m==='p'?' act':'');
  $('calM').style.display = m==='m'?'':'none';
  $('calW').style.display = m==='w'?'':'none';
  $('calP').style.display = m==='p'?'':'none';
}
function calSave() {
  api('/api/calibrate', {channel:+$('cc').value, stepsperml:+$('cs').value})
    .then(()=>{ $('cm').textContent='Guardado'; setTimeout(()=>$('cm').textContent='',2500); });
}
function calStart() {
  const ch = +$('cc').value;
  const sec = +$('cwSec').value;
  $('calWStep').textContent = `Bombeando ${sec}s a ${$('cs').value} pasos/mL...`;
  api('/api/calibrate/run', {channel:ch, duration:sec})
    .then(()=>{
      $('calWStep').textContent = `Hecho. Cuantos mL recogiste?`;
    });
}
function calCalc() {
  const ml = +$('cwMl').value, sec = +$('cwSec').value, ch = +$('cc').value;
  if (ml <= 0 || sec <= 0) { $('cm').textContent='Datos invalidos'; return; }
  api('/api/calibrate/compute', {channel:ch, ml, seconds:sec})
    .then(d => {
      if (d?.stepsperml) {
        $('cs').value = d.stepsperml;
        $('cm').textContent = `Calibrado: ${d.stepsperml} pasos/mL`;
        setTimeout(()=>$('cm').textContent='',4000);
      }
    });
}
function calStartPeso() {
  const ch=+$('cc').value, sec=+$('cpSec').value;
  api('/api/calibrate/run',{channel:ch,duration:sec}).then(()=>{
    $('cm').textContent=`Bombeando ${sec} s... coloca el vaso en la báscula y tara ahora.`;
  });
}
function calCalcPeso() {
  const gr=+$('cpGr').value, dens=+(($('cpDens').value)||1)||1;
  const ml=gr/dens, sec=+$('cpSec').value, ch=+$('cc').value;
  if(ml<=0||sec<=0){$('cm').textContent='Datos inválidos';return;}
  api('/api/calibrate/compute',{channel:ch,ml,seconds:sec})
    .then(d=>{
      if(d?.stepsperml){
        $('cs').value=d.stepsperml;
        $('cm').textContent=`Calibrado: ${d.stepsperml} pasos/mL`;
        setTimeout(()=>$('cm').textContent='',4000);
      }
    });
}
function capSave() {
  api('/api/cap', {channel:+$('cc').value, cap:+$('capV').value})
    .then(()=>{ $('cm').textContent='Tope guardado'; setTimeout(()=>$('cm').textContent='',2500); });
}

function saveCfg() {
  api('/api/config', {server:$('ms').value, port:+$('mp').value, user:$('mu').value, pass:$('mw').value})
    .then(d=>{ $('mm').textContent=d?'Guardado':'Error'; setTimeout(()=>$('mm').textContent='',3000); });
}
function rstWifi() { if(confirm('Borrar WiFi y reiniciar?')) api('/api/resetwifi', {}); }

function renderSensorSlots() {
  const html = [0,1,2].map(i => {
    const cfg = sensors[i]?.cfg;
    const rd  = sensors[i]?.reading;
    if (!cfg?.assigned) return `
      <div class="slot">
        <div class="slot-hd">Canal ${i+1}</div>
        <div style="color:#aaa;font-size:.8rem">Sin sensor asignado</div>
      </div>`;
    const ago = rd?.valid && rd.timestamp ? relTime(rd.timestamp) : 'Sin lectura';
    const bat = rd?.battery ? `<span class="bat ${rd.battery<20?'lo':''}">&#128267; ${rd.battery}%</span>` : '';
    return `
      <div class="slot asgn">
        <div class="slot-hd">Canal ${i+1} - ${cfg.name||cfg.mac} ${bat}</div>
        <div style="color:#888;font-size:.75rem">${cfg.mac}</div>
        ${rd?.valid ? `
        <div class="reading">
          <span>Humedad</span><b>${rd.moisture}%</b>
          <span>Temperatura</span><b>${rd.temperature?.toFixed(1)}C</b>
          <span>Luz</span><b>${rd.light} lux</b>
          <span>Fertilidad</span><b>${rd.conductivity} uS/cm</b>
        </div>` : '<div style="color:#aaa;margin:6px 0;font-size:.78rem">Sin lectura</div>'}
        <div style="font-size:.73rem;color:#999;margin-bottom:6px">${ago} - Umbral: ${cfg.threshold>0?cfg.threshold+'%':'sin umbral'}</div>
        <button class="stp" style="padding:4px 10px;font-size:.78rem;flex:none" onclick="removeSensor(${i})">Quitar</button>
      </div>`;
  }).join('');
  $('sensorSlots').innerHTML = html;
}
function relTime(ts) {
  const diff = Math.floor(Date.now()/1000) - ts;
  if (diff < 60)   return 'Hace <1 min';
  if (diff < 3600) return `Hace ${Math.floor(diff/60)} min`;
  return `Hace ${Math.floor(diff/3600)} h`;
}
function loadSensors() {
  api('/api/sensors').then(d => { if (!d) return; sensors = d; renderSensorSlots(); });
}
function removeSensor(ch) { if (confirm(`Quitar sensor del Canal ${ch+1}?`)) api('/api/sensors?channel='+ch, null, 'DELETE').then(loadSensors); }
function readNow() {
  $('sensorMsg').textContent = 'Leyendo sensores...';
  api('/api/sensors/read', {}).then(() => setTimeout(() => { loadSensors(); $('sensorMsg').textContent=''; }, 20000));
}
function startScan() {
  $('scanBtn').disabled = true;
  $('scanStatus').textContent = 'Escaneando... (10 s)';
  $('scanList').innerHTML = '';
  hideAssign();
  api('/api/ble/scan', {}).then(() => setTimeout(() => {
    api('/api/ble/results').then(renderScanResults);
    $('scanBtn').disabled = false;
    $('scanStatus').textContent = '';
  }, 12000));
}
function renderScanResults(data) {
  if (!data || !data.length) { $('scanList').innerHTML='<div style="color:#999;font-size:.82rem">No se encontraron dispositivos.</div>'; return; }
  const items = data.map(d => {
    const isFlora = /flower|flora|hhcc/i.test(d.name);
    return `<div class="scan-item">
      <div><b>${d.name}</b>${isFlora?'<span class="flora">&#127807; Mi Flora</span>':''}
        <div style="color:#999;font-size:.73rem">${d.addr} &nbsp;-&nbsp; ${d.rssi} dBm</div>
      </div>
      <button class="sec" style="flex:none;padding:5px 12px;font-size:.8rem" onclick="showAssign('${d.addr}','${d.name.replace(/'/g,"\\'")}')">Asignar</button>
    </div>`;
  }).join('');
  $('scanList').innerHTML = items;
}
function showAssign(mac, name) {
  assigningMAC = mac; assigningName = name;
  $('assignForm').style.display = 'block';
  $('assignForm').scrollIntoView({behavior:'smooth', block:'nearest'});
}
function hideAssign() { $('assignForm').style.display='none'; }
function doAssign() {
  api('/api/sensors', {channel:+$('aCh').value, mac:assigningMAC, name:assigningName, threshold:+$('aTh').value})
    .then(d => {
      if (!d?.ok) { $('sensorMsg').textContent='Error'; return; }
      hideAssign(); $('scanList').innerHTML=''; loadSensors();
      $('sensorMsg').textContent='Sensor asignado';
      setTimeout(()=>$('sensorMsg').textContent='',2500);
    });
}

function toggleDay(d) {
  selDays ^= (1<<d);
  document.querySelectorAll('.db').forEach(b=>b.classList.toggle('sel',!!(selDays&(1<<+b.dataset.d))));
}
function daysStr(mask) { return DAY_NAMES.filter((_,i)=>mask&(1<<i)).join(' ')||'-'; }
function nextRun(s) {
  if (!s.enabled || !s.days) return '-';
  const now = new Date();
  for (let i = 0; i < 8; i++) {
    const cand = new Date(now); cand.setDate(now.getDate()+i);
    cand.setHours(s.hour, s.minute, 0, 0);
    if (cand <= now) continue;
    const dow = (cand.getDay()+6) % 7;
    if (!(s.days & (1<<dow))) continue;
    const days = ['L','M','X','J','V','S','D'];
    return `${days[dow]} ${String(s.hour).padStart(2,'0')}:${String(s.minute).padStart(2,'0')}`;
  }
  return '-';
}
function loadSchedules() {
  api('/api/schedules').then(data => {
    if (!data) return;
    const tb = $('schedBody'); tb.innerHTML = '';
    data.forEach(s => {
      const tr = document.createElement('tr');
      tr.innerHTML =
        '<td>Ch.'+s.channel+'</td>'+
        '<td style="white-space:nowrap">'+daysStr(s.days)+'</td>'+
        '<td>'+String(s.hour).padStart(2,'0')+':'+String(s.minute).padStart(2,'0')+'</td>'+
        '<td>'+s.volume+' mL</td>'+
        '<td>'+s.speed+'</td>'+
        '<td>'+s.direction+'</td>'+
        '<td>'+(s.moistureThreshold>0?s.moistureThreshold+'%':'-')+'</td>'+
        '<td style="white-space:nowrap;color:#1565c0">'+nextRun(s)+'</td>'+
        '<td><input type="checkbox"'+(s.enabled?' checked':'')+' onchange="toggleSched('+s.id+',this.checked)"></td>'+
        '<td><button class="stp" style="padding:3px 10px;font-size:.78rem;flex:none" onclick="delSched('+s.id+')">&#10005;</button></td>';
      tb.appendChild(tr);
    });
  });
}
function addSched() {
  if (!selDays) { $('schedMsg').textContent='Selecciona al menos un dia'; return; }
  const parts = $('nhm').value.split(':');
  api('/api/schedules', {
    channel:+$('nch').value, hour:+parts[0], minute:+parts[1],
    volume:+$('nvol').value, speed:+$('nspd').value,
    direction:$('ndir').value, days:selDays, enabled:true,
    moistureThreshold:+$('nmth').value
  }).then(d=>{
    if(!d?.ok){$('schedMsg').textContent='Error';return;}
    loadSchedules();
    $('schedMsg').textContent='Horario anadido';
    setTimeout(()=>$('schedMsg').textContent='',2500);
  });
}
function delSched(id) { if(confirm('Eliminar?')) api('/api/schedules?id='+id,null,'DELETE').then(loadSchedules); }
function toggleSched(id,en) { api('/api/schedules',{id,enabled:en}); }
function saveTz() {
  api('/api/timezone',{tz:$('tzIn').value})
    .then(()=>{$('schedMsg').textContent='Zona horaria guardada';setTimeout(()=>$('schedMsg').textContent='',2500);});
}
function saveNtp() {
  api('/api/ntp',{ntp1:$('ntp1').value, ntp2:$('ntp2').value})
    .then(()=>{$('schedMsg').textContent='NTP guardado (reinicio recomendado)';setTimeout(()=>$('schedMsg').textContent='',3000);});
}
function saveQuiet() {
  api('/api/quiet',{start:+$('qS').value, end:+$('qE').value})
    .then(()=>{$('schedMsg').textContent='Horas silencio guardadas';setTimeout(()=>$('schedMsg').textContent='',2500);});
}

function renderSys(d) {
  $('sysInfo').innerHTML =
    `<span>Versi&oacute;n</span><b>${d.version||'?'}</b>` +
    `<span>Uptime</span><b>${formatDur(d.uptime||0)}</b>` +
    `<span>Heap libre</span><b>${(d.heap/1024|0)} KB</b>` +
    `<span>WiFi RSSI</span><b>${d.rssi||0} dBm</b>` +
    `<span>IP</span><b>${d.ip||'?'}</b>` +
    `<span>MAC</span><b>${d.mac||'?'}</b>`;
}
function formatDur(s) {
  const d=Math.floor(s/86400), h=Math.floor((s%86400)/3600), m=Math.floor((s%3600)/60);
  if (d) return `${d}d ${h}h`;
  if (h) return `${h}h ${m}m`;
  return `${m}m`;
}
function poll() {
  api('/api/status').then(d=>{
    if(!d)return;
    $('dW').className='dot on'; $('lW').textContent=d.ip||'WiFi';
    $('dM').className='dot '+(d.mqtt?'on':'off');
    const vb=$('vacBadge'); if(vb) vb.style.display=d.vacation?'':'none';
    const vcb=$('vacMode'); if(vcb&&vcb!==document.activeElement) vcb.checked=!!d.vacation;
    for(let i=0;i<3;i++){
      const m=d.motors[i]||{};
      $('p'+(i+1)).style.width=(m.progress||0)+'%';
      $('p'+(i+1)+'t').textContent=(m.progress||0)+'%';
      const cap = d.caps && d.caps[i]>0 ? ` / ${d.caps[i]} mL` : '';
      $('da'+(i+1)).textContent = `Hoy: ${(d.daily?.[i]||0).toFixed(1)} mL${cap}`;
    }
    if(d.time)$('clock').textContent=d.time;
  });
}
function pollSys() { api('/api/system').then(d=>{ if(d) renderSys(d); }); }

renderChannels();
api('/api/config').then(d=>{if(!d)return;$('ms').value=d.server||'';$('mp').value=d.port||1883;$('mu').value=d.user||'';});
api('/api/timezone').then(d=>{if(d)$('tzIn').value=d.tz||'';});
api('/api/ntp').then(d=>{if(d){$('ntp1').value=d.ntp1||'';$('ntp2').value=d.ntp2||'';}});
api('/api/quiet').then(d=>{if(d){$('qS').value=d.start||0;$('qE').value=d.end||0;}});
loadHistory();
loadVacation();
$('cc').addEventListener('change', () => {
  const ch = +$('cc').value;
  api('/api/cap?channel='+ch).then(d=>{if(d)$('capV').value=d.cap||0;});
  api('/api/calibrate?channel='+ch).then(d=>{if(d)$('cs').value=d.stepsperml||1600;});
});
$('cc').dispatchEvent(new Event('change'));
loadSchedules();
loadSensors();
setInterval(poll,1000);
setInterval(pollSys,15000);
setInterval(loadSensors,30000);
setInterval(loadHistory,60000);
poll();
pollSys();
</script>
</body>
</html>
)rawliteral";


// =========================================================
// MOTOR ACTIONS — array-based, driven via Stepper595
// =========================================================
static void applyRunCommon(int i, long stepsAbs, float speedStepsPerSec) {
    long absSteps = (stepsAbs < 0) ? -stepsAbs : stepsAbs;
    bool cw       = (stepsAbs >= 0);

    StepsToMove[i]    = absSteps;
    SpeedToMove[i]    = (long)round(speedStepsPerSec);
    Progress[i]       = 0;
    StepperRunning[i] = true;
    LastRunStart[i]   = time(nullptr);
    motors.run(i, absSteps, speedStepsPerSec, cw, onTargetReached);
}

void doRunVolume(int ch1based, float vol, float spd, const char* dir, const char* trig) {
    if (ch1based < 1 || ch1based > NUM_CH) return;
    if (vol <= 0 || spd <= 0) return;
    int i = ch1based - 1;

    if (DailyVolumeCap[i] > 0 && DailyVolume[i] + vol > DailyVolumeCap[i]) {
        Serial.printf("Channel %d daily cap reached (%.1f/%d mL); run rejected\n",
                      ch1based, DailyVolume[i], DailyVolumeCap[i]);
        if (mqttEnabled) {
            char topic[64]; snprintf(topic, sizeof(topic), "peristaltica/event/cap_reached/ch%d", ch1based);
            mqttClient.publish(topic, 0, false, "1");
        }
        return;
    }

    bool ccw = !strcmp(dir, "ccw");
    float v  = ccw ? -vol : vol;
    long steps = (long)(v * StepsPerMili[i]);
    float stepsPerSec = spd * StepsPerMili[i] / 60.0f;

    DurationMode[i] = false;
    DailyVolume[i] += vol;
    logHistoryEvent(ch1based, vol, trig);
    applyRunCommon(i, steps, stepsPerSec);
    publishChannelState(ch1based);
}

void doRunDuration(int ch1based, uint32_t durSec, float spd, const char* dir, const char* trig) {
    if (ch1based < 1 || ch1based > NUM_CH) return;
    if (durSec == 0 || spd <= 0) return;
    int i = ch1based - 1;

    bool ccw = !strcmp(dir, "ccw");
    float vol = spd * durSec / 60.0f;

    if (DailyVolumeCap[i] > 0 && DailyVolume[i] + vol > DailyVolumeCap[i]) {
        Serial.printf("Channel %d daily cap would be exceeded; duration run rejected\n", ch1based);
        return;
    }

    float v = ccw ? -vol : vol;
    long steps = (long)(v * StepsPerMili[i]);
    float stepsPerSec = spd * StepsPerMili[i] / 60.0f;

    DurationMode[i] = true;
    DailyVolume[i] += vol;
    logHistoryEvent(ch1based, vol, trig);
    applyRunCommon(i, steps, stepsPerSec);
    publishChannelState(ch1based);
}

void doStop(int ch1based) {
    if (ch1based == 0) {
        for (int i = 0; i < NUM_CH; i++) {
            if (StepperRunning[i]) {
                motors.stop(i);
                StepperRunning[i] = false;
                onEmergencyStop(i + 1);
            }
        }
        motors.disableMotors();
        return;
    }
    if (ch1based < 1 || ch1based > NUM_CH) return;
    int i = ch1based - 1;
    if (StepperRunning[i]) {
        motors.stop(i);
        StepperRunning[i] = false;
        onEmergencyStop(ch1based);
    }
}


// =========================================================
// PROGRESS + DEFERRED MQTT
// =========================================================
long ProgressPreviousMillis = 0;
const long ProgressInterval = 5000;

void checkProgress() {
    long now = millis();
    if (now - ProgressPreviousMillis > ProgressInterval) {
        ProgressPreviousMillis = now;
        for (int i = 0; i < NUM_CH; i++) {
            if (!StepperRunning[i]) continue;
            long total = StepsToMove[i];
            if (total > 0) {
                long done = total - motors.stepsRemaining(i);
                if (done < 0) done = 0;
                Progress[i] = round(100.0f * done / (float)total);
            }
            if (mqttEnabled) publishChannelState(i + 1);
        }
    }
}

void publishPendingMqttEvents() {
    if (!mqttEnabled) {
        for (int i=0; i<NUM_CH; i++) pendingMqtt[i].pending = false;
        return;
    }
    for (int i = 0; i < NUM_CH; i++) {
        if (!pendingMqtt[i].pending) continue;
        PendingMqttEvent ev = const_cast<PendingMqttEvent&>(pendingMqtt[i]);
        pendingMqtt[i].pending = false;
        publishChannelState(ev.channel);
    }
}


// =========================================================
// MQTT — structured topics + HA discovery
// =========================================================
String haDeviceId;
const char* HA_DISCOVERY_PREFIX = "homeassistant";

static String topicChannelState(int ch1based) {
    String t = "peristaltica/ch"; t += ch1based; t += "/state"; return t;
}
static String topicSensorState(int ch1based) {
    String t = "peristaltica/sensor"; t += ch1based; t += "/state"; return t;
}
static String topicCommand(int ch1based) {
    String t = "peristaltica/ch"; t += ch1based; t += "/cmd"; return t;
}

void publishChannelState(int ch1based) {
    if (!mqttEnabled) return;
    int i = ch1based - 1;
    StaticJsonDocument<256> doc;
    doc["running"]  = (bool)StepperRunning[i];
    doc["progress"] = Progress[i];
    doc["daily_ml"] = DailyVolume[i];
    doc["cap_ml"]   = DailyVolumeCap[i];
    char buf[256]; serializeJson(doc, buf);
    mqttClient.publish(topicChannelState(ch1based).c_str(), 0, true, buf);
}

void publishSensorState(int ch1based) {
    if (!mqttEnabled) return;
    int i = ch1based - 1;
    if (!sensorCfg[i].assigned || !sensorData[i].valid) return;
    StaticJsonDocument<256> doc;
    SENSOR_LOCK();
    doc["moisture"]     = sensorData[i].moisture;
    doc["temperature"]  = sensorData[i].temperature;
    doc["light"]        = sensorData[i].light;
    doc["conductivity"] = sensorData[i].conductivity;
    doc["battery"]      = sensorData[i].battery;
    doc["timestamp"]    = (uint32_t)sensorData[i].timestamp;
    SENSOR_UNLOCK();
    char buf[256]; serializeJson(doc, buf);
    mqttClient.publish(topicSensorState(ch1based).c_str(), 0, true, buf);
}

void publishHaDiscovery() {
    if (!mqttEnabled) return;

    auto deviceBlock = [&](JsonObject d){
        d["identifiers"][0] = haDeviceId;
        d["name"]           = "Peristaltica";
        d["manufacturer"]   = "DIY";
        d["model"]          = "MKS DLC32 v2.1 + 3 stepper pumps";
        d["sw_version"]     = BUILD_VERSION;
    };

    for (int ch = 1; ch <= NUM_CH; ch++) {
        {
            StaticJsonDocument<512> d;
            d["name"] = String("Bomba ") + ch;
            d["uniq_id"] = haDeviceId + "_pump" + ch;
            d["cmd_t"] = topicCommand(ch);
            d["payload_press"] = String("{\"action\":\"run\",\"volume\":50,\"speed\":10,\"direction\":\"cw\"}");
            deviceBlock(d.createNestedObject("dev"));
            String topic = String(HA_DISCOVERY_PREFIX) + "/button/" + haDeviceId + "_p" + ch + "/config";
            char buf[512]; serializeJson(d, buf);
            mqttClient.publish(topic.c_str(), 0, true, buf);
        }
        {
            StaticJsonDocument<512> d;
            d["name"] = String("Bomba ") + ch + " progreso";
            d["uniq_id"] = haDeviceId + "_progress" + ch;
            d["stat_t"] = topicChannelState(ch);
            d["val_tpl"] = "{{ value_json.progress }}";
            d["unit_of_meas"] = "%";
            deviceBlock(d.createNestedObject("dev"));
            String topic = String(HA_DISCOVERY_PREFIX) + "/sensor/" + haDeviceId + "_pr" + ch + "/config";
            char buf[512]; serializeJson(d, buf);
            mqttClient.publish(topic.c_str(), 0, true, buf);
        }
        {
            StaticJsonDocument<512> d;
            d["name"] = String("Bomba ") + ch + " volumen hoy";
            d["uniq_id"] = haDeviceId + "_daily" + ch;
            d["stat_t"] = topicChannelState(ch);
            d["val_tpl"] = "{{ value_json.daily_ml }}";
            d["unit_of_meas"] = "mL";
            deviceBlock(d.createNestedObject("dev"));
            String topic = String(HA_DISCOVERY_PREFIX) + "/sensor/" + haDeviceId + "_d" + ch + "/config";
            char buf[512]; serializeJson(d, buf);
            mqttClient.publish(topic.c_str(), 0, true, buf);
        }
        const char* fields[][3] = {
            {"moisture","Humedad","%"},
            {"temperature","Temperatura","C"},
            {"light","Luz","lx"},
            {"conductivity","Conductividad","uS/cm"},
            {"battery","Bateria","%"},
        };
        for (auto& f : fields) {
            StaticJsonDocument<512> d;
            d["name"] = String("Sensor ") + ch + " " + f[1];
            d["uniq_id"] = haDeviceId + "_s" + ch + "_" + f[0];
            d["stat_t"] = topicSensorState(ch);
            d["val_tpl"] = String("{{ value_json.") + f[0] + " }}";
            d["unit_of_meas"] = f[2];
            deviceBlock(d.createNestedObject("dev"));
            String topic = String(HA_DISCOVERY_PREFIX) + "/sensor/" + haDeviceId + "_" + f[0] + ch + "/config";
            char buf[512]; serializeJson(d, buf);
            mqttClient.publish(topic.c_str(), 0, true, buf);
        }
    }
    Serial.println("HA discovery published");
}

void connectToMqtt() {
    if (!mqttEnabled) return;
    if (mqttClient.connected()) return;
    Serial.println("Connecting to MQTT...");
    mqttClient.connect();
}

void startMqtt() {
    if (!mqttEnabled) return;
    mqttClient.setServer(mqttServer.c_str(), mqttPort);
    if (mqttUser.length()) mqttClient.setCredentials(mqttUser.c_str(), mqttPass.c_str());
    mqttClient.connect();
}

void onMqttConnect(bool) {
    Serial.println("MQTT connected");
    for (int ch = 1; ch <= NUM_CH; ch++) mqttClient.subscribe(topicCommand(ch).c_str(), 1);
    mqttClient.subscribe("peristaltica/action", 1);
    mqttClient.publish("peristaltica/system", 0, true,
        ("{\"status\":\"online\",\"ip\":\"" + WiFi.localIP().toString() +
         "\",\"version\":\"" BUILD_VERSION "\"}").c_str());
    publishHaDiscovery();
    for (int ch = 1; ch <= NUM_CH; ch++) publishChannelState(ch);
}
void onMqttDisconnect(AsyncMqttClientDisconnectReason) {
    Serial.println("MQTT disconnected");
    if (WiFi.isConnected()) xTimerStart(mqttReconnectTimer, 0);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties, size_t len, size_t, size_t) {
    StaticJsonDocument<256> doc;
    if (deserializeJson(doc, payload, len)) return;

    int chFromTopic = 0;
    if (sscanf(topic, "peristaltica/ch%d/cmd", &chFromTopic) == 1) {
        const char* action = doc["action"] | "";
        if (!strcmp(action, "run")) {
            if (doc.containsKey("duration"))
                doRunDuration(chFromTopic, doc["duration"]|0, doc["speed"]|10.0f, doc["direction"]|"cw", "mqtt");
            else
                doRunVolume(chFromTopic, doc["volume"]|0.0f, doc["speed"]|10.0f, doc["direction"]|"cw", "mqtt");
        } else if (!strcmp(action, "stop")) {
            doStop(chFromTopic);
        }
        return;
    }

    const char* action = doc["action"] | "";
    int ch = doc["channel"] | 0;
    if (!strcmp(action, "run")) {
        if (doc.containsKey("duration")) doRunDuration(ch, doc["duration"]|0, doc["speed"]|10.0f, doc["direction"]|"cw", "mqtt");
        else doRunVolume(ch, doc["volume"]|0.0f, doc["speed"]|10.0f, doc["direction"]|"cw", "mqtt");
    }
    else if (!strcmp(action, "stop"))      doStop(ch);
    else if (!strcmp(action, "calibrate")) saveCalibration(ch, doc["stepsperml"]|1600L);
}


// =========================================================
// WIFI
// =========================================================
void reconnectWifi() { WiFi.reconnect(); }
static void mqttTimerCb(TimerHandle_t) { connectToMqtt(); }
static void wifiTimerCb(TimerHandle_t) { reconnectWifi(); }

void WiFiEvent(WiFiEvent_t event) {
    switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
        Serial.print("WiFi OK - "); Serial.println(WiFi.localIP());
        connectToMqtt();
        break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
        Serial.println("WiFi lost");
        xTimerStop(mqttReconnectTimer, 0);
        xTimerStart(wifiReconnectTimer, 0);
        break;
    default: break;
    }
}


// =========================================================
// CORE 0: OTA + BLE
// =========================================================
TaskHandle_t C0;

void core0assignments(void*) {
    esp_task_wdt_delete(NULL);
    for (;;) {
        ArduinoOTA.handle();
        if (bleScanReq) { bleScanReq = false; doBleScan(); }
        if (bleReadReq) {
            bleReadReq = false;
            doReadAllSensors();
            for (int ch = 1; ch <= NUM_CH; ch++) publishSensorState(ch);
        }
        vTaskDelay(1);
    }
}


// =========================================================
// CALIBRATION WIZARD STATE
// =========================================================
struct CalRun {
    bool      active;
    int       channel;     // 1-based
    uint32_t  steps;       // steps to move (and how many we recorded)
    long      startPos;
} calRun = { false, 0, 0, 0 };


// =========================================================
// WEB SERVER
// =========================================================
String currentTimeStr() {
    struct tm t; if (!getLocalTime(&t, 0)) return "";
    char buf[10]; strftime(buf, sizeof(buf), "%H:%M:%S", &t);
    return String(buf);
}

void setupWebServer() {

    webServer.on("/", HTTP_GET, [](AsyncWebServerRequest* req){
        req->send_P(200, "text/html", index_html);
    });

    webServer.on("/api/status", HTTP_GET, [](AsyncWebServerRequest* req){
        StaticJsonDocument<512> doc;
        doc["ip"]       = WiFi.localIP().toString();
        doc["mqtt"]     = mqttClient.connected();
        doc["time"]     = currentTimeStr();
        doc["vacation"] = vacationMode;
        JsonArray m  = doc.createNestedArray("motors");
        JsonArray dy = doc.createNestedArray("daily");
        JsonArray cp = doc.createNestedArray("caps");
        for (int i = 0; i < NUM_CH; i++) {
            JsonObject mo = m.createNestedObject();
            mo["running"]  = (bool)StepperRunning[i];
            mo["progress"] = Progress[i];
            dy.add(DailyVolume[i]);
            cp.add(DailyVolumeCap[i]);
        }
        char buf[512]; serializeJson(doc, buf);
        req->send(200, "application/json", buf);
    });

    webServer.on("/api/system", HTTP_GET, [](AsyncWebServerRequest* req){
        StaticJsonDocument<256> doc;
        doc["version"] = BUILD_VERSION;
        doc["uptime"]  = millis() / 1000;
        doc["heap"]    = ESP.getFreeHeap();
        doc["rssi"]    = WiFi.RSSI();
        doc["ip"]      = WiFi.localIP().toString();
        doc["mac"]     = WiFi.macAddress();
        char buf[256]; serializeJson(doc, buf);
        req->send(200, "application/json", buf);
    });

    webServer.on("/api/system/reboot", HTTP_POST, [](AsyncWebServerRequest* req){
        req->send(200, "application/json", "{\"ok\":true}");
        delay(300); ESP.restart();
    });

    webServer.on("/api/calibrate", HTTP_GET, [](AsyncWebServerRequest* req){
        if (!req->hasParam("channel")) { req->send(400); return; }
        int ch = req->getParam("channel")->value().toInt();
        if (ch < 1 || ch > NUM_CH) { req->send(400); return; }
        StaticJsonDocument<64> doc; doc["stepsperml"] = StepsPerMili[ch-1];
        char buf[64]; serializeJson(doc, buf);
        req->send(200, "application/json", buf);
    });

    webServer.on("/api/cap", HTTP_GET, [](AsyncWebServerRequest* req){
        if (!req->hasParam("channel")) { req->send(400); return; }
        int ch = req->getParam("channel")->value().toInt();
        if (ch < 1 || ch > NUM_CH) { req->send(400); return; }
        StaticJsonDocument<64> doc; doc["cap"] = DailyVolumeCap[ch-1];
        char buf[64]; serializeJson(doc, buf);
        req->send(200, "application/json", buf);
    });

    webServer.on("/api/config", HTTP_GET, [](AsyncWebServerRequest* req){
        StaticJsonDocument<192> doc;
        doc["server"] = mqttServer; doc["port"] = mqttPort; doc["user"] = mqttUser;
        char buf[192]; serializeJson(doc, buf); req->send(200, "application/json", buf);
    });

    webServer.on("/api/schedules", HTTP_GET, [](AsyncWebServerRequest* req){
        StaticJsonDocument<2048> doc; JsonArray arr = doc.to<JsonArray>();
        for (int i = 0; i < MAX_SCHEDULES; i++) {
            if (schedules[i].days == 0) continue;
            JsonObject o = arr.createNestedObject();
            o["id"]=i; o["enabled"]=schedules[i].enabled; o["channel"]=schedules[i].channel;
            o["days"]=schedules[i].days; o["hour"]=schedules[i].hour; o["minute"]=schedules[i].minute;
            o["volume"]=schedules[i].volume; o["speed"]=schedules[i].speed;
            o["direction"]=schedules[i].direction; o["moistureThreshold"]=schedules[i].moistureThreshold;
        }
        char buf[2048]; serializeJson(doc, buf); req->send(200, "application/json", buf);
    });

    webServer.on("/api/schedules", HTTP_DELETE, [](AsyncWebServerRequest* req){
        if (!req->hasParam("id")) { req->send(400); return; }
        int id = req->getParam("id")->value().toInt();
        if (id < 0 || id >= MAX_SCHEDULES) { req->send(400); return; }
        deleteSchedule(id); req->send(200, "application/json", "{\"ok\":true}");
    });

    webServer.on("/api/sensors", HTTP_GET, [](AsyncWebServerRequest* req){
        StaticJsonDocument<1024> doc; JsonArray arr = doc.to<JsonArray>();
        SENSOR_LOCK();
        SensorConfig  cfgCopy[NUM_CH];
        SensorReading dataCopy[NUM_CH];
        memcpy(cfgCopy,  sensorCfg,  sizeof(cfgCopy));
        memcpy(dataCopy, sensorData, sizeof(dataCopy));
        SENSOR_UNLOCK();
        for (int i = 0; i < NUM_CH; i++) {
            JsonObject o = arr.createNestedObject();
            JsonObject cfg = o.createNestedObject("cfg");
            cfg["assigned"]=cfgCopy[i].assigned; cfg["mac"]=cfgCopy[i].mac;
            cfg["name"]=cfgCopy[i].name; cfg["threshold"]=cfgCopy[i].threshold;
            JsonObject rd = o.createNestedObject("reading");
            rd["valid"]=dataCopy[i].valid; rd["moisture"]=dataCopy[i].moisture;
            rd["temperature"]=dataCopy[i].temperature; rd["light"]=dataCopy[i].light;
            rd["conductivity"]=dataCopy[i].conductivity; rd["battery"]=dataCopy[i].battery;
            rd["timestamp"]=(uint32_t)dataCopy[i].timestamp;
        }
        char buf[1024]; serializeJson(doc, buf); req->send(200, "application/json", buf);
    });

    webServer.on("/api/sensors", HTTP_DELETE, [](AsyncWebServerRequest* req){
        if (!req->hasParam("channel")) { req->send(400); return; }
        int ch = req->getParam("channel")->value().toInt();
        if (ch < 0 || ch > 2) { req->send(400); return; }
        deleteSensorConfig(ch); req->send(200, "application/json", "{\"ok\":true}");
    });

    webServer.on("/api/ble/results", HTTP_GET, [](AsyncWebServerRequest* req){
        if (bleScanning) { req->send(200, "application/json", "{\"scanning\":true}"); return; }
        StaticJsonDocument<2048> doc; JsonArray arr = doc.to<JsonArray>();
        SENSOR_LOCK();
        int n = scanResultCount;
        BLEScanResult copy[MAX_SCAN_RESULTS];
        memcpy(copy, scanResults, sizeof(BLEScanResult) * n);
        SENSOR_UNLOCK();
        for (int i = 0; i < n; i++) {
            JsonObject o = arr.createNestedObject();
            o["addr"]=copy[i].addr; o["name"]=copy[i].name; o["rssi"]=copy[i].rssi;
        }
        char buf[2048]; serializeJson(doc, buf); req->send(200, "application/json", buf);
    });

    webServer.on("/api/timezone", HTTP_GET, [](AsyncWebServerRequest* req){
        PREFS_LOCK();
        prefs.begin("peristaltica", true);
        String tz = prefs.getString("timezone", "CET-1CEST,M3.5.0,M10.5.0/3");
        prefs.end();
        PREFS_UNLOCK();
        StaticJsonDocument<128> doc; doc["tz"] = tz;
        char buf[128]; serializeJson(doc, buf); req->send(200, "application/json", buf);
    });

    webServer.on("/api/ntp", HTTP_GET, [](AsyncWebServerRequest* req){
        StaticJsonDocument<256> doc;
        doc["ntp1"] = ntpServer1;
        doc["ntp2"] = ntpServer2;
        char buf[256]; serializeJson(doc, buf); req->send(200, "application/json", buf);
    });

    webServer.on("/api/quiet", HTTP_GET, [](AsyncWebServerRequest* req){
        StaticJsonDocument<64> doc;
        doc["start"] = quietStartHour; doc["end"] = quietEndHour;
        char buf[64]; serializeJson(doc, buf); req->send(200, "application/json", buf);
    });

    auto bodyHandler = [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total){
        const size_t MAX_BODY = 1024;
        if (total > MAX_BODY) { req->send(413, "application/json", "{\"ok\":false,\"error\":\"too_large\"}"); return; }

        char* buf = (char*) req->_tempObject;
        if (index == 0) {
            if (buf) { free(buf); buf = nullptr; }
            buf = (char*) malloc(total + 1);
            if (!buf) { req->send(500); return; }
            req->_tempObject = buf;
        }
        if (!buf) return;
        memcpy(buf + index, data, len);
        if (index + len < total) return;
        buf[total] = 0;

        StaticJsonDocument<512> doc;
        DeserializationError err = deserializeJson(doc, buf, total);
        free(buf); req->_tempObject = nullptr;
        if (err) { req->send(400, "application/json", "{\"ok\":false}"); return; }

        String path = req->url();

        if (path == "/api/run") {
            int ch = doc["channel"] | 0;
            if (ch < 1 || ch > NUM_CH) { req->send(400); return; }
            if (doc.containsKey("duration"))
                doRunDuration(ch, doc["duration"]|0, doc["speed"]|10.0f, doc["direction"]|"cw");
            else
                doRunVolume(ch, doc["volume"]|0.0f, doc["speed"]|10.0f, doc["direction"]|"cw");
            req->send(200, "application/json", "{\"ok\":true}");
        }
        else if (path == "/api/stop") {
            int ch = doc["channel"] | 0;
            if (ch < 0 || ch > NUM_CH) { req->send(400); return; }
            doStop(ch);
            req->send(200, "application/json", "{\"ok\":true}");
        }
        else if (path == "/api/calibrate") {
            int ch = doc["channel"] | 0;
            if (ch < 1 || ch > NUM_CH) { req->send(400); return; }
            saveCalibration(ch, doc["stepsperml"] | 1600L);
            req->send(200, "application/json", "{\"ok\":true}");
        }
        else if (path == "/api/calibrate/run") {
            int ch = doc["channel"] | 0;
            uint32_t durSec = doc["duration"] | 0;
            if (ch < 1 || ch > NUM_CH || durSec == 0 || durSec > 300) { req->send(400); return; }
            int i = ch - 1;
            calRun.active = true; calRun.channel = ch;
            calRun.startPos = 0;
            float spd = 10.0f;
            doRunDuration(ch, durSec, spd, "cw", "cal");  // "cal" suppresses history logging
            calRun.steps = (uint32_t)StepsToMove[i];
            req->send(200, "application/json", "{\"ok\":true}");
        }
        else if (path == "/api/calibrate/compute") {
            int ch = doc["channel"] | 0;
            float ml = doc["ml"] | 0.0f;
            if (ch < 1 || ch > NUM_CH || ml <= 0) { req->send(400); return; }
            int i = ch - 1;
            uint32_t steps = calRun.steps;
            if (calRun.channel != ch || steps == 0) {
                long total = motors.stepsTotal(i);
                long rem   = motors.stepsRemaining(i);
                long done  = total - rem; if (done < 0) done = 0;
                steps = (uint32_t)done;
            }
            if (steps == 0) { req->send(400, "application/json", "{\"ok\":false,\"error\":\"no_run\"}"); return; }
            long spm = (long)round(steps / ml);
            saveCalibration(ch, spm);
            calRun.active = false;
            StaticJsonDocument<64> r; r["ok"] = true; r["stepsperml"] = spm;
            char rbuf[64]; serializeJson(r, rbuf);
            req->send(200, "application/json", rbuf);
        }
        else if (path == "/api/cap") {
            int ch = doc["channel"] | 0;
            int cap = doc["cap"] | 0;
            if (ch < 1 || ch > NUM_CH || cap < 0 || cap > 65535) { req->send(400); return; }
            saveDailyCap(ch, (uint16_t)cap);
            req->send(200, "application/json", "{\"ok\":true}");
        }
        else if (path == "/api/config") {
            saveMqttConfig(doc["server"]|"", doc["port"]|MQTT_PORT_DEFAULT, doc["user"]|"", doc["pass"]|"");
            if (mqttEnabled) {
                mqttClient.disconnect(); delay(200);
                mqttClient.setServer(mqttServer.c_str(), mqttPort);
                if (mqttUser.length()) mqttClient.setCredentials(mqttUser.c_str(), mqttPass.c_str());
                mqttClient.connect();
            }
            req->send(200, "application/json", "{\"ok\":true}");
        }
        else if (path == "/api/schedules") {
            int id = doc["id"] | -1;
            if (doc.containsKey("channel")) {
                int c = doc["channel"];
                if (c < 1 || c > NUM_CH) { req->send(400, "application/json", "{\"ok\":false,\"error\":\"bad_channel\"}"); return; }
            }
            if (id >= 0 && id < MAX_SCHEDULES) {
                if (doc.containsKey("enabled"))           schedules[id].enabled=doc["enabled"];
                if (doc.containsKey("days"))              schedules[id].days=doc["days"];
                if (doc.containsKey("hour"))              schedules[id].hour=doc["hour"];
                if (doc.containsKey("minute"))            schedules[id].minute=doc["minute"];
                if (doc.containsKey("volume"))            schedules[id].volume=doc["volume"];
                if (doc.containsKey("speed"))             schedules[id].speed=doc["speed"];
                if (doc.containsKey("channel"))           schedules[id].channel=doc["channel"];
                if (doc.containsKey("direction"))         strlcpy(schedules[id].direction,doc["direction"]|"cw",4);
                if (doc.containsKey("moistureThreshold")) schedules[id].moistureThreshold=doc["moistureThreshold"];
                saveSchedule(id);
            } else {
                bool ok = false;
                for (int i = 0; i < MAX_SCHEDULES && !ok; i++) {
                    if (schedules[i].days != 0) continue;
                    schedules[i].enabled=doc["enabled"]|true; schedules[i].channel=doc["channel"]|1;
                    schedules[i].days=doc["days"]|0; schedules[i].hour=doc["hour"]|0; schedules[i].minute=doc["minute"]|0;
                    schedules[i].volume=doc["volume"]|0.0f; schedules[i].speed=doc["speed"]|10.0f;
                    schedules[i].moistureThreshold=doc["moistureThreshold"]|0;
                    strlcpy(schedules[i].direction, doc["direction"]|"cw", 4);
                    saveSchedule(i); ok = true;
                }
                if (!ok) { req->send(507, "application/json", "{\"ok\":false,\"error\":\"full\"}"); return; }
            }
            req->send(200, "application/json", "{\"ok\":true}");
        }
        else if (path == "/api/sensors") {
            int ch = doc["channel"] | 0;
            const char* mac = doc["mac"] | "";
            if (ch < 0 || ch > 2) { req->send(400); return; }
            if (!isValidMac(mac)) { req->send(400, "application/json", "{\"ok\":false,\"error\":\"bad_mac\"}"); return; }
            SENSOR_LOCK();
            sensorCfg[ch].assigned = true;
            sensorCfg[ch].threshold = doc["threshold"] | 0;
            strlcpy(sensorCfg[ch].mac,  mac, 18);
            strlcpy(sensorCfg[ch].name, doc["name"] | "", 40);
            SENSOR_UNLOCK();
            saveSensorConfig(ch);
            bleReadReq = true;
            req->send(200, "application/json", "{\"ok\":true}");
        }
        else if (path == "/api/sensors/read") {
            if (anySensorAssigned()) bleReadReq = true;
            req->send(200, "application/json", "{\"ok\":true}");
        }
        else if (path == "/api/ble/scan") {
            if (!bleScanning) bleScanReq = true;
            req->send(200, "application/json", "{\"ok\":true,\"scanning\":true}");
        }
        else if (path == "/api/timezone") {
            String tz = doc["tz"] | "CET-1CEST,M3.5.0,M10.5.0/3";
            PREFS_LOCK();
            prefs.begin("peristaltica", false); prefs.putString("timezone", tz); prefs.end();
            PREFS_UNLOCK();
            setenv("TZ", tz.c_str(), 1); tzset();
            req->send(200, "application/json", "{\"ok\":true}");
        }
        else if (path == "/api/ntp") {
            String n1 = doc["ntp1"] | "pool.ntp.org";
            String n2 = doc["ntp2"] | "time.cloudflare.com";
            PREFS_LOCK();
            prefs.begin("peristaltica", false);
            prefs.putString("ntp1", n1); prefs.putString("ntp2", n2);
            prefs.end();
            PREFS_UNLOCK();
            ntpServer1 = n1; ntpServer2 = n2;
            configTime(0, 0, ntpServer1.c_str(), ntpServer2.c_str());
            req->send(200, "application/json", "{\"ok\":true}");
        }
        else if (path == "/api/quiet") {
            uint8_t s = doc["start"] | 0;
            uint8_t e = doc["end"]   | 0;
            if (s > 23 || e > 23) { req->send(400); return; }
            saveQuietHours(s, e);
            req->send(200, "application/json", "{\"ok\":true}");
        }
        else if (path == "/api/vacation") {
            bool en    = doc["enabled"] | false;
            uint32_t u = doc["until"]   | 0;
            vacationMode  = en;
            vacationUntil = (time_t)u;
            saveVacationMode();
            req->send(200, "application/json", "{\"ok\":true}");
        }
        else if (path == "/api/resetwifi") {
            req->send(200, "application/json", "{\"ok\":true}");
            delay(500);
            WiFi.disconnect(true, true);
            ESP.restart();
        }
        else { req->send(404); }
    };

    const char* posts[] = {
        "/api/run", "/api/stop", "/api/calibrate", "/api/calibrate/run", "/api/calibrate/compute",
        "/api/cap", "/api/config", "/api/schedules", "/api/sensors", "/api/sensors/read",
        "/api/ble/scan", "/api/timezone", "/api/ntp", "/api/quiet", "/api/resetwifi",
        "/api/vacation"
    };
    for (auto p : posts) {
        webServer.on(p, HTTP_POST, [](AsyncWebServerRequest*){}, NULL, bodyHandler);
    }

    // ---- Irrigation history ----
    webServer.on("/api/history", HTTP_GET, [](AsyncWebServerRequest* req){
        String json = "{\"events\":[";
        int n = min(histCount, HIST_MAX);
        for (int i = 0; i < n; i++) {
            int idx = ((histHead - n + i) % HIST_MAX + HIST_MAX) % HIST_MAX;
            if (i) json += ',';
            json += "{\"ts\":";  json += histBuf[idx].ts;
            json += ",\"ch\":";  json += (int)histBuf[idx].ch;
            json += ",\"v\":";   json += String(histBuf[idx].vol, 1);
            json += ",\"t\":\""; json += histBuf[idx].trig; json += "\"}";
        }
        json += "]}";
        req->send(200, "application/json", json);
    });

    webServer.on("/api/history", HTTP_DELETE, [](AsyncWebServerRequest* req){
        clearHistory();
        req->send(200, "application/json", "{\"ok\":true}");
    });

    // ---- Vacation mode ----
    webServer.on("/api/vacation", HTTP_GET, [](AsyncWebServerRequest* req){
        String json = "{\"enabled\":";
        json += vacationMode ? "true" : "false";
        json += ",\"until\":"; json += (uint32_t)vacationUntil;
        json += "}";
        req->send(200, "application/json", json);
    });

    // ---- OTA via browser (/update) ----
    webServer.on("/update", HTTP_GET, [](AsyncWebServerRequest* req) {
        req->send(200, "text/html", R"rawliteral(
<!DOCTYPE html><html lang="es">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>OTA – Perist&aacute;ltica</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:system-ui,sans-serif;background:#f0f2f5;color:#333}
header{background:#1565c0;color:#fff;padding:14px 20px}
header h1{font-size:1.1rem;font-weight:600}
main{max-width:520px;margin:24px auto;padding:0 14px}
.card{background:#fff;border-radius:10px;padding:22px;box-shadow:0 1px 5px rgba(0,0,0,.1)}
.card h2{font-size:.95rem;margin-bottom:14px;color:#1565c0;font-weight:600;
         border-bottom:1px solid #e3e8f0;padding-bottom:8px}
p{font-size:.86rem;color:#555;margin:8px 0;line-height:1.5}
code{background:#f0f2f5;padding:2px 6px;border-radius:4px;font-size:.83rem}
label{display:block;font-size:.78rem;color:#666;margin:14px 0 4px}
input[type=file]{width:100%;padding:7px;border:1px dashed #90a4ae;
                 border-radius:6px;font-size:.85rem;background:#fafafa;cursor:pointer}
.row{margin-top:16px}
button{width:100%;padding:10px;border:none;border-radius:6px;
       font-size:.92rem;font-weight:600;cursor:pointer;transition:opacity .15s}
.run{background:#1565c0;color:#fff}.run:disabled{opacity:.5;cursor:not-allowed}
#msg{margin-top:12px;font-size:.84rem;min-height:1.2em;font-weight:500}
.ok{color:#2e7d32}.err{color:#c62828}
.pb-wrap{display:none;margin-top:12px}
.pb{height:8px;background:#e0e0e0;border-radius:4px;overflow:hidden}
.pf{height:100%;background:#1565c0;border-radius:4px;width:0;transition:width .3s}
</style>
</head>
<body>
<header><h1>&#11014; Actualizaci&oacute;n OTA – Bomba Perist&aacute;ltica</h1></header>
<main>
<div class="card">
  <h2>Subir nuevo firmware</h2>
  <p>Selecciona el fichero <code>firmware.bin</code> generado por PlatformIO:<br>
     <small style="color:#888">.pio/build/mks_dlc32_v2_1/firmware.bin</small></p>
  <form id="frm" method="POST" action="/update" enctype="multipart/form-data">
    <label>Fichero .bin</label>
    <input type="file" name="firmware" id="file" accept=".bin" required>
    <div class="row">
      <button type="submit" class="run" id="btn">&#11014; Subir y actualizar</button>
    </div>
  </form>
  <div class="pb-wrap" id="pbWrap"><div class="pb"><div class="pf" id="pf"></div></div></div>
  <div id="msg"></div>
  <p style="margin-top:18px;font-size:.78rem;color:#aaa">
    &larr; <a href="/" style="color:#1565c0">Volver a la UI principal</a>
  </p>
</div>
</main>
<script>
const frm = document.getElementById('frm');
const btn = document.getElementById('btn');
const msg = document.getElementById('msg');
const pbWrap = document.getElementById('pbWrap');
const pf = document.getElementById('pf');

frm.onsubmit = async function(e) {
  e.preventDefault();
  const file = document.getElementById('file').files[0];
  if (!file) return;
  btn.disabled = true;
  btn.textContent = 'Actualizando... no apagues la placa';
  msg.className = ''; msg.textContent = 'Subiendo ' + (file.size/1024).toFixed(0) + ' KB...';
  pbWrap.style.display = 'block';

  const xhr = new XMLHttpRequest();
  xhr.open('POST', '/update');
  xhr.upload.onprogress = function(ev) {
    if (ev.lengthComputable) {
      const pct = Math.round(ev.loaded / ev.total * 100);
      pf.style.width = pct + '%';
      msg.textContent = 'Subiendo... ' + pct + '%';
    }
  };
  xhr.onload = function() {
    if (xhr.status === 200 && xhr.responseText.indexOf('OK') >= 0) {
      pf.style.width = '100%';
      msg.className = 'ok';
      msg.textContent = '✅ Actualización correcta. Reiniciando en 5 s...';
      setTimeout(() => { location.replace('/'); }, 8000);
    } else {
      msg.className = 'err';
      msg.textContent = '❌ Error: ' + xhr.responseText;
      btn.disabled = false;
      btn.textContent = '⬆ Subir y actualizar';
    }
  };
  xhr.onerror = function() {
    msg.className = 'ok';
    // Connection lost = board restarted = update probably OK
    msg.textContent = '✅ Placa reiniciando... espera 8 s.';
    setTimeout(() => { location.replace('/'); }, 8000);
  };
  const fd = new FormData();
  fd.append('firmware', file);
  xhr.send(fd);
};
</script>
</body>
</html>
)rawliteral");
    });

    webServer.on("/update", HTTP_POST,
        // Response handler — called after upload completes
        [](AsyncWebServerRequest* req) {
            bool ok = !Update.hasError();
            req->send(200, "text/plain", ok ? "OK" : Update.errorString());
            if (ok) otaRestartPending = true;
        },
        // Upload handler — called for each chunk of the incoming file
        [](AsyncWebServerRequest* req, String filename,
           size_t index, uint8_t* data, size_t len, bool final) {
            if (!index) {
                Serial.printf("OTA: start – %s (%u bytes)\n",
                              filename.c_str(), req->contentLength());
                if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) {
                    Serial.print("OTA begin error: ");
                    Update.printError(Serial);
                }
            }
            if (Update.isRunning()) {
                if (Update.write(data, len) != len) {
                    Serial.print("OTA write error: ");
                    Update.printError(Serial);
                }
            }
            if (final) {
                if (Update.end(true)) {
                    Serial.printf("OTA: done – %u bytes\n", index + len);
                } else {
                    Serial.print("OTA end error: ");
                    Update.printError(Serial);
                }
            }
        }
    );

    webServer.begin();
    Serial.println("Web server started");
}


// =========================================================
// SETUP
// =========================================================
void setup() {
    Serial.begin(115200);

    prefsMutex  = xSemaphoreCreateMutex();
    sensorMutex = xSemaphoreCreateMutex();
    fsMutex     = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(core0assignments, "Core_0", 10000, NULL, 1, &C0, 0);

    mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, mqttTimerCb);
    wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, wifiTimerCb);

    WiFi.onEvent(WiFiEvent);

    if (!LittleFS.begin(true)) Serial.println("LittleFS mount failed");
    loadMqttConfig();
    loadCalibration();
    loadSchedules();
    loadSensorConfigs();
    loadVacationMode();
    loadHistory();

    String mac = WiFi.macAddress(); mac.replace(":", "");
    haDeviceId = "peristaltica-" + mac.substring(6);

    // Init custom 74HC595-based stepper driver (DLC32 v2.1)
    motors.begin();

    // Init BLE BEFORE WiFi connects: ESP32 shares the radio between WiFi and BLE.
    // Initialising NimBLE after WiFi is already connected can trigger a panic
    // (radio contention during coexistence negotiation).  Starting it first lets
    // the IDF coexistence layer set up cleanly before WiFi takes the channel.
    NimBLEDevice::init("");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);

    WiFiManager wm;
    wm.setConfigPortalTimeout(180); wm.setConnectTimeout(30); wm.setHostname("peristaltica");
    if (!wm.autoConnect("Peristaltica-Setup")) {
        Serial.println("WiFiManager timeout - restarting"); ESP.restart();
    }

    if (MDNS.begin("peristaltica")) {
        MDNS.addService("http", "tcp", 80);
        Serial.println("mDNS: http://peristaltica.local");
    }

    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    mqttClient.onMessage(onMqttMessage);
    if (mqttEnabled) {
        mqttClient.setWill("peristaltica/system", 1, true, "{\"status\":\"offline\"}");
    }
    startMqtt();

    setupNTP();

    ArduinoOTA.setHostname("peristaltica");
    ArduinoOTA.begin();
    setupWebServer();

    esp_task_wdt_init(30, true);
    esp_task_wdt_add(NULL);

    Serial.print("Ready - http://"); Serial.println(WiFi.localIP());
}


// =========================================================
// LOOP
// =========================================================
void loop() {
    esp_task_wdt_reset();
    if (otaRestartPending) { delay(300); ESP.restart(); }
    publishPendingMqttEvents();
    checkProgress();
    rolloverDailyTotals();
    checkSchedules();

    static long lastSensorRead = -(15L*60*1000);
    if (millis() - lastSensorRead > 15L*60*1000) {
        lastSensorRead = millis();
        if (anySensorAssigned()) bleReadReq = true;
    }

    delay(1);
}
