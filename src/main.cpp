/*CEL MAI BUN COD FINAL ZIUA 1, MERGI TOT DAR NU PI WEBSITE
 * ================================================================
 * Hard&Soft Competition - Task 1
 * ESP32-C3 Super Mini — Environmental Monitor (Minimal MQTT)
 * ================================================================
 *
 * COMPONENTS:
 *   - ESP32-C3 Super Mini          (3.3V logic)
 *   - SSD1306 0.96" OLED 128x64   (I2C 0x3C)
 *   - INA219 current/power monitor (I2C 0x40)
 *   - DS1307 RTC module HW-111    (I2C 0x68)
 *   - PCF8591 ADC/DAC HW-011      (I2C 0x48)
 *   - LG INR18650MH1 battery      (3.6V nom, 3200mAh)
 *   - 2x tactile push buttons
 *
 * WIRING:
 *   I2C Bus:
 *     ESP32 GPIO8 (SDA) --> OLED, INA219, DS1307, PCF8591
 *     ESP32 GPIO9 (SCL) --> OLED, INA219, DS1307, PCF8591
 *   Buttons (active LOW, internal pull-up):
 *     ESP32 GPIO3 --> Button NEXT --> GND
 *     ESP32 GPIO4 --> Button PREV --> GND
 *
 * FIXES vs original:
 *   - PCF8591: triple-read with inter-read delay for stable ADC values
 *   - I2C clock set to 100kHz (PCF8591 & DS1307 are slow devices)
 *   - INA219: explicit 16V/400mA calibration for battery monitoring
 *   - Removed WebServer/HTTP/dashboard — MQTT + OLED only
 *   - Reduced RAM: smaller JSON buffer, no HTML blob
 *
 * LIBRARIES:
 *   - Adafruit SSD1306, Adafruit GFX, Adafruit INA219
 *   - RTClib (Adafruit), PubSubClient, ArduinoJson v7+
 */

#include <Arduino.h>
#include <algorithm>
#include <cmath>

#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_INA219.h>
#include <RTClib.h>
#include <ArduinoJson.h>
#include <time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_freertos_hooks.h>
#include <Preferences.h>

// ================================================================
// CONFIGURATION
// ================================================================
static const char* WIFI_SSID = "DIGI-75hC";
static const char* WIFI_PASS = "9T2Du96euz";

static const float BATTERY_CAPACITY = 3200.0f;  // mAh
static const float BATT_REST_CURRENT_MA = 80.0f;  // ~C/40 for 3200mAh cell
static const uint32_t BATT_REST_TIME_MS = 30000;
static const uint32_t BATT_SAVE_INTERVAL_MS = 60000;
static const char* PREF_NS = "battery";
static const char* PREF_KEY_USED_MAH = "used_mAh";

// ================================================================
// MQTT
// ================================================================
static const char* MQTT_BROKER      = "broker.emqx.io";
static const uint16_t MQTT_PORT     = 1883;
static const char* MQTT_TOPIC       = "hardandsoft/esp32/data";
static const char* MQTT_RAW_TOPIC   = "hardandsoft/esp32/gpio_raw";
static const char* MQTT_CMD_TOPIC   = "hardandsoft/esp32/cmd";
static const char* MQTT_STATE_TOPIC = "hardandsoft/esp32/state";
static const char* MQTT_USER        = "emqx";
static const char* MQTT_PASS_MQTT   = "public";

// ================================================================
// PINS
// ================================================================
static const uint8_t I2C_SDA  = 8;
static const uint8_t I2C_SCL  = 9;
static const uint8_t BTN_NEXT = 3;
static const uint8_t BTN_PREV = 4;

// GPIO snapshot pins to publish as raw digital states.
static const uint8_t GPIO_PUBLISH_PINS[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 18, 19, 20, 21};
static const size_t GPIO_PUBLISH_PIN_COUNT = sizeof(GPIO_PUBLISH_PINS) / sizeof(GPIO_PUBLISH_PINS[0]);

// ================================================================
// I2C
// ================================================================
static const uint8_t  PCF8591_ADDR  = 0x48;
static const uint32_t I2C_FREQ      = 100000;  // 100kHz — safe for PCF8591 & DS1307

// ================================================================
// PCF8591 CHANNEL MAP (HW-011, verified empirically)
// ================================================================
static const uint8_t CH_LDR   = 0;   // LDR (light)
static const uint8_t CH_THERM = 1;   // NTC thermistor (temperature)
static const uint8_t CH_EXT   = 2;   // External / unused
static const uint8_t CH_POT   = 3;   // Potentiometer

// ================================================================
// NTC THERMISTOR (typical 10k NTC on HW-011)
// Circuit: VCC -> R_fixed -> ADC -> NTC -> GND
// R_fixed is 1kΩ on this HW-011 module (not 10k!)
// ================================================================
static const float R_FIXED   = 1000.0f;
static const float R_NOMINAL = 10000.0f;
static const float T_NOMINAL = 25.0f;
static const float B_COEFF   = 3950.0f;

// ================================================================
// DISPLAY & TIMING
// ================================================================
static const uint8_t  SCREEN_W      = 128;
static const uint8_t  SCREEN_H      = 64;
static const uint8_t  NUM_PAGES     = 8;
static const uint16_t DEBOUNCE_MS   = 200;

// ================================================================
// OBJECTS
// ================================================================
static Adafruit_SSD1306 oled(SCREEN_W, SCREEN_H, &Wire, -1);
static Adafruit_INA219  ina;
static RTC_DS1307       rtc;
static WiFiClient       espClient;
static PubSubClient     mqtt(espClient);
static Preferences      prefs;

// ================================================================
// DEVICE STATUS FLAGS (set during setup, never change after)
// ================================================================
static bool hasOLED   = false;
static bool hasINA219 = false;
static bool hasRTC    = false;
static bool hasPCF    = false;

// ================================================================
// SENSOR DATA
// ================================================================
static float    gTemp       = 0.0f;
static uint8_t  gLightRaw   = 0;
static uint8_t  gThermRaw   = 0;
static uint8_t  gExtRaw     = 0;
static uint8_t  gPotRaw     = 0;
static float    gLightPct   = 0.0f;
static int16_t  gRSSI       = 0;
static float    gVoltage    = 0.0f;
static float    gCurrentMA  = 0.0f;
static float    gPowerMW    = 0.0f;
static uint16_t gInaBusRaw  = 0;
static int16_t  gInaCurrRaw = 0;
static uint16_t gInaPowRaw  = 0;
static float    gTotalMAh   = 0.0f;
static float    gBattPct    = 100.0f;
static float    gBattLife   = -1.0f;
static float    gCpuLoad    = 0.0f;
static char     gTimestamp[20] = "0000-00-00 00:00:00";
static uint32_t gUptime     = 0;
static bool     gBatteryPresent = true;

// ================================================================
// INTERNAL STATE
// ================================================================
static volatile int8_t page = 0;
static uint32_t lastEnergy  = 0;

// CPU load tracking
static volatile uint32_t cpuIdleHookCount = 0;
static uint32_t cpuLastIdleHookCount = 0;
static float cpuIdleHookBaseline = 0.0f;
static uint32_t cpuPrevTotalRunTime = 0;
static uint32_t cpuPrevIdleRunTime = 0;
static bool cpuRuntimeSeeded = false;

// Battery SOC tracking
static float gSocEstimatePct = 100.0f;
static int gBattPctShown = 100;
static uint32_t gRestAccumMs = 0;
static uint32_t gLastBattPersistMs = 0;

// Sensor read interval tracking
static uint32_t lastSensorRead = 0;

// Button ISR flags
static volatile bool isrNextFlag = false;
static volatile bool isrPrevFlag = false;

// MQTT client ID
static String mqttClientId;

// Module enable/disable (toggled via MQTT)
static bool moduleTemp    = true;
static bool moduleLight   = true;
static bool moduleCPU     = true;
static bool moduleCurrent = true;
static volatile bool moduleCpuStress = false;

// Forward declarations for functions used before their definitions.
static void publishState();
static void drawPage();
static float measureCpuLoadPct();
static void buildRawGpioJSON(char* buf, size_t bufSize);
static void printDegC();
static bool readNtpTimestamp(char* out, size_t outSize);
static bool cpuIdleHook();
static bool getCpuLoadFromRuntimeStats(float& loadOut);
static float getCpuLoadFromIdleHook();
static void cpuStressTask(void* parm);

static bool systemTimeValid() {
  time_t now = time(nullptr);
  // 2024-01-01 UTC guard to reject unsynced epoch values
  return now > 1704067200;
}

static void syncClockFromNtp() {
  if (WiFi.status() != WL_CONNECTED) return;

  configTzTime("EET-2EEST,M3.5.0/3,M10.5.0/4", "pool.ntp.org", "time.nist.gov");

  uint8_t tries = 0;
  while (!systemTimeValid() && tries < 25) {
    delay(200);
    tries++;
  }

  if (hasRTC && systemTimeValid()) {
    struct tm ti;
    if (getLocalTime(&ti, 100)) {
      rtc.adjust(DateTime(
        ti.tm_year + 1900,
        ti.tm_mon + 1,
        ti.tm_mday,
        ti.tm_hour,
        ti.tm_min,
        ti.tm_sec
      ));
    }
  }
}

static bool readNtpTimestamp(char* out, size_t outSize) {
  time_t now = time(nullptr);
  if (now < 1700000000) return false;

  struct tm tmNow;
  localtime_r(&now, &tmNow);
  snprintf(out, outSize, "%04d-%02d-%02d %02d:%02d:%02d",
           tmNow.tm_year + 1900, tmNow.tm_mon + 1, tmNow.tm_mday,
           tmNow.tm_hour, tmNow.tm_min, tmNow.tm_sec);
  return true;
}

static void loadBatteryState() {
  if (!prefs.begin(PREF_NS, false)) return;
  gTotalMAh = prefs.getFloat(PREF_KEY_USED_MAH, gTotalMAh);
  prefs.end();
  gTotalMAh = constrain(gTotalMAh, 0.0f, BATTERY_CAPACITY);
}

static void persistBatteryStateIfDue(uint32_t nowMs) {
  if (nowMs - gLastBattPersistMs < BATT_SAVE_INTERVAL_MS) return;
  gLastBattPersistMs = nowMs;

  if (!prefs.begin(PREF_NS, false)) return;
  prefs.putFloat(PREF_KEY_USED_MAH, gTotalMAh);
  prefs.end();
}

static float ocvToSocPct(float v) {
  static const float vPts[] = {4.20f, 4.10f, 4.00f, 3.90f, 3.80f, 3.70f, 3.60f, 3.50f, 3.40f, 3.30f, 3.20f};
  static const float sPts[] = {100.0f, 90.0f, 75.0f, 60.0f, 45.0f, 30.0f, 18.0f, 10.0f, 5.0f, 2.0f, 0.0f};

  if (v >= vPts[0]) return 100.0f;
  if (v <= vPts[10]) return 0.0f;

  for (int i = 0; i < 10; i++) {
    if (v <= vPts[i] && v >= vPts[i + 1]) {
      float t = (v - vPts[i + 1]) / (vPts[i] - vPts[i + 1]);
      return sPts[i + 1] + t * (sPts[i] - sPts[i + 1]);
    }
  }
  return 0.0f;
}

static float measureCpuLoadPct() {
  if (!moduleCPU) return 0.0f;

  float nativeLoad = 0.0f;
  if (getCpuLoadFromRuntimeStats(nativeLoad)) {
    return nativeLoad;
  }

  // Fallback when runtime stats are not available in this build.
  return getCpuLoadFromIdleHook();
}

static bool cpuIdleHook() {
  cpuIdleHookCount++;
  return true;
}

static bool getCpuLoadFromRuntimeStats(float& loadOut) {
#if (configUSE_TRACE_FACILITY == 1) && (configGENERATE_RUN_TIME_STATS == 1)
  UBaseType_t taskCount = uxTaskGetNumberOfTasks();
  if (taskCount == 0) return false;

  TaskStatus_t* stats = (TaskStatus_t*)pvPortMalloc(taskCount * sizeof(TaskStatus_t));
  if (stats == nullptr) return false;

  uint32_t totalRunTime = 0;
  UBaseType_t filled = uxTaskGetSystemState(stats, taskCount, &totalRunTime);
  uint32_t idleRunTime = 0;
  for (UBaseType_t i = 0; i < filled; i++) {
    if (strncmp(stats[i].pcTaskName, "IDLE", 4) == 0) {
      idleRunTime += stats[i].ulRunTimeCounter;
    }
  }
  vPortFree(stats);

  if (!cpuRuntimeSeeded) {
    cpuRuntimeSeeded = true;
    cpuPrevTotalRunTime = totalRunTime;
    cpuPrevIdleRunTime = idleRunTime;
    return false;
  }

  uint32_t dTotal = totalRunTime - cpuPrevTotalRunTime;
  uint32_t dIdle = idleRunTime - cpuPrevIdleRunTime;
  cpuPrevTotalRunTime = totalRunTime;
  cpuPrevIdleRunTime = idleRunTime;

  if (dTotal == 0) return false;

  float idlePct = (100.0f * (float)dIdle) / (float)dTotal;
  loadOut = constrain(100.0f - idlePct, 0.0f, 100.0f);
  return true;
#else
  (void)loadOut;
  return false;
#endif
}

static float getCpuLoadFromIdleHook() {
  uint32_t idleNow = cpuIdleHookCount;
  uint32_t idleDelta = idleNow - cpuLastIdleHookCount;
  cpuLastIdleHookCount = idleNow;

  if (cpuIdleHookBaseline < 1.0f) {
    cpuIdleHookBaseline = (float)idleDelta;
    return 0.0f;
  }

  if ((float)idleDelta > cpuIdleHookBaseline) {
    cpuIdleHookBaseline = (float)idleDelta;
  } else {
    cpuIdleHookBaseline = cpuIdleHookBaseline * 0.995f + (float)idleDelta * 0.005f;
  }

  if (cpuIdleHookBaseline < 1.0f) return 0.0f;

  float idlePct = (100.0f * (float)idleDelta) / cpuIdleHookBaseline;
  return constrain(100.0f - idlePct, 0.0f, 100.0f);
}

static void cpuStressTask(void* parm) {
  (void)parm;
  volatile float sink = 0.0001f;
  for (;;) {
    if (moduleCpuStress) {
      // Continuous heavy math while enabled from MQTT.
      for (int i = 1; i <= 800; i++) {
        float x = (float)i + sink;
        sink += sqrtf(x) * logf(x + 1.0f);
        sink = fmodf(sink, 1000.0f);
      }
      taskYIELD();
    } else {
      vTaskDelay(pdMS_TO_TICKS(20));
    }
  }
}

// ================================================================
// ISRs (buttons only — sensor read uses millis() for portability)
// ================================================================
void IRAM_ATTR isrBtnNext() { isrNextFlag = true; }
void IRAM_ATTR isrBtnPrev() { isrPrevFlag = true; }

// ================================================================
// PCF8591 — STABLE READ
//
// Control byte format (written to PCF8591):
//   Bit 7:   0
//   Bit 6:   1 = analog output enable (REQUIRED on HW-011!)
//   Bit 5-4: 00 = four single-ended inputs
//   Bit 3:   0
//   Bit 2:   0 = no auto-increment
//   Bit 1-0: channel number (0-3)
//
// Without bit 6 set, the ADC returns 0xFF on all channels.
//
// The first byte read after a channel switch is always the
// PREVIOUS conversion result. We request 2 bytes in one
// transaction: discard byte 0, keep byte 1.
// Two back-to-back transactions ensure full settling.
// ================================================================
static uint8_t readPCF8591Stable(uint8_t channel) {
  uint8_t ctrl = 0x40 | (channel & 0x03);  // bit6=1 (output enable) + channel

  // Transaction 1: switch channel, discard stale byte, get first conversion
  Wire.beginTransmission(PCF8591_ADDR);
  Wire.write(ctrl);
  if (Wire.endTransmission() != 0) return 0;

  delay(1);  // 1ms — let S&H capacitor settle after channel switch

  Wire.requestFrom((uint8_t)PCF8591_ADDR, (uint8_t)2);
  if (Wire.available()) Wire.read();  // byte 0: previous channel's result — discard
  uint8_t val = Wire.available() ? Wire.read() : 0;  // byte 1: first conversion on new channel

  // Transaction 2: same channel, fully settled value
  Wire.requestFrom((uint8_t)PCF8591_ADDR, (uint8_t)2);
  if (Wire.available()) Wire.read();  // discard
  val = Wire.available() ? Wire.read() : val;  // stable result

  return val;
}

// ================================================================
// THERMISTOR — ADC TO TEMPERATURE (Steinhart-Hart / B-parameter)
//
// Verified HW-011 circuit: VCC -> R_fixed -> ADC pin -> NTC -> GND
//   Warm = NTC drops = ADC voltage drops = adc value drops
//   R_ntc = R_fixed * adc / (255 - adc)
// ================================================================
static float readThermistor() {
  uint8_t adc = readPCF8591Stable(CH_THERM);

  if (adc <= 1 || adc >= 254) return -99.0f;

  float rNtc = R_FIXED * (float)adc / (255.0f - (float)adc);

  // B-parameter Steinhart equation
  float steinhart = logf(rNtc / R_NOMINAL);
  steinhart /= B_COEFF;
  steinhart += 1.0f / (T_NOMINAL + 273.15f);
  steinhart = 1.0f / steinhart;
  steinhart -= 273.15f;

  if (steinhart < -40.0f || steinhart > 125.0f) return -99.0f;

  return steinhart;
}

// ================================================================
// I2C BUS PROBE — check if device ACKs
// ================================================================
static bool i2cProbe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

static bool readINA219Register16(uint8_t reg, uint16_t& out) {
  Wire.beginTransmission(0x40);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;

  Wire.requestFrom((uint8_t)0x40, (uint8_t)2);
  if (Wire.available() < 2) return false;

  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  out = ((uint16_t)msb << 8) | (uint16_t)lsb;
  return true;
}

// ================================================================
// WIFI CONNECT (with TX power fallback)
// ================================================================
static bool connectWiFiWithTxPower(wifi_power_t txPower, uint8_t maxTries = 30) {
  WiFi.disconnect(true, true);
  delay(120);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.setTxPower(txPower);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.printf("[WiFi] Connecting to %s (tx=%d)", WIFI_SSID, (int)txPower);
  uint8_t tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < maxTries) {
    delay(500);
    Serial.print(".");
    tries++;
  }
  Serial.println();

  return WiFi.status() == WL_CONNECTED;
}

// ================================================================
// READ ALL SENSORS
// ================================================================
static void readSensors() {
  // --- Time source priority: NTP/system -> RTC -> uptime fallback ---
  if (systemTimeValid()) {
    struct tm ti;
    if (getLocalTime(&ti, 50)) {
      snprintf(gTimestamp, sizeof(gTimestamp), "%04d-%02d-%02d %02d:%02d:%02d",
               ti.tm_year + 1900, ti.tm_mon + 1, ti.tm_mday,
               ti.tm_hour, ti.tm_min, ti.tm_sec);
    }
  } else if (hasRTC) {
    DateTime now = rtc.now();
    snprintf(gTimestamp, sizeof(gTimestamp), "%04d-%02d-%02d %02d:%02d:%02d",
             now.year(), now.month(), now.day(),
             now.hour(), now.minute(), now.second());
  } else {
    // Fallback timestamp from uptime when RTC is missing.
    uint32_t s = millis() / 1000;
    uint32_t hh = (s / 3600) % 24;
    uint32_t mm = (s / 60) % 60;
    uint32_t ss = s % 60;
    snprintf(gTimestamp, sizeof(gTimestamp), "1970-01-01 %02lu:%02lu:%02lu",
             (unsigned long)hh, (unsigned long)mm, (unsigned long)ss);
  }
  gUptime = millis() / 1000;

  // --- PCF8591: temperature ---
  if (hasPCF) {
    gThermRaw = readPCF8591Stable(CH_THERM);
    gLightRaw = readPCF8591Stable(CH_LDR);
    gExtRaw   = readPCF8591Stable(CH_EXT);
    gPotRaw   = readPCF8591Stable(CH_POT);
  }

  if (hasPCF && moduleTemp) {
    gTemp = readThermistor();
  }

  // --- PCF8591: light ---
  if (hasPCF && moduleLight) {
    gLightPct = ((255 - gLightRaw) / 255.0f) * 100.0f;  // lower ADC = more light
  }

  // --- INA219 ---
  if (hasINA219) {
    uint16_t raw = 0;
    if (readINA219Register16(0x02, raw)) gInaBusRaw = raw;
    if (readINA219Register16(0x04, raw)) gInaCurrRaw = (int16_t)raw;
    if (readINA219Register16(0x03, raw)) gInaPowRaw = raw;
  }

  if (hasINA219 && moduleCurrent) {
    gVoltage   = ina.getBusVoltage_V();
    gCurrentMA = ina.getCurrent_mA();
    gPowerMW   = ina.getPower_mW();

    // Guard against garbage reads
    if (isnan(gVoltage)   || isinf(gVoltage))   gVoltage   = 0.0f;
    if (isnan(gCurrentMA) || isinf(gCurrentMA))  gCurrentMA = 0.0f;
    if (isnan(gPowerMW)   || isinf(gPowerMW))    gPowerMW   = 0.0f;
  }

  // --- Energy accumulation (coulomb counting) ---
  uint32_t now_ms = millis();
  uint32_t dt_ms = now_ms - lastEnergy;
  float dt_hours = dt_ms / 3600000.0f;
  float deltaMAh = gCurrentMA * dt_hours;  // positive = discharge, negative = charging
  gTotalMAh = constrain(gTotalMAh + deltaMAh, 0.0f, BATTERY_CAPACITY);
  lastEnergy = now_ms;
  persistBatteryStateIfDue(now_ms);

  // --- Battery SOC estimation (hybrid: coulomb + OCV anchor) ---
  gBatteryPresent = gVoltage > 2.5f;

  float socFromCount = 100.0f - (gTotalMAh / BATTERY_CAPACITY) * 100.0f;
  socFromCount = constrain(socFromCount, 0.0f, 100.0f);

  bool nearRest = fabsf(gCurrentMA) <= BATT_REST_CURRENT_MA;
  if (nearRest) gRestAccumMs += dt_ms;
  else          gRestAccumMs = 0;

  if (nearRest && gRestAccumMs >= BATT_REST_TIME_MS && gVoltage > 2.5f) {
    // At near-rest, voltage is more trustworthy: slowly pull SOC to OCV estimate.
    float socFromOcv = ocvToSocPct(gVoltage);
    gSocEstimatePct = 0.75f * socFromCount + 0.25f * socFromOcv;
    gTotalMAh = BATTERY_CAPACITY * (1.0f - gSocEstimatePct / 100.0f);
  } else {
    gSocEstimatePct = socFromCount;
  }

  gSocEstimatePct = constrain(gSocEstimatePct, 0.0f, 100.0f);

  // 1% stepped display behavior with anti-jitter.
  int targetPct = (int)roundf(gSocEstimatePct);
  if (gCurrentMA >= 5.0f) {
    if (targetPct < gBattPctShown) gBattPctShown = targetPct;
  } else if (gCurrentMA <= -20.0f) {
    if (targetPct > gBattPctShown) gBattPctShown = targetPct;
  } else {
    gBattPctShown = targetPct;
  }
  gBattPctShown = constrain(gBattPctShown, 0, 100);
  gBattPct = (float)gBattPctShown;

  float remaining = BATTERY_CAPACITY * (gSocEstimatePct / 100.0f);
  gBattLife = (gBatteryPresent && fabsf(gCurrentMA) > 0.1f)
    ? (remaining / fabsf(gCurrentMA)) * 60.0f
    : -1.0f;

  if (!gBatteryPresent) {
    gCurrentMA = 0.0f;
    gPowerMW = 0.0f;
    gSocEstimatePct = 0.0f;
    gBattPctShown = 0;
    gBattPct = 0.0f;
    gBattLife = -1.0f;
    gTotalMAh = 0.0f;
  }

  // --- WiFi ---
  gRSSI = WiFi.RSSI();

  // --- CPU load ---
  gCpuLoad = measureCpuLoadPct();
}

// ================================================================
// BUILD JSON — compact, only real data
// ================================================================
static void buildJSON(char* buf, size_t bufSize) {
  JsonDocument doc;

  doc["ts"]   = gTimestamp;
  doc["up"]   = gUptime;
  doc["temp"]   = roundf(gTemp * 10.0f) / 10.0f;
  doc["ldr"]    = gLightRaw;
  doc["ldr_pct"] = roundf(gLightPct * 10.0f) / 10.0f;
  doc["rssi"]   = gRSSI;
  doc["cpu"]    = (int)roundf(gCpuLoad);
  doc["v"]      = roundf(gVoltage * 100.0f) / 100.0f;
  doc["ma"]     = roundf(gCurrentMA * 10.0f) / 10.0f;
  doc["mw"]     = (int)roundf(gPowerMW);
  doc["mah"]    = roundf((gBatteryPresent ? gTotalMAh : 0.0f) * 100.0f) / 100.0f;
  doc["batt"]   = roundf(gBattPct * 10.0f) / 10.0f;
  doc["batt_min"] = (gBatteryPresent && gBattLife > 0) ? (int)gBattLife : -1;
  doc["ssid"]   = WiFi.SSID();
  doc["ip"]     = WiFi.localIP().toString();
  doc["mac"]    = WiFi.macAddress();
  doc["channel"] = (int)WiFi.channel();

  serializeJson(doc, buf, bufSize);
}

// ================================================================
// BUILD RAW JSON — exact GPIO + raw sensor values
// ================================================================
static void buildRawGpioJSON(char* buf, size_t bufSize) {
  JsonDocument doc;
  doc["ts"] = gTimestamp;
  doc["up"] = gUptime;

  JsonObject gpio = doc["gpio"].to<JsonObject>();
  for (size_t i = 0; i < GPIO_PUBLISH_PIN_COUNT; i++) {
    const uint8_t pin = GPIO_PUBLISH_PINS[i];
    char key[10];
    snprintf(key, sizeof(key), "gpio%u", pin);
    gpio[key] = digitalRead(pin);
  }

  JsonObject pcfRaw = doc["pcf8591_raw"].to<JsonObject>();
  pcfRaw["ch0_ldr"] = gLightRaw;
  pcfRaw["ch1_therm"] = gThermRaw;
  pcfRaw["ch2_ext"] = gExtRaw;
  pcfRaw["ch3_pot"] = gPotRaw;

  JsonObject inaRaw = doc["ina219_raw"].to<JsonObject>();
  inaRaw["bus"] = gInaBusRaw;
  inaRaw["current"] = gInaCurrRaw;
  inaRaw["power"] = gInaPowRaw;

  serializeJson(doc, buf, bufSize);
}

// ================================================================
// MQTT CONNECT
// ================================================================
static void connectMqtt() {
  if (WiFi.status() != WL_CONNECTED) return;

  Serial.printf("[MQTT] Connecting to %s...\n", MQTT_BROKER);
  if (mqtt.connect(mqttClientId.c_str(), MQTT_USER, MQTT_PASS_MQTT)) {
    Serial.println("[MQTT] Connected");
    mqtt.subscribe(MQTT_CMD_TOPIC);
    mqtt.publish(MQTT_TOPIC, "{\"status\":\"online\"}");
    publishState();
  } else {
    Serial.printf("[MQTT] Failed rc=%d\n", mqtt.state());
  }
}

// ================================================================
// PUBLISH MODULE STATE
// ================================================================
static void publishState() {
  char buf[128];
  JsonDocument doc;
  JsonObject m = doc["modules"].to<JsonObject>();
  m["temperature"] = moduleTemp;
  m["light"]       = moduleLight;
  m["cpu"]         = moduleCPU;
  m["current"]     = moduleCurrent;
  m["cpu_stress"]  = moduleCpuStress;
  serializeJson(doc, buf, sizeof(buf));
  mqtt.publish(MQTT_STATE_TOPIC, buf);
}

// ================================================================
// MQTT MESSAGE HANDLER
// ================================================================
static void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  // Stack-allocate: no heap fragmentation
  char msg[128];
  size_t len = min((unsigned int)(sizeof(msg) - 1), length);
  memcpy(msg, payload, len);
  msg[len] = '\0';

  Serial.printf("[MQTT] Rx: %s\n", msg);

  JsonDocument doc;
  if (deserializeJson(doc, msg) != DeserializationError::Ok) return;

  const char* mod = doc["module"];
  if (!mod) return;

  bool en = doc["enabled"] | true;

  if      (strcmp(mod, "temperature") == 0)  moduleTemp    = en;
  else if (strcmp(mod, "light") == 0)        moduleLight   = en;
  else if (strcmp(mod, "current") == 0)      moduleCurrent = en;
  else if (strcmp(mod, "cpu_stress") == 0)   moduleCpuStress = en;

  Serial.printf("[MQTT] %s -> %s\n", mod, en ? "ON" : "OFF");
  publishState();
}

// ================================================================
// BUTTON HANDLING (ISR-driven, debounced)
// ================================================================
static void handleButtons() {
  uint32_t now = millis();

  static uint32_t lastBtnN = 0;
  static uint32_t lastBtnP = 0;

  if (isrNextFlag) {
    isrNextFlag = false;
    if (now - lastBtnN > DEBOUNCE_MS) {
      lastBtnN = now;
      page = (page + 1) % NUM_PAGES;
      drawPage();
    }
  }

  if (isrPrevFlag) {
    isrPrevFlag = false;
    if (now - lastBtnP > DEBOUNCE_MS) {
      lastBtnP = now;
      page = (page - 1 + NUM_PAGES) % NUM_PAGES;
      drawPage();
    }
  }
}

// ================================================================
// OLED UTILITIES
// ================================================================
static void drawTitleBar(const char* title) {
  oled.fillRect(0, 0, 128, 12, SSD1306_WHITE);
  oled.setTextColor(SSD1306_BLACK);
  oled.setTextSize(1);
  int x = (128 - (int)strlen(title) * 6) / 2;
  if (x < 1) x = 1;
  oled.setCursor(x, 2);
  oled.print(title);
  oled.setTextColor(SSD1306_WHITE);
}

static void drawProgressBar(int x, int y, int w, int h, float pct) {
  pct = constrain(pct, 0.0f, 100.0f);
  oled.drawRoundRect(x, y, w, h, 3, SSD1306_WHITE);
  int fw = (int)(pct / 100.0f * (w - 4));
  if (fw > 0) oled.fillRoundRect(x + 2, y + 2, fw, h - 4, 2, SSD1306_WHITE);
}

static void drawCentered(const char* text, int y, int sz) {
  oled.setTextSize(sz);
  int x = (128 - (int)strlen(text) * 6 * sz) / 2;
  if (x < 0) x = 0;
  oled.setCursor(x, y);
  oled.print(text);
}

static void printDegC() {
  oled.write(248);  // Degree symbol in cp437 for Adafruit GFX default font
  oled.print("C");
}

// ================================================================
// OLED PAGES
// ================================================================
static void pageDashboard() {
  oled.fillRect(0, 0, 128, 12, SSD1306_WHITE);
  oled.setTextColor(SSD1306_BLACK);
  oled.setTextSize(1);
  oled.setCursor(4, 2);
  oled.print("MONITOR");
  if (strlen(gTimestamp) >= 19) {
    oled.setCursor(76, 2);
    char hhmmss[9];
    memcpy(hhmmss, gTimestamp + 11, 8);
    hhmmss[8] = '\0';
    oled.print(hhmmss);
  }
  oled.setTextColor(SSD1306_WHITE);

  oled.setTextSize(1);
  oled.setCursor(0, 15);  oled.printf("T:%.1f", gTemp); printDegC();
  oled.setCursor(68, 15); oled.printf("L:%.0f%%", gLightPct);
  oled.setCursor(0, 25);  oled.printf("P:%.0fmW", gPowerMW);
  oled.setCursor(68, 25); oled.printf("C:%.0fmA", gCurrentMA);
  oled.setCursor(0, 35);  oled.printf("R:%ddBm", gRSSI);
  oled.setCursor(68, 35); oled.printf("CPU:%d%%", (int)roundf(gCpuLoad));

  drawProgressBar(0, 47, 104, 10, gBattPct);
  oled.setCursor(108, 49);
  oled.printf("%.0f%%", gBattPct);
}

static void pageTemp() {
  drawTitleBar("TEMPERATURA");
  char buf[10];
  snprintf(buf, sizeof(buf), "%.1f", gTemp);
  int numW = strlen(buf) * 18;
  int sx = (128 - numW - 14) / 2;
  if (sx < 0) sx = 0;
  oled.setTextSize(3); oled.setCursor(sx, 18); oled.print(buf);
  int cx = sx + numW + 6;
  int cy = 22;
  oled.setTextSize(2); oled.setCursor(cx, cy); oled.print("C");
  oled.drawCircle(cx - 4, cy + 1, 1, SSD1306_WHITE);
  float pct = constrain((gTemp + 10.0f) / 60.0f * 100.0f, 0, 100);
  drawProgressBar(4, 48, 120, 8, pct);
}

static void pageLight() {
  drawTitleBar("LUMINA");
  char buf[10];
  snprintf(buf, sizeof(buf), "%.0f", gLightPct);
  int numW = strlen(buf) * 18;
  int sx = (128 - numW - 14) / 2;
  if (sx < 0) sx = 0;
  oled.setTextSize(3); oled.setCursor(sx, 16); oled.print(buf);
  oled.setTextSize(2); oled.setCursor(sx + numW + 2, 20); oled.print("%");
  oled.setTextSize(1);
  char raw[16];
  snprintf(raw, sizeof(raw), "Brut: %d / 255", gLightRaw);
  drawCentered(raw, 42, 1);
  drawProgressBar(4, 52, 120, 8, gLightPct);
}

static void pageWiFi() {
  drawTitleBar("RETEA + MQTT");
  oled.setTextSize(1);
  oled.setCursor(0, 15); oled.printf("SSID: %s", WiFi.SSID().c_str());
  oled.setCursor(0, 25); oled.printf("IP:   %s", WiFi.localIP().toString().c_str());
  oled.setCursor(0, 35); oled.printf("RSSI: %d dBm", gRSSI);
  oled.setCursor(0, 45); oled.printf("Data: %.10s", strlen(gTimestamp) >= 10 ? gTimestamp : "----------");
  oled.setCursor(0, 55); oled.printf("MQTT:%s", mqtt.connected() ? "OK" : "Off");

  int bars = 0;
  if (WiFi.status() == WL_CONNECTED) {
    bars = constrain((gRSSI + 100) / 10, 0, 4);
  }
  for (int i = 0; i < 4; i++) {
    int bh = 4 + i * 3;
    int by = 54 - bh;
    int bx = 100 + i * 7;
    if (i < bars) oled.fillRect(bx, by, 5, bh, SSD1306_WHITE);
    else          oled.drawRect(bx, by, 5, bh, SSD1306_WHITE);
  }
}

static void pageCPU() {
  drawTitleBar("INC. CPU");
  char buf[10];
  snprintf(buf, sizeof(buf), "%d", (int)roundf(gCpuLoad));
  int numW = strlen(buf) * 18;
  int sx = (128 - numW - 14) / 2;
  if (sx < 0) sx = 0;
  oled.setTextSize(3); oled.setCursor(sx, 18); oled.print(buf);
  oled.setTextSize(2); oled.setCursor(sx + numW + 2, 22); oled.print("%");
  drawProgressBar(4, 48, 120, 8, gCpuLoad);
}

static void pageVoltage() {
  drawTitleBar("TENSIUNE");
  char buf[10];
  snprintf(buf, sizeof(buf), "%.2f", gVoltage);
  int numW = strlen(buf) * 18;
  int sx = (128 - numW - 14) / 2;
  if (sx < 0) sx = 0;
  oled.setTextSize(3); oled.setCursor(sx, 16); oled.print(buf);
  oled.setTextSize(2); oled.setCursor(sx + numW + 2, 20); oled.print("V");
  char pow[20];
  snprintf(pow, sizeof(pow), "Put: %.0f mW", gPowerMW);
  drawCentered(pow, 42, 1);
  float pct = constrain((gVoltage - 3.0f) / 1.2f * 100.0f, 0, 100);
  drawProgressBar(4, 52, 120, 8, pct);
}

static void pageCurrent() {
  drawTitleBar("CURENT");
  char buf[16];
  snprintf(buf, sizeof(buf), "%.0f mA", gCurrentMA);
  drawCentered(buf, 18, 2);
  char total[22];
  snprintf(total, sizeof(total), "CP: %.2f mAh", gBatteryPresent ? gTotalMAh : 0.0f);
  drawCentered(total, 38, 1);
  char pow[20];
  snprintf(pow, sizeof(pow), "Put: %.0f mW", gPowerMW);
  drawCentered(pow, 48, 1);
}

static void pageBattery() {
  drawTitleBar("BATERIE");
  char buf[10];
  snprintf(buf, sizeof(buf), "%.0f", gBattPct);
  int numW = strlen(buf) * 18;
  int sx = (128 - numW - 14) / 2;
  if (sx < 0) sx = 0;
  oled.setTextSize(3); oled.setCursor(sx, 16); oled.print(buf);
  oled.setTextSize(2); oled.setCursor(sx + numW + 2, 20); oled.print("%");
  oled.setTextSize(1);
  if (gBatteryPresent && gBattLife > 0) {
    int hrs = (int)gBattLife / 60;
    int mins = (int)gBattLife % 60;
    char est[20];
    snprintf(est, sizeof(est), "Est: %dh %dm", hrs, mins);
    drawCentered(est, 40, 1);
  } else if (!gBatteryPresent) {
    drawCentered("Est: -- (fara bat)", 40, 1);
  } else {
    drawCentered("Est: -- (fara sarc)", 40, 1);
  }
  char used[24];
  snprintf(used, sizeof(used), "CP: %.2f mAh", gBatteryPresent ? gTotalMAh : 0.0f);
  drawCentered(used, 50, 1);
}

// ================================================================
// OLED PAGE ROUTER
// ================================================================
static void drawPage() {
  if (!hasOLED) return;

  oled.clearDisplay();
  switch (page) {
    case 0: pageDashboard(); break;
    case 1: pageTemp();      break;
    case 2: pageLight();     break;
    case 3: pageWiFi();      break;
    case 4: pageCPU();       break;
    case 5: pageVoltage();   break;
    case 6: pageCurrent();   break;
    case 7: pageBattery();   break;
  }
  oled.setTextSize(1);
  oled.setCursor(100, 57);
  oled.printf("%d/%d", page + 1, NUM_PAGES);
  oled.display();
}

// ================================================================
// SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== Hard&Soft Task 1 — MQTT Monitor ===\n");

  // --- I2C at 100kHz (PCF8591 max is 100kHz, DS1307 too) ---
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_FREQ);
  Wire.setTimeOut(50);

  // --- CPU load hook (fallback when runtime stats are unavailable) ---
  bool idleHookOk = false;
#if CONFIG_FREERTOS_NUMBER_OF_CORES == 1
  idleHookOk = esp_register_freertos_idle_hook(cpuIdleHook);
#else
  idleHookOk = esp_register_freertos_idle_hook_for_cpu(cpuIdleHook, 0);
#endif

  if (!idleHookOk) {
    Serial.println("[CPU] Idle hook not available, using runtime stats/0%");
  }
  cpuLastIdleHookCount = cpuIdleHookCount;

  if (xTaskCreate(cpuStressTask, "cpu_stress", 3072, nullptr, 1, nullptr) != pdPASS) {
    Serial.println("[CPU] Stress task create FAILED");
  }

  // --- Buttons (interrupt-driven) ---
  pinMode(BTN_NEXT, INPUT_PULLUP);
  pinMode(BTN_PREV, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTN_NEXT), isrBtnNext, FALLING);
  attachInterrupt(digitalPinToInterrupt(BTN_PREV), isrBtnPrev, FALLING);

  // --- I2C scan ---
  Serial.println("[I2C] Scan (100kHz):");
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  0x%02X", addr);
      if (addr == 0x3C) Serial.print(" SSD1306");
      if (addr == 0x40) Serial.print(" INA219");
      if (addr == 0x48) Serial.print(" PCF8591");
      if (addr == 0x50) Serial.print(" AT24C32");
      if (addr == 0x68) Serial.print(" DS1307");
      Serial.println();
    }
  }
  Serial.println();

  // --- OLED ---
  hasOLED = i2cProbe(0x3C);
  if (hasOLED) {
    hasOLED = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    if (hasOLED) {
      oled.clearDisplay();
      oled.setTextColor(SSD1306_WHITE);
      oled.setTextSize(1);
      oled.setCursor(0, 0);
      oled.println("Booting...");
      oled.display();
    }
  }
  Serial.printf("[OLED]   %s\n", hasOLED ? "OK" : "MISSING");

  // --- INA219 (16V, 400mA range — better resolution for battery monitoring) ---
  hasINA219 = i2cProbe(0x40);
  if (hasINA219) {
    hasINA219 = ina.begin();
    if (hasINA219) {
      ina.setCalibration_16V_400mA();  // <-- KEY FIX: proper calibration
    }
  }
  Serial.printf("[INA219] %s\n", hasINA219 ? "OK (16V/400mA)" : "MISSING");

  // --- RTC ---
  hasRTC = i2cProbe(0x68);
  if (hasRTC) {
    hasRTC = rtc.begin();
    if (hasRTC && !rtc.isrunning()) {
      Serial.println("[RTC]    Setting to compile time");
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
  }
  Serial.printf("[RTC]    %s\n", hasRTC ? "OK" : "MISSING");

  // --- PCF8591 ---
  hasPCF = i2cProbe(PCF8591_ADDR);
  Serial.printf("[PCF]    %s\n", hasPCF ? "OK" : "MISSING");

  if (hasPCF) {
    Serial.println("[PCF] Channel diagnostic (stable reads):");
    for (int ch = 0; ch < 4; ch++) {
      uint8_t val = readPCF8591Stable(ch);
      Serial.printf("  AIN%d = %3d", ch, val);
      if (ch == CH_POT)   Serial.print("  (pot)");
      if (ch == CH_LDR)   Serial.print("  (LDR)");
      if (ch == CH_THERM) Serial.print("  (therm)");
      if (ch == CH_EXT)   Serial.print("  (ext)");
      Serial.println();
    }
    Serial.println("  Cover LDR or warm thermistor to verify channels\n");
  }

  // --- WiFi ---
  if (hasOLED) {
    oled.clearDisplay();
    oled.setCursor(0, 0);
    oled.println("WiFi...");
    oled.println(WIFI_SSID);
    oled.display();
  }

  const wifi_power_t txLevels[] = {
    WIFI_POWER_8_5dBm,
    WIFI_POWER_13dBm,
    WIFI_POWER_19_5dBm
  };

  bool wifiOk = false;
  for (size_t i = 0; i < (sizeof(txLevels) / sizeof(txLevels[0])); i++) {
    if (connectWiFiWithTxPower(txLevels[i])) {
      wifiOk = true;
      break;
    }
    Serial.println("[WiFi] Retry with different TX power...");
  }

  if (wifiOk) {
    syncClockFromNtp();
    Serial.printf("[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
    if (hasOLED) {
      oled.clearDisplay();
      oled.setCursor(0, 0);
      oled.println("WiFi OK");
      oled.println(WiFi.localIP().toString());
      oled.println("\nMQTT: broker.emqx.io");
      oled.display();
    }
  } else {
    Serial.println("[WiFi] FAILED — offline mode");
  }

  // --- MQTT ---
  mqttClientId = "esp32-hs-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setCallback(onMqttMessage);
  mqtt.setBufferSize(1024);
  connectMqtt();

  loadBatteryState();

  lastSensorRead = millis();
  lastEnergy = millis();
  gLastBattPersistMs = millis();

  delay(1500);
  cpuLastIdleHookCount = cpuIdleHookCount;
  Serial.println("\n[READY] Loop starting\n");
}

// ================================================================
// LOOP — minimal: only react to flags, no polling
// ================================================================
void loop() {
  // MQTT keepalive (PubSubClient needs frequent calls)
  mqtt.loop();

  // MQTT reconnect (non-blocking, every 5s)
  static uint32_t lastMqttRetry = 0;
  if (!mqtt.connected() && millis() - lastMqttRetry > 5000) {
    lastMqttRetry = millis();
    connectMqtt();
  }

  // Buttons (ISR-driven, just check flags)
  handleButtons();

  // Sensor read + publish every 1 second
  if (millis() - lastSensorRead >= 1000) {
    lastSensorRead = millis();

    readSensors();

    // MQTT publish
    if (mqtt.connected()) {
      char json[384];
      buildJSON(json, sizeof(json));
      mqtt.publish(MQTT_TOPIC, json);

      char rawJson[768];
      buildRawGpioJSON(rawJson, sizeof(rawJson));
      mqtt.publish(MQTT_RAW_TOPIC, rawJson);
    }

    drawPage();

    // Serial log
    Serial.printf("[%s] T=%.1f L=%d V=%.2f I=%.1f CPU=%.1f%%\n",
      gTimestamp, gTemp, gLightRaw, gVoltage, gCurrentMA, gCpuLoad);
  }

  delay(10);
}