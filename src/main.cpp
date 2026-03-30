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
 *   - 10k NTC thermistor divider on GPIO0 (ADC)
 *   - LSM6DS3-compatible gyro      (I2C 0x6A)
 *   - LG INR18650MH1 battery      (3.6V nom, 3200mAh)
 *   - 2x tactile push buttons
 *
 * WIRING:
 *   I2C Bus:
 *     ESP32 GPIO8 (SDA) --> OLED, INA219, DS1307, LSM6DS3
 *     ESP32 GPIO9 (SCL) --> OLED, INA219, DS1307, LSM6DS3
 *   Thermistor divider:
 *     3.3V -> 10k resistor -> GPIO0 -> 10k NTC -> GND
 *   Buttons (active LOW, internal pull-up):
 *     ESP32 GPIO3 --> Button NEXT --> GND
 *     ESP32 GPIO4 --> Button PREV --> GND
 *
 * FIXES vs original:
 *   - Migrated thermistor to direct ADC read on GPIO0
 *   - Added LSM6DS3 gyroscope readout over I2C
 *   - I2C clock set to 100kHz for DS1307 compatibility
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
#include <stdarg.h>

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
#include <vector>
// ================================================================
// CONFIGURATION
// ================================================================


static const std::vector<const char*> WIFI_SSID = {"stanislav", "Network", "DIGI", "DIGI-75hC"};
static const std::vector<const char*> WIFI_PASS = {"stas1524", "19911313", "27mc2004ca", "9T2Du96euz"};
  

static const float BATTERY_CAPACITY = 3200.0f;  // mAh
static const float BATT_REST_CURRENT_MA = 80.0f;  // ~C/40 for 3200mAh cell
static const uint32_t BATT_REST_TIME_MS = 30000;
static const float BATT_OCV_MISMATCH_RECOVER_PCT = 20.0f;
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
// GPIO18/19 are native USB D-/D+ on ESP32-C3; avoid probing them as GPIO.
static const uint8_t GPIO_PUBLISH_PINS[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 20, 21};
static const size_t GPIO_PUBLISH_PIN_COUNT = sizeof(GPIO_PUBLISH_PINS) / sizeof(GPIO_PUBLISH_PINS[0]);

// ================================================================
// I2C + ANALOG
// ================================================================
static const uint32_t I2C_FREQ      = 100000;  // 100kHz — safe for DS1307
static const uint8_t THERM_PIN      = 0;       // GPIO0: divider midpoint
static const float ADC_MAX          = 4095.0f; // ESP32-C3 12-bit ADC

// ================================================================
// NTC THERMISTOR (10k divider)
// Circuit: 3.3V -> 10k fixed -> ADC(GPIO0) -> 10k NTC -> GND
// ================================================================
static const float R_FIXED   = 10000.0f;
static const float R_NOMINAL = 10000.0f;
static const float T_NOMINAL = 25.0f;
static const float B_COEFF   = 3950.0f;

// LSM6DS3/compatible gyro sensor over I2C.
static const uint8_t GYRO_I2C_ADDR = 0x6A;
static const uint8_t GYRO_WHO_AM_I_REG = 0x0F;
static const uint8_t GYRO_WHO_AM_I_VAL = 0x69;
static const uint8_t GYRO_CTRL2_G = 0x11;
static const uint8_t GYRO_OUTX_L_G = 0x22;
static const float GYRO_SCALE_DPS = 0.07f;  // FS=2000 dps => 70 mdps/LSB

// ================================================================
// DISPLAY & TIMING
// ================================================================
static const uint8_t  SCREEN_W      = 128;
static const uint8_t  SCREEN_H      = 64;
static const uint8_t  NUM_PAGES     = 9;
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
static bool hasGyro   = false;

// ================================================================
// SENSOR DATA
// ================================================================
static float    gTemp       = 0.0f;
static uint16_t gThermRaw   = 0;
static int16_t  gGyroRawX   = 0;
static int16_t  gGyroRawY   = 0;
static int16_t  gGyroRawZ   = 0;
static float    gGyroX      = 0.0f;
static float    gGyroY      = 0.0f;
static float    gGyroZ      = 0.0f;
static float    gGyroAbs    = 0.0f;
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
static bool gHasPersistedBatteryState = false;
static bool gBootstrapFromVoltageDone = false;

// OLED page 9 compact live logs (same sources as MQTT raw payload)
static const uint8_t OLED_LOG_LINES = 5;
static const uint8_t OLED_LOG_LINE_LEN = 21;  // 21 chars fit in 128px at text size 1
static char gOledLog[OLED_LOG_LINES][OLED_LOG_LINE_LEN + 1];
static uint8_t gOledLogHead = 0;
static bool gOledLogHasData = false;
static int8_t gGpioLastState[GPIO_PUBLISH_PIN_COUNT];
static bool gGpioLogPrimed = false;
static uint32_t gGpioLogSeq = 0;
static uint8_t gRawSnapshotPhase = 0;

// Sensor read interval tracking
static uint32_t lastSensorRead = 0;

// Button ISR flags
static volatile bool isrNextFlag = false;
static volatile bool isrPrevFlag = false;

// MQTT client ID
static String mqttClientId;

// Module enable/disable (toggled via MQTT)
static bool moduleTemp    = true;
static bool moduleGyro    = true;
static bool moduleCPU     = true;
static bool moduleCurrent = true;
static volatile bool moduleCpuStress = false;

// Forward declarations for functions used before their definitions.
static void publishState();
static void drawPage();
static float measureCpuLoadPct();
static void buildRawGpioJSON(char* buf, size_t bufSize);
static void printDegC();
static bool initGyro();
static bool readGyro();
static bool cpuIdleHook();
static bool getCpuLoadFromRuntimeStats(float& loadOut);
static float getCpuLoadFromIdleHook();
static void cpuStressTask(void* parm);
static void appendOledLog(const char* fmt, ...);
static void updateOledRawLog();
static const char* gpioShortName(uint8_t pin);

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

static void loadBatteryState() {
  if (!prefs.begin(PREF_NS, false)) return;
  gHasPersistedBatteryState = prefs.isKey(PREF_KEY_USED_MAH);
  if (gHasPersistedBatteryState) {
    gTotalMAh = prefs.getFloat(PREF_KEY_USED_MAH, gTotalMAh);
  }
  prefs.end();
  gTotalMAh = constrain(gTotalMAh, 0.0f, BATTERY_CAPACITY);

  // If no persisted value exists, SOC will be bootstrapped from measured voltage.
  if (!gHasPersistedBatteryState) {
    gBootstrapFromVoltageDone = false;
  }
}

static void persistBatteryStateIfDue(uint32_t nowMs) {
  if (nowMs - gLastBattPersistMs < BATT_SAVE_INTERVAL_MS) return;
  gLastBattPersistMs = nowMs;

  if (!prefs.begin(PREF_NS, false)) return;
  prefs.putFloat(PREF_KEY_USED_MAH, gTotalMAh);
  prefs.end();
}

static void appendOledLog(const char* fmt, ...) {
  char line[OLED_LOG_LINE_LEN + 1];
  va_list args;
  va_start(args, fmt);
  vsnprintf(line, sizeof(line), fmt, args);
  va_end(args);

  line[OLED_LOG_LINE_LEN] = '\0';
  strncpy(gOledLog[gOledLogHead], line, OLED_LOG_LINE_LEN);
  gOledLog[gOledLogHead][OLED_LOG_LINE_LEN] = '\0';

  gOledLogHead = (gOledLogHead + 1) % OLED_LOG_LINES;
  gOledLogHasData = true;
}

static const char* gpioShortName(uint8_t pin) {
  switch (pin) {
    case 3: return "BTN+";
    case 4: return "BTN-";
    case 8: return "SDA";
    case 9: return "SCL";
    default: return "IO";
  }
}

static void updateOledRawLog() {
  gGpioLogSeq++;

  // First sample seeds baseline to avoid startup noise in the change feed.
  if (!gGpioLogPrimed) {
    for (size_t i = 0; i < GPIO_PUBLISH_PIN_COUNT; i++) {
      gGpioLastState[i] = digitalRead(GPIO_PUBLISH_PINS[i]);
    }
    appendOledLog("Logger pornit");
    gGpioLogPrimed = true;
    return;
  }

  int changes = 0;
  for (size_t i = 0; i < GPIO_PUBLISH_PIN_COUNT; i++) {
    const uint8_t pin = GPIO_PUBLISH_PINS[i];
    int8_t nowState = digitalRead(pin);
    if (nowState != gGpioLastState[i]) {
      appendOledLog("#%lu G%02u %s %s", (unsigned long)(gGpioLogSeq % 1000), pin,
                    gpioShortName(pin), nowState ? "ON" : "OFF");
      gGpioLastState[i] = nowState;
      changes++;
      if (changes >= 2) break;  // keep feed readable on a tiny screen
    }
  }

  if (changes > 0) return;

  // No GPIO edge this second: rotate compact raw snapshots (same sources as MQTT raw).
  switch (gRawSnapshotPhase % 3) {
    case 0:
      appendOledLog("TH RAW:%4u T:%4.1f", gThermRaw, gTemp);
      break;
    case 1:
      appendOledLog("GYR X:%4.0f Y:%4.0f", gGyroX, gGyroY);
      break;
    default:
      appendOledLog("GYR Z:%4.0f INA:%5u", gGyroZ, gInaBusRaw);
      break;
  }
  gRawSnapshotPhase++;
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
// THERMISTOR — ADC TO TEMPERATURE (B-parameter)
// ================================================================
static float readThermistor() {
  uint32_t sum = 0;
  constexpr uint8_t samples = 8;
  for (uint8_t i = 0; i < samples; i++) {
    sum += analogRead(THERM_PIN);
    delayMicroseconds(250);
  }

  gThermRaw = (uint16_t)(sum / samples);
  if (gThermRaw <= 1 || gThermRaw >= (uint16_t)(ADC_MAX - 1.0f)) return -99.0f;

  float adc = (float)gThermRaw;
  float rNtc = R_FIXED * adc / (ADC_MAX - adc);

  // B-parameter Steinhart equation
  float steinhart = logf(rNtc / R_NOMINAL);
  steinhart /= B_COEFF;
  steinhart += 1.0f / (T_NOMINAL + 273.15f);
  steinhart = 1.0f / steinhart;
  steinhart -= 273.15f;

  if (steinhart < -40.0f || steinhart > 125.0f) return -99.0f;

  return steinhart;
}

static bool i2cWriteReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool i2cReadRegs(uint8_t addr, uint8_t reg, uint8_t* data, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;

  size_t read = Wire.requestFrom(addr, (uint8_t)len);
  if (read < len) return false;
  for (size_t i = 0; i < len; i++) {
    data[i] = Wire.read();
  }
  return true;
}

static bool initGyro() {
  uint8_t who = 0;
  if (!i2cReadRegs(GYRO_I2C_ADDR, GYRO_WHO_AM_I_REG, &who, 1)) return false;
  if (who != GYRO_WHO_AM_I_VAL) return false;

  // ODR=104Hz, FS=2000dps, BW default.
  if (!i2cWriteReg(GYRO_I2C_ADDR, GYRO_CTRL2_G, 0x4C)) return false;
  delay(10);
  return true;
}

static bool readGyro() {
  uint8_t buf[6] = {0};
  if (!i2cReadRegs(GYRO_I2C_ADDR, (uint8_t)(GYRO_OUTX_L_G | 0x80), buf, sizeof(buf))) {
    return false;
  }

  gGyroRawX = (int16_t)((buf[1] << 8) | buf[0]);
  gGyroRawY = (int16_t)((buf[3] << 8) | buf[2]);
  gGyroRawZ = (int16_t)((buf[5] << 8) | buf[4]);

  gGyroX = gGyroRawX * GYRO_SCALE_DPS;
  gGyroY = gGyroRawY * GYRO_SCALE_DPS;
  gGyroZ = gGyroRawZ * GYRO_SCALE_DPS;
  gGyroAbs = sqrtf(gGyroX * gGyroX + gGyroY * gGyroY + gGyroZ * gGyroZ);
  return true;
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

  if (moduleTemp) {
    gTemp = readThermistor();
  }

  if (hasGyro && moduleGyro) {
    if (!readGyro()) {
      gGyroX = gGyroY = gGyroZ = 0.0f;
      gGyroAbs = 0.0f;
    }
  }

  // --- INA219 ---
  if (hasINA219 && moduleCurrent) {
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
  } else if (!moduleCurrent) {
    // Module OFF: stop current/power processing and battery integration.
    gCurrentMA = 0.0f;
    gPowerMW = 0.0f;
    gBattLife = -1.0f;
    gBatteryPresent = false;
  }

  // If Preferences was reset/missing, seed SOC from OCV instead of defaulting to 100%.
  if (!gHasPersistedBatteryState && !gBootstrapFromVoltageDone && gVoltage > 2.5f) {
    float socFromOcv = ocvToSocPct(gVoltage);
    gSocEstimatePct = constrain(socFromOcv, 0.0f, 100.0f);
    gTotalMAh = BATTERY_CAPACITY * (1.0f - gSocEstimatePct / 100.0f);
    gBattPctShown = (int)roundf(gSocEstimatePct);
    gBootstrapFromVoltageDone = true;
  }

  // --- Energy accumulation (coulomb counting) ---
  uint32_t now_ms = millis();
  uint32_t dt_ms = now_ms - lastEnergy;
  if (moduleCurrent) {
    float dt_hours = dt_ms / 3600000.0f;
    float deltaMAh = gCurrentMA * dt_hours;  // positive = discharge, negative = charging
    gTotalMAh = constrain(gTotalMAh + deltaMAh, 0.0f, BATTERY_CAPACITY);
    persistBatteryStateIfDue(now_ms);
  }
  lastEnergy = now_ms;

  // --- Battery SOC estimation (hybrid: coulomb + adaptive OCV correction) ---
  if (moduleCurrent) {
    gBatteryPresent = gVoltage > 2.5f;
  }

  float socFromCount = 100.0f - (gTotalMAh / BATTERY_CAPACITY) * 100.0f;
  socFromCount = constrain(socFromCount, 0.0f, 100.0f);

  bool nearRest = fabsf(gCurrentMA) <= BATT_REST_CURRENT_MA;
  if (nearRest) gRestAccumMs += dt_ms;
  else          gRestAccumMs = 0;

  if (moduleCurrent && gBatteryPresent) {
    float socFromOcv = ocvToSocPct(gVoltage);
    float mismatchPct = fabsf(socFromCount - socFromOcv);
    bool longRest = nearRest && gRestAccumMs >= BATT_REST_TIME_MS;

    // Base OCV trust by operating mode.
    float ocvWeight = 0.15f;  // under load: small correction only
    if (nearRest) ocvWeight = 0.35f;
    if (longRest) ocvWeight = 0.55f;

    // Recover quickly if persisted/current counter state disagrees strongly with voltage.
    if (mismatchPct >= BATT_OCV_MISMATCH_RECOVER_PCT) {
      ocvWeight = max(ocvWeight, longRest ? 0.75f : 0.50f);
    }

    gSocEstimatePct = (1.0f - ocvWeight) * socFromCount + ocvWeight * socFromOcv;

    // Keep coulomb counter aligned with corrected SOC to avoid snapping back.
    gTotalMAh = BATTERY_CAPACITY * (1.0f - gSocEstimatePct / 100.0f);
    gTotalMAh = constrain(gTotalMAh, 0.0f, BATTERY_CAPACITY);
  } else if (moduleCurrent) {
    gSocEstimatePct = 0.0f;
  }

  gSocEstimatePct = constrain(gSocEstimatePct, 0.0f, 100.0f);

  // 1% stepped display behavior with anti-jitter.
  int targetPct = (int)roundf(gSocEstimatePct);
  if (moduleCurrent && gCurrentMA >= 5.0f) {
    if (targetPct < gBattPctShown) gBattPctShown = targetPct;
  } else if (moduleCurrent && gCurrentMA <= -20.0f) {
    if (targetPct > gBattPctShown) gBattPctShown = targetPct;
  } else {
    gBattPctShown = targetPct;
  }
  gBattPctShown = constrain(gBattPctShown, 0, 100);
  gBattPct = (float)gBattPctShown;

  float remaining = BATTERY_CAPACITY * (gSocEstimatePct / 100.0f);
  gBattLife = (moduleCurrent && gBatteryPresent && fabsf(gCurrentMA) > 0.1f)
    ? (remaining / fabsf(gCurrentMA)) * 60.0f
    : -1.0f;

  if (moduleCurrent && !gBatteryPresent) {
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

  // --- OLED log feed uses same raw sources as MQTT raw payload ---
  updateOledRawLog();
}

// ================================================================
// BUILD JSON — compact, only real data
// ================================================================
static void buildJSON(char* buf, size_t bufSize) {
  JsonDocument doc;

  doc["ts"]   = gTimestamp;
  doc["up"]   = gUptime;
  if (moduleTemp) doc["temp"] = roundf(gTemp * 10.0f) / 10.0f;
  else            doc["temp"] = nullptr;
  doc["temp_raw"] = moduleTemp ? gThermRaw : 0;

  if (moduleGyro && hasGyro) {
    doc["gx"] = roundf(gGyroX * 10.0f) / 10.0f;
    doc["gy"] = roundf(gGyroY * 10.0f) / 10.0f;
    doc["gz"] = roundf(gGyroZ * 10.0f) / 10.0f;
    doc["gabs"] = roundf(gGyroAbs * 10.0f) / 10.0f;
  } else {
    doc["gx"] = nullptr;
    doc["gy"] = nullptr;
    doc["gz"] = nullptr;
    doc["gabs"] = nullptr;
  }

  doc["rssi"]   = gRSSI;
  doc["cpu"]    = (int)roundf(gCpuLoad);

  if (moduleCurrent) doc["v"] = roundf(gVoltage * 100.0f) / 100.0f;
  else               doc["v"] = nullptr;

  if (moduleCurrent) doc["ma"] = roundf(gCurrentMA * 10.0f) / 10.0f;
  else               doc["ma"] = nullptr;

  if (moduleCurrent) doc["mw"] = (int)roundf(gPowerMW);
  else               doc["mw"] = nullptr;

  if (moduleCurrent) doc["mah"] = roundf((gBatteryPresent ? gTotalMAh : 0.0f) * 100.0f) / 100.0f;
  else               doc["mah"] = nullptr;

  if (moduleCurrent) doc["batt"] = roundf(gBattPct * 10.0f) / 10.0f;
  else               doc["batt"] = nullptr;

  doc["batt_min"] = (moduleCurrent && gBatteryPresent && gBattLife > 0) ? (int)gBattLife : -1;
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

  doc["therm_raw"] = moduleTemp ? gThermRaw : 0;

  JsonObject gyroRaw = doc["gyro_raw"].to<JsonObject>();
  if (moduleGyro && hasGyro) {
    gyroRaw["x"] = gGyroRawX;
    gyroRaw["y"] = gGyroRawY;
    gyroRaw["z"] = gGyroRawZ;
  } else {
    gyroRaw["x"] = nullptr;
    gyroRaw["y"] = nullptr;
    gyroRaw["z"] = nullptr;
  }

  JsonObject inaRaw = doc["ina219_raw"].to<JsonObject>();
  if (moduleCurrent) inaRaw["bus"] = gInaBusRaw;
  else               inaRaw["bus"] = nullptr;
  if (moduleCurrent) inaRaw["current"] = gInaCurrRaw;
  else               inaRaw["current"] = nullptr;
  if (moduleCurrent) inaRaw["power"] = gInaPowRaw;
  else               inaRaw["power"] = nullptr;

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
  m["gyro"]        = moduleGyro;
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
  (void)topic;

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
  else if (strcmp(mod, "gyro") == 0)         moduleGyro    = en;
  else if (strcmp(mod, "cpu") == 0)          moduleCPU     = en;
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
  oled.setCursor(68, 15); oled.printf("G:%.0f", gGyroAbs);
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

static void pageGyro() {
  drawTitleBar("GIROSCOP");
  oled.setTextSize(1);
  oled.setCursor(0, 16); oled.printf("X:%7.1f dps", gGyroX);
  oled.setCursor(0, 26); oled.printf("Y:%7.1f dps", gGyroY);
  oled.setCursor(0, 36); oled.printf("Z:%7.1f dps", gGyroZ);
  oled.setCursor(0, 46); oled.printf("ABS:%5.1f dps", gGyroAbs);
  float gyroPct = constrain(gGyroAbs / 10.0f, 0.0f, 100.0f);
  drawProgressBar(4, 54, 120, 8, gyroPct);
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

static void pageRawLog() {
  bool summaryView = ((millis() / 5000UL) % 2UL) == 1UL;

  if (summaryView) drawTitleBar("RAW SUMMARY");
  else             drawTitleBar("RAW EVENTS");

  oled.setTextSize(1);

  if (summaryView) {
    int gpioHigh = 0;
    for (size_t i = 0; i < GPIO_PUBLISH_PIN_COUNT; i++) {
      int state = digitalRead(GPIO_PUBLISH_PINS[i]);
      if (state) gpioHigh++;
    }

    oled.setCursor(0, 14); oled.printf("THM RAW:%4u", gThermRaw);
    oled.setCursor(0, 24); oled.printf("GYR X:%5d", gGyroRawX);
    oled.setCursor(0, 34); oled.printf("GYR Y:%5d", gGyroRawY);
    oled.setCursor(0, 44); oled.printf("GYR Z:%5d", gGyroRawZ);
    oled.setCursor(0, 54); oled.printf("GPIO HIGH:%2d/%2d", gpioHigh, (int)GPIO_PUBLISH_PIN_COUNT);
    return;
  }

  if (!gOledLogHasData) {
    drawCentered("Astept evenimente...", 28, 1);
    return;
  }

  for (uint8_t row = 0; row < OLED_LOG_LINES; row++) {
    uint8_t idx = (gOledLogHead + row) % OLED_LOG_LINES;
    oled.setCursor(0, 14 + row * 10);
    if (row == OLED_LOG_LINES - 1) oled.print('>');
    else oled.print(' ');
    oled.print(gOledLog[idx]);
  }
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
    case 2: pageGyro();      break;
    case 3: pageWiFi();      break;
    case 4: pageCPU();       break;
    case 5: pageVoltage();   break;
    case 6: pageCurrent();   break;
    case 7: pageBattery();   break;
    case 8: pageRawLog();    break;
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

  // --- I2C at 100kHz (DS1307-compatible) ---
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_FREQ);
  Wire.setTimeOut(50);

  analogReadResolution(12);
  analogSetPinAttenuation(THERM_PIN, ADC_11db);

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
      if (addr == 0x50) Serial.print(" AT24C32");
      if (addr == 0x68) Serial.print(" DS1307");
      if (addr == 0x6A) Serial.print(" LSM6DS3");
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

  // --- Gyroscope ---
  hasGyro = i2cProbe(GYRO_I2C_ADDR) && initGyro();
  if (!hasGyro) moduleGyro = false;
  Serial.printf("[GYRO]   %s\n", hasGyro ? "OK" : "MISSING/UNKNOWN");

  // --- WiFi ---

  for(int i=0 ; i<= WIFI_SSID.size(); i++) {
  

    Serial.printf("[WiFi]   Trying SSID: %s\n", WIFI_SSID[i]);

  if (hasOLED) {
    oled.clearDisplay();
    oled.setCursor(0, 0);
    oled.println("WiFi...");
    oled.println("Connecting to:");
    oled.println(WIFI_SSID[i]);
    oled.display();
  }

  if (WiFi.begin(WIFI_SSID[i], WIFI_PASS[i]) == WL_CONNECTED) {
    syncClockFromNtp();
    Serial.printf("[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
    if (hasOLED) {
      oled.clearDisplay();
      oled.setCursor(0, 0);
      oled.println("WiFi OK");
      oled.println(WiFi.localIP().toString());
      oled.println("\nMQTT: broker.emqx.io");
      oled.display();
      break;
    }
  } else if (i < WIFI_SSID.size()) {
    Serial.printf("[WiFi] Nereusit — Se incearca urmatoarea varianta: %s", WIFI_SSID[i]);
    if(hasOLED)
    {oled.clearDisplay();
    oled.printf("[WiFi] Nereusit — Se incearca varianta %d %s", i,WIFI_SSID[i]);
  
  }else{
    oled.println("[WiFi] Nereusit - MOD OFFLINE");
  }
}
  }
  // --- MQTT ---
  mqttClientId = "esp32-hs-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setCallback(onMqttMessage);
  mqtt.setBufferSize(1024);
  connectMqtt();

  loadBatteryState();

  for (size_t i = 0; i < GPIO_PUBLISH_PIN_COUNT; i++) {
    gGpioLastState[i] = -1;
  }

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
    Serial.printf("[%s] T=%.1f TH=%u GX=%.1f GY=%.1f GZ=%.1f V=%.2f I=%.1f CPU=%.1f%%\n",
      gTimestamp, gTemp, gThermRaw, gGyroX, gGyroY, gGyroZ, gVoltage, gCurrentMA, gCpuLoad);
  }

  delay(10);
}