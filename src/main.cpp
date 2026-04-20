/*CEL MAI BUN COD FINAL ZIUA 1, MERGI TOT DAR NU PI WEBSITE
 * ================================================================
 * Hard&Soft Competition - Task 3 (Skydiver Monitor)
 * ESP32-C3 Super Mini — Environmental Monitor (BLE-Only)
 * ================================================================
 *
 * COMPONENTS:
 *   - ESP32-C3 Super Mini          (3.3V logic)
 *   - SSD1306 0.96" OLED 128x64   (I2C 0x3C)
 *   - INA219 current/power monitor (I2C 0x40)
 *   - DS1307 RTC module HW-111    (I2C 0x68)
 *   - 10k NTC thermistor divider on GPIO0 (ADC)
 *   - LSM6DS3-compatible gyro      (I2C 0x6A)
 *   - MPU9250/MPU6500 IMU          (I2C 0x68/0x69)
 *   - MAX30102 (Blood Oxygen/HR)   (I2C 0x57)
 *   - LG INR18650MH1 battery      (3.6V nom, 3200mAh)
 *   - 2x tactile push buttons
 *
 * COMMUNICATION:
 *   - BLE (Bluetooth Low Energy) ONLY — no WiFi/Internet
 *   - Designed for offline operation between wearable and phone
 *
 * WIRING:
 *   I2C Bus:
 *     ESP32 GPIO8 (SDA) --> OLED, INA219, DS1307, LSM6DS3, MAX30102
 *     ESP32 GPIO9 (SCL) --> OLED, INA219, DS1307, LSM6DS3, MAX30102
 *   Thermistor divider:
 *     3.3V -> 10k NTC -> GPIO0 -> 10k resistor -> GND
 *   Buttons (active LOW, internal pull-up):
 *     ESP32 GPIO3 --> Button NEXT --> GND
 *     ESP32 GPIO4 --> Button PREV --> GND
 *
 * CHANGES from MQTT version:
 *   - Removed WiFi and MQTT (online mode)
 *   - Removed WebSocket IMU streaming
 *   - Added BLE GATT server with sensor characteristics
 *   - Sensor data published via BLE notifications
 *   - Offline-only operation for wearable devices
 *
 * LIBRARIES:
 *   - Adafruit SSD1306, Adafruit GFX, Adafruit INA219
 *   - RTClib (Adafruit), ArduinoJson v7+
 *   - ESP32 BLE libraries (BLEDevice, BLEServer, etc.)
 */

#include <Arduino.h>
#include <algorithm>
#include <cmath>
#include <stdarg.h>

#include <Wire.h>
#include <Thermistor.h>
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
#include <SparkFunMPU9250-DMP.h>
#include "ble_manager.h"
#include "sensors/sensor_oled.h"

extern "C" int min(int a, int b) {
  return (a < b) ? a : b;
}

static const float BATTERY_CAPACITY = 3200.0f;  // mAh
static const float BATT_REST_CURRENT_MA = 25.0f;  // Rest threshold for OCV correction
static const uint32_t BATT_REST_TIME_MS = 30000;
static const float BATT_OCV_MISMATCH_RECOVER_PCT = 20.0f;
static const uint32_t BATT_SAVE_INTERVAL_MS = 60000;
static const float BATT_PRESENT_ON_V = 2.8f;
static const float BATT_PRESENT_OFF_V = 2.4f;
static const uint32_t BATT_PRESENCE_DEBOUNCE_MS = 5000;
static const float BATT_CURRENT_DEADBAND_MA = 5.0f;
static const char* PREF_NS = "battery";
static const char* PREF_KEY_USED_MAH = "used_mAh";

// =============================================================================
// BLE publishing cadence
// =============================================================================
static const uint32_t IMU_PUBLISH_INTERVAL_MS = 20;        // 20ms when BLE connected (50 Hz)
static const uint32_t IMU_PUBLISH_IDLE_INTERVAL_MS = 5000; // 5s when BLE not connected (saves power)
static const uint32_t SENSOR_PUBLISH_INTERVAL_MS = 250;    // 4 Hz for smoother slow-parameter updates

// =============================================================================
// Hardware pin map
// =============================================================================
static const uint8_t I2C_SDA   = 8;
static const uint8_t I2C_SCL   = 9;
static const uint8_t BTN_NEXT  = 3;
static const uint8_t BTN_PREV  = 4;
static const uint8_t GYRO_INT  = 10;  // MPU9250/6500 data ready interrupt pin

// GPIO snapshot pins to publish as raw digital states.
// GPIO18/19 are native USB D-/D+ on ESP32-C3; avoid probing them as GPIO.
static const uint8_t GPIO_PUBLISH_PINS[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 20, 21};
static const size_t GPIO_PUBLISH_PIN_COUNT = sizeof(GPIO_PUBLISH_PINS) / sizeof(GPIO_PUBLISH_PINS[0]);

// =============================================================================
// I2C + ADC
// =============================================================================
static const uint32_t I2C_FREQ      = 100000;  // 100kHz — safe for DS1307
static const uint8_t THERM_PIN      = 0;       // GPIO0: divider midpoint
static const float ADC_MAX          = 4095.0f; // ESP32-C3 12-bit ADC

// =============================================================================
// Thermistor model and filtering
// =============================================================================
// Circuit: 3.3V -> 10k NTC -> ADC(GPIO0) -> 10k fixed -> GND
static const float R_FIXED   = 10000.0f;
static const float R_NOMINAL = 10000.0f;  // 10k NTC @ 25°C
static const float T_NOMINAL = 25.0f;
static const float B_COEFF   = 3950.0f;   // Standard NTC B coefficient
static const float THERM_ADC_REF_V = 3.3f;       // kept for open/short conversion; voltage read via analogReadMilliVolts()
static const bool THERM_SUPPLY_IS_VBAT = false;    // Divider is powered from 3.3V rail; do not use INA/VBAT for NTC conversion
static const float THERM_SUPPLY_FALLBACK_V = 3.3f; // used when THERM_SUPPLY_IS_VBAT=true and live VBAT is unavailable
static const float THERM_FILTER_ALPHA = 0.18f;        // More responsive baseline smoothing
static const float THERM_FILTER_ALPHA_FAST = 0.45f;   // Fast response for real temperature changes
static const float THERM_FILTER_FAST_DELTA_C = 0.5f;  // Enter fast mode sooner
static const float THERM_FILTER_DEADBAND_C = 0.02f;   // Keep small deadband only for ADC jitter
static const uint8_t THERM_SAMPLE_COUNT = 17;         // Odd count for median selection
static const uint16_t THERM_SAMPLE_DELAY_US = 500;    // 0.5 ms between ADC reads
static const uint16_t THERM_TRIM_WINDOW_MV = 45;      // Keep samples close to median
// FIXED topology - do NOT auto-detect
// Circuit: 3.3V -> NTC -> ADC -> R_FIXED(10k) -> GND
static const bool THERM_NTC_TO_GND = false;
static const float THERM_CAL_GAIN = 1.0f;
static const float THERM_CAL_OFFSET_C = 0.0f;

// =============================================================================
// IMU registers and fusion tuning
// =============================================================================
// IMU support:
// - LSM6DS3/compatible at 0x6A
// - MPU6500/MPU9250 at 0x68 or 0x69 (AD0 strap dependent)
static const uint8_t IMU_ADDR_LSM6 = 0x6A;
static const uint8_t IMU_ADDR_MPU0 = 0x68;
static const uint8_t IMU_ADDR_MPU1 = 0x69;

static const uint8_t LSM_WHO_AM_I_REG = 0x0F;
static const uint8_t LSM_WHO_AM_I_VAL = 0x69;
static const uint8_t LSM_CTRL2_G = 0x11;
static const uint8_t LSM_OUTX_L_G = 0x22;

static const uint8_t MPU_WHO_AM_I_REG = 0x75;
static const uint8_t MPU_WHO_AM_I_6500 = 0x70;
static const uint8_t MPU_WHO_AM_I_9250 = 0x71;
static const uint8_t MPU_SMPLRT_DIV = 0x19;
static const uint8_t MPU_CONFIG = 0x1A;
static const uint8_t MPU_PWR_MGMT_1 = 0x6B;
static const uint8_t MPU_PWR_MGMT_2 = 0x6C;
static const uint8_t MPU_GYRO_CONFIG = 0x1B;
static const uint8_t MPU_ACCEL_CONFIG = 0x1C;
static const uint8_t MPU_ACCEL_XOUT_H = 0x3B;
static const uint8_t MPU_GYRO_XOUT_H = 0x43;
static const uint8_t MPU_INT_PIN_CFG = 0x37;  // Interrupt pin configuration register
static const uint8_t MPU_INT_ENABLE = 0x38;   // Interrupt enable register
static const uint8_t MPU_INT_STATUS = 0x3A;   // Interrupt status register (clear by reading)

static const uint8_t RTC_ADDR_DS1307 = 0x68;
static const uint8_t RTC_ADDR_ALT = 0x60;

// FS=+-500 dps for MPU (better low-rate resolution), +-2000 dps for LSM fallback.
static const float MPU_GYRO_SCALE_DPS = 1.0f / 65.5f;  // MPU6500/9250 @ +-500 dps
static const float LSM_GYRO_SCALE_DPS = 0.07f;         // LSM6DS3 @ +-2000 dps
static const float MPU_ACCEL_SCALE_G = 1.0f / 16384.0f; // MPU @ +-2g
static const uint8_t MPU_BIAS_SAMPLES = 96;
static const float GYRO_FILTER_ALPHA = 0.25f;  // REDUCED: 0.70→0.25 for faster real-time response (was causing 2-3s lag)
static const float GYRO_DEADBAND_DPS = 0.5f;  // TIGHTENED: 1.2→0.5 for better precision
static const float GYRO_STILL_DPS = 2.5f;
static const uint8_t GYRO_STILL_REQUIRED_SAMPLES = 6;
static const uint8_t ZUPT_STILL_REQUIRED_SAMPLES = 10;
static const uint32_t BIAS_RECAL_INTERVAL_MS = 300000;
static const float GYRO_BIAS_STILL_ALPHA = 0.015f;
static const float GYRO_BIAS_RECAL_ALPHA = 0.06f;
static const float ACCEL_BIAS_STILL_ALPHA = 0.01f;
static const float ACCEL_BIAS_RECAL_ALPHA = 0.04f;
static const float ZUPT_VEL_DECAY = 0.02f;
static const float ZUPT_POS_HOLD_ALPHA = 0.002f;
static const float STATIONARY_WORLD_ACC_G = 0.08f;

// Quaternion fusion constants (gyro + accel).
static const float QUAT_ACC_GAIN = 1.8f;

static const uint8_t OLED_TITLE_H = 16;
static const uint8_t OLED_TITLE_TEXT_Y = 4;
static const signed char orientationDefault[9] = {0, 1, 0, 0, 0, 1, 1, 0, 0};
// SparkFun DMP path can hang on some ESP32-C3 + MPU setups during boot.
// Keep fallback to register fusion if DMP init fails.
#define USE_MPU_DMP 0

// =============================================================================
// Display and UI timing
// =============================================================================
static const uint8_t  SCREEN_W      = 128;
static const uint8_t  SCREEN_H      = 64;
static const uint8_t  NUM_PAGES     = 11;
static const uint16_t DEBOUNCE_MS   = 200;

// =============================================================================
// Global objects
// =============================================================================
static Adafruit_SSD1306 oled(SCREEN_W, SCREEN_H, &Wire, -1);
static Adafruit_INA219  ina;
static RTC_DS1307       rtc;
static Thermistor       thermistor(R_NOMINAL, R_FIXED, B_COEFF, 12, 3.3, 298.15);
static Preferences      prefs;
static MPU9250_DMP      mpu;

// =============================================================================
// BLE/runtime state
// =============================================================================
static bool oldBleConnected = false;

// Device presence flags
static bool hasOLED   = false;
static bool hasINA219 = false;
static bool hasRTC    = false;
static bool hasGyro   = false;
static uint8_t gRtcAddr = 0;

// Sensor and fusion state
static float    gTemp       = 0.0f;
static uint16_t gThermRaw   = 0;
static bool     gThermValid = false;
static float    gThermFiltered = 0.0f;
static bool     gThermFilterSeeded = false;
static int16_t  gGyroRawX   = 0;
static int16_t  gGyroRawY   = 0;
static int16_t  gGyroRawZ   = 0;
static int16_t  gAccelRawX  = 0;
static int16_t  gAccelRawY  = 0;
static int16_t  gAccelRawZ  = 0;
static float    gGyroX      = 0.0f;
static float    gGyroY      = 0.0f;
static float    gGyroZ      = 0.0f;
static float    gGyroAbs    = 0.0f;
static float    gGyroBiasX  = 0.0f;
static float    gGyroBiasY  = 0.0f;
static float    gGyroBiasZ  = 0.0f;
static float    gAccelBiasX  = 0.0f;
static float    gAccelBiasY  = 0.0f;
static float    gAccelBiasZ  = 0.0f;
static bool     gGyroFilterSeeded = false;
static bool     gAccelBiasSeeded = false;
static uint8_t  gGyroStillCount = 0;
static float    gAccelX     = 0.0f;
static float    gAccelY     = 0.0f;
static float    gAccelZ     = 0.0f;
static float    gRoll       = 0.0f;
static float    gPitch      = 0.0f;
static float    gYaw        = 0.0f;
static float    gHeading    = 0.0f;
static float    gQuat0      = 1.0f;
static float    gQuat1      = 0.0f;
static float    gQuat2      = 0.0f;
static float    gQuat3      = 0.0f;
static float    gLinAccX    = 0.0f;
static float    gLinAccY    = 0.0f;
static float    gLinAccZ    = 0.0f;
static float    gVelX       = 0.0f;
static float    gVelY       = 0.0f;
static float    gVelZ       = 0.0f;
static float    gPosX       = 0.0f;
static float    gPosY       = 0.0f;
static float    gPosZ       = 0.0f;
static float    gGravityX   = 0.0f;
static float    gGravityY   = 0.0f;
static float    gGravityZ   = 1.0f;
static bool     gFusionSeeded = false;
static uint32_t gLastFusionUs = 0;
static uint32_t gLastMotionUs = 0;
static uint8_t  gMotionStillCount = 0;
static bool     gMotionSeeded = false;
static float    gMotionDtMs = 0.0f;
static uint32_t gLastBiasUpdateMs = 0;
static bool     gMotionAnchorValid = false;
static float    gMotionAnchorX = 0.0f;
static float    gMotionAnchorY = 0.0f;
static float    gMotionAnchorZ = 0.0f;
static uint8_t  gImuAddr    = 0;
static uint8_t  gImuWhoAmI  = 0;
static bool     gImuIsMpu   = false;
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

// =============================================================================
// Internal state
// =============================================================================
static volatile int8_t page = 0;
static uint32_t lastEnergy  = 0;
// Mission Timer State
static bool gTimerRunning = false;
static uint32_t gTimerStartMs = 0;
static uint32_t gTimerElapsedMs = 0;
// CPU load tracking
static volatile uint32_t cpuIdleHookCount = 0;
static uint32_t cpuLastIdleHookCount = 0;
static uint32_t cpuLastLoadSampleMs = 0;
static uint32_t cpuPrevIdleHookCount = 0;
static float cpuIdleRateRef = 0.0f;
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
static uint32_t gBattLowAccumMs = 0;
static uint32_t gBattHighAccumMs = 0;

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

// I2C scan cache for OLED page
static const uint8_t I2C_SCAN_MAX_DEVICES = 16;
static uint8_t gI2cFound[I2C_SCAN_MAX_DEVICES];
static uint8_t gI2cFoundCount = 0;
static bool gI2cScanOverflow = false;
static uint32_t gLastI2cScanMs = 0;

// Periodic task timers
static uint32_t lastSensorRead = 0;    // Other sensors: 1Hz
static uint32_t lastOtherSensorRead = 0;
static uint32_t lastImuPublishMs = 0;

// Interrupt hand-off flags
static volatile bool isrNextFlag = false;
static volatile bool isrPrevFlag = false;

// Gyro ISR event counter (data ready from MPU9250/6500)
static volatile uint16_t gyroDataReadyCount = 0;
static uint32_t gLastGyroReadMs = 0;
static uint32_t gIgnoreButtonsUntilMs = 0;
static bool gMpuDmpActive = false;
static uint32_t gLastImuSummaryMs = 0;

// Module enable switches
static bool moduleTemp    = true;
static bool moduleGyro    = true;
static bool moduleCPU     = true;
static bool moduleCurrent = true;
static volatile bool moduleCpuStress = false;

// =============================================================================
// Forward declarations
// =============================================================================
static void publishState();
static void drawPage();
static float measureCpuLoadPct();
static void updateEulerFromQuaternion();
static size_t buildImuJSON(char* buf, size_t bufSize);
static size_t buildRawGpioJSON(char* buf, size_t bufSize);
static void printDegC();
static bool initGyro();
static bool readGyro();
static bool i2cProbe(uint8_t addr);
static bool cpuIdleHook();
static bool getCpuLoadFromRuntimeStats(float& loadOut);
static float getCpuLoadFromIdleHook();
static void cpuStressTask(void* parm);
static void appendOledLog(const char* fmt, ...);
static void updateOledRawLog();
static const char* gpioShortName(uint8_t pin);
static void scanI2cDevices();
static const char* i2cDeviceName(uint8_t addr);
static bool rtcWriteDateTime(int year, int month, int day, int hour, int minute, int second);
static bool rtcReadDateTime(int& year, int& month, int& day, int& hour, int& minute, int& second);
static bool thermistorTempFromRatio(float adcRatio, bool ntcToGnd, float& tempC);
static bool thermistorTempFromResistance(float rNtc, float r25, float& tempC);
static void updateOrientationFusion(float dt, float gx, float gy, float gz, float ax, float ay, float az);
static void handleBleCommand(const String& cmd);
static void configureI2cAndAdc();
static void initCpuLoadSubsystem();
static void initButtonInterrupts();
static void scanAndPrintI2cMap();
static void initOledDisplay();
static void initIna219Sensor();
static void initRtcClock();
static void initGyroSubsystem();
static void initBleSubsystem();
static void processGyroFastPath();
static void publishImuFastIfDue();
static void runOneHzTasks();
static void refreshI2cScanCacheIfDue();
static void updateBleConnectionUiState();
static void refreshRunningTimerPage();


#include "sensors/sensor_mpu_fusion.cpp"

#define HS_MAIN_COMPOSED 1
#include "utils/battery_utils.cpp"
#include "utils/oled_log_utils.cpp"
#include "utils/cpu_utils.cpp"
#include "utils/json_utils.cpp"
#include "utils/button_utils.cpp"
#include "utils/i2c_utils.cpp"

static bool systemTimeValid() {
  time_t now = time(nullptr);
  // 2024-01-01 UTC guard to reject unsynced epoch values
  return now > 1704067200;
}

static void syncClockFromNtp() {
  // Removed: NTP sync no longer available in BLE-only (offline) mode
  // RTC is used for timekeeping. Set RTC manually if needed via compile time.
  Serial.println("[TIME] Using RTC for time source (no NTP in offline mode)");
}

// WebSocket handler removed - now using BLE notifications instead

// =============================================================================
// ISRs
// =============================================================================
void IRAM_ATTR isrBtnNext() { isrNextFlag = true; }
void IRAM_ATTR isrBtnPrev() { isrPrevFlag = true; }
void IRAM_ATTR isrGyroDataReady() {
  if (gyroDataReadyCount < 1000) {
    gyroDataReadyCount++;
  }
}  // MPU data ready interrupt

// Sensor implementation units included in composed build.
#include "sensors/sensor_ntc.cpp"
#include "sensors/sensor_mpu_driver.cpp"
#include "sensors/sensor_rtc.cpp"

// =============================================================================
// Local helpers
// =============================================================================
static bool i2cProbe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

#include "sensors/sensor_ina219.cpp"
#include "utils/sensor_polling_utils.cpp"

// =============================================================================
// BLE command handling
// =============================================================================

static void handleBleCommand(const String& cmd) {
  if (cmd == "START") {
    if (!gTimerRunning) {
      gTimerStartMs = millis() - gTimerElapsedMs;  // Resume from paused time.
      gTimerRunning = true;
      Serial.println("[TIMER] Started!");
    }
  } else if (cmd == "STOP") {
    if (gTimerRunning) {
      gTimerElapsedMs = millis() - gTimerStartMs;
      gTimerRunning = false;
      Serial.println("[TIMER] Stopped!");
    }
  } else if (cmd == "RESET") {
    gTimerRunning = false;
    gTimerElapsedMs = 0;
    Serial.println("[TIMER] Reset!");
  }
}


// =============================================================================
// State publishing
// =============================================================================
// BLE replaces the old MQTT publish path; publishing happens from loop().

static void publishState() {
  // In BLE mode, module state is communicated via BLE characteristics
  Serial.println("[BLE] Module state updated");
}

// OLED page renderer implementation unit.
#include "sensors/sensor_oled.cpp"

// =============================================================================
// Setup/loop decomposition helpers
// =============================================================================
static void configureI2cAndAdc() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_FREQ);
  Wire.setTimeOut(50);

  analogReadResolution(12);
  // ADC_11db extends measurable range close to 3.3V for the thermistor divider.
  analogSetPinAttenuation(THERM_PIN, ADC_11db);
}

static void initCpuLoadSubsystem() {
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
  cpuPrevIdleHookCount = cpuIdleHookCount;
  cpuLastLoadSampleMs = millis();

  moduleCpuStress = false;
  if (xTaskCreate(cpuStressTask, "cpu_stress", 3072, nullptr, 1, nullptr) != pdPASS) {
    Serial.println("[CPU] Stress task create FAILED");
  }
}

static void initButtonInterrupts() {
  pinMode(BTN_NEXT, INPUT_PULLUP);
  pinMode(BTN_PREV, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTN_NEXT), isrBtnNext, FALLING);
  attachInterrupt(digitalPinToInterrupt(BTN_PREV), isrBtnPrev, FALLING);
}

static void scanAndPrintI2cMap() {
  Serial.println("[I2C] Scan (100kHz):");
  scanI2cDevices();
  for (uint8_t i = 0; i < gI2cFoundCount; i++) {
    const uint8_t addr = gI2cFound[i];
    Serial.printf("  0x%02X %s\n", addr, i2cDeviceName(addr));
  }
  if (gI2cScanOverflow) {
    Serial.println("  ... more devices (list truncated)");
  }
  Serial.println();
}

static void initOledDisplay() {
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
}

static void initIna219Sensor() {
  hasINA219 = i2cProbe(0x40);
  if (hasINA219) {
    hasINA219 = ina.begin();
    if (hasINA219) {
      ina.setCalibration_16V_400mA();
    }
  }

  loadBatteryState();
  Serial.printf("[INA219] %s\n", hasINA219 ? "OK (16V/400mA)" : "MISSING");
}

static void initRtcClock() {
  gRtcAddr = 0;
  if (i2cProbe(RTC_ADDR_DS1307)) {
    gRtcAddr = RTC_ADDR_DS1307;
  } else if (i2cProbe(RTC_ADDR_ALT)) {
    gRtcAddr = RTC_ADDR_ALT;
  }

  hasRTC = (gRtcAddr != 0);
  if (hasRTC && gRtcAddr == RTC_ADDR_DS1307) {
    rtc.begin();
    if (!rtc.isrunning()) {
      Serial.println("[RTC]    Setting to compile time");
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
  }

  if (hasRTC) {
    Serial.printf("[RTC]    OK @0x%02X\n", gRtcAddr);
  } else {
    Serial.printf("[RTC]    MISSING\n");
  }
}

static void initGyroSubsystem() {
  pinMode(GYRO_INT, INPUT_PULLUP);

  hasGyro = initGyro();
  if (!hasGyro) {
    moduleGyro = false;
  }

  Serial.printf("[GYRO]   %s", hasGyro ? "OK" : "MISSING/UNKNOWN");
  if (!hasGyro) {
    Serial.println();
    return;
  }

  Serial.printf(" @0x%02X %s\n", gImuAddr, gImuIsMpu ? "(MPU6500/9250)" : "(LSM6DS3)");
  if (gImuIsMpu) {
    const char* modelName = (gImuWhoAmI == MPU_WHO_AM_I_9250) ? "MPU9250" :
                            (gImuWhoAmI == MPU_WHO_AM_I_6500) ? "MPU6500" : "MPU-unknown";
    Serial.printf("[IMU]    WHO_AM_I=0x%02X -> %s\n", gImuWhoAmI, modelName);

    attachInterrupt(digitalPinToInterrupt(GYRO_INT), isrGyroDataReady, gMpuDmpActive ? FALLING : RISING);
    Serial.printf("[GYRO-INT] Attached to GPIO%u (data ready interrupt)\n", GYRO_INT);
    Serial.printf("[GYRO]   MPU mode: %s\n", gMpuDmpActive ? "DMP quaternion" : "register fusion");
  } else {
    Serial.printf("[IMU]    WHO_AM_I=0x%02X\n", gImuWhoAmI);
  }
}

static void initBleSubsystem() {
  Serial.println("[BLE] Initializing BLE communication...");
  BleManager::setCommandHandler(handleBleCommand);
  BleManager::init();
}

static void processGyroFastPath() {
  if (!hasGyro || !moduleGyro) {
    return;
  }

  uint16_t pending = 0;
  noInterrupts();
  pending = gyroDataReadyCount;
  gyroDataReadyCount = 0;
  interrupts();

  if (pending > 3) {
    pending = 3;
  }

  while (pending--) {
    (void)readGyro();
    gLastGyroReadMs = millis();
  }

  // Fallback polling path in case the interrupt line is missing/unreliable.
  if (millis() - gLastGyroReadMs >= 20) {
    (void)readGyro();
    gLastGyroReadMs = millis();
  }
}

static void publishImuFastIfDue() {
  if (!hasGyro || !moduleGyro) {
    return;
  }

  const bool connected = BleManager::isConnected();
  const uint32_t nowMs = millis();

  if (connected && (nowMs - lastImuPublishMs >= IMU_PUBLISH_INTERVAL_MS)) {
    lastImuPublishMs = nowMs;
    BleManager::publishImuData({
      gUptime,
      gQuat0,
      gQuat1,
      gQuat2,
      gQuat3,
      gRoll,
      gPitch,
      gYaw,
      gMotionStillCount,
      ZUPT_STILL_REQUIRED_SAMPLES,
      gAccelX,
      gAccelY,
      gAccelZ,
      gGyroX,
      gGyroY,
      gGyroZ
    });
    return;
  }

  if (!connected && (nowMs - lastImuPublishMs >= IMU_PUBLISH_IDLE_INTERVAL_MS)) {
    lastImuPublishMs = nowMs;
  }
}

static void runOneHzTasks() {
  if (millis() - lastOtherSensorRead < SENSOR_PUBLISH_INTERVAL_MS) {
    return;
  }

  lastOtherSensorRead = millis();
  readOtherSensors();

  if (BleManager::isConnected()) {
    BleManager::publishSensorData({
      gTimestamp,
      gUptime,
      moduleTemp,
      gThermValid,
      gTemp,
      moduleGyro,
      hasGyro,
      gGyroX,
      gGyroY,
      gGyroZ,
      gRoll,
      gPitch,
      gYaw,
      gQuat0,
      gQuat1,
      gQuat2,
      gQuat3,
      gMotionStillCount,
      ZUPT_STILL_REQUIRED_SAMPLES,
      moduleCurrent,
      gVoltage,
      gCurrentMA,
      gBattPct,
      moduleCPU,
      gCpuLoad
    });

    BleManager::publishImuData({
      gUptime,
      gQuat0,
      gQuat1,
      gQuat2,
      gQuat3,
      gRoll,
      gPitch,
      gYaw,
      gMotionStillCount,
      ZUPT_STILL_REQUIRED_SAMPLES,
      gAccelX,
      gAccelY,
      gAccelZ,
      gGyroX,
      gGyroY,
      gGyroZ
    });
  }

  drawPage();

  Serial.printf("[%s] T=%.1f TH=%u GX=%.1f GY=%.1f GZ=%.1f V=%.2f I=%.1f CPU=%.1f%%\n",
                gTimestamp, gTemp, gThermRaw, gGyroX, gGyroY, gGyroZ, gVoltage, gCurrentMA, gCpuLoad);
}

static void refreshI2cScanCacheIfDue() {
  if (millis() - gLastI2cScanMs >= 3000) {
    scanI2cDevices();
  }
}

static void updateBleConnectionUiState() {
  const bool bleConnected = BleManager::isConnected();
  if (oldBleConnected == bleConnected) {
    return;
  }

  oldBleConnected = bleConnected;
  if (bleConnected) {
    Serial.println("[BLE] Client connected");
    return;
  }

  Serial.println("[BLE] Client disconnected");
  page = 3;
  drawPage();
}

static void refreshRunningTimerPage() {
  if (hasOLED && page == 10 && gTimerRunning) {
    drawPage();
  }
}



// =============================================================================
// Arduino lifecycle
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== Hard&Soft Task 3 — Skydiver Monitor (BLE-Only) ===\n");

  configureI2cAndAdc();
  initCpuLoadSubsystem();
  initButtonInterrupts();
  scanAndPrintI2cMap();
  initOledDisplay();
  initIna219Sensor();
  initRtcClock();
  initGyroSubsystem();

  initBleSubsystem();
  
  // Run splash/pairing UX after peripherals and BLE are initialized.
  runOledBlePairingAnimation();
}

void loop() {
  handleButtons();

  processGyroFastPath();
  publishImuFastIfDue();
  runOneHzTasks();
  refreshI2cScanCacheIfDue();
  updateBleConnectionUiState();
  refreshRunningTimerPage();



  delay(2);  // Small delay to yield to other tasks
}