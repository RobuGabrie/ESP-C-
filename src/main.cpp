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
 *   - MPU9250 IMU with AK8963 mag  (I2C 0x68/0x69)
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
 *     ESP32 GPIO8 (SDA) --> OLED, INA219, DS1307, LSM6DS3, MAX30102, AK8963
 *     ESP32 GPIO9 (SCL) --> OLED, INA219, DS1307, LSM6DS3, MAX30102, AK8963
 *   Thermistor divider:
 *     3.3V -> 10k resistor -> GPIO0 -> 10k NTC -> GND
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

// === BLE Includes ===
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

extern "C" int min(int a, int b) {
  return (a < b) ? a : b;
}

// ================================================================
// BLE CONFIGURATION
// ================================================================
#define BLE_DEVICE_NAME "Skydiver-Monitor"
#define SERVICE_UUID "180D"  // Heart Rate Service
#define CHAR_SENSOR_DATA_UUID "2A3F"  // Heart Rate Measurement (repurposed)
#define CHAR_TEMPERATURE_UUID "2A1C"  // Temperature Measurement
#define CHAR_GYRO_UUID "2A4E"  // Pulse Oxymetry UUID (repurposed for gyro)
#define CHAR_BATTERY_UUID "2A19"  // Battery Level
#define CHARACTERISTIC_NOTIFY_UUID "a0000001-1000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_IMU_UUID "a0000002-1000-1000-8000-00805f9b34fb"

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

// ================================================================
// BLE SENSOR PUBLISHING INTERVALS
// ================================================================
static const uint32_t IMU_PUBLISH_INTERVAL_MS = 100;       // 100ms when BLE connected (~10 Hz)
static const uint32_t IMU_PUBLISH_IDLE_INTERVAL_MS = 5000; // 5s when BLE not connected (saves power)

// ================================================================
// PINS
// ================================================================
static const uint8_t I2C_SDA   = 8;
static const uint8_t I2C_SCL   = 9;
static const uint8_t BTN_NEXT  = 3;
static const uint8_t BTN_PREV  = 4;
static const uint8_t GYRO_INT  = 10;  // MPU9250/6500 data ready interrupt pin

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
// Circuit: 3.3V -> R_FIXED(10k) -> ADC -> NTC(to GND) -> GND
static const bool THERM_NTC_TO_GND = true;
static const float THERM_CAL_GAIN = 1.0f;
static const float THERM_CAL_OFFSET_C = -6.0f;  // One-point calibration baseline; fine-tune after stability checks

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

// AK8963 magnetometer inside MPU9250 (not present on MPU6500)
static const uint8_t AK8963_ADDR = 0x0C;
static const uint8_t AK8963_WHO_AM_I = 0x00;
static const uint8_t AK8963_ST1 = 0x02;
static const uint8_t AK8963_XOUT_L = 0x03;
static const uint8_t AK8963_CNTL1 = 0x0A;
static const uint8_t AK8963_ASAX = 0x10;
static const uint8_t AK8963_WHO_AM_I_VAL = 0x48;

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

// Kalman orientation fusion constants (gyro + accel) and yaw correction with mag.
static const float KALMAN_Q_ANGLE = 0.001f;
static const float KALMAN_Q_BIAS = 0.003f;
static const float KALMAN_R_MEASURE = 0.03f;
static const float YAW_MAG_BLEND_MIN = 0.03f;
static const float YAW_MAG_BLEND_MAX = 0.14f;
static const float QUAT_ACC_GAIN = 1.8f;
static const float QUAT_MAG_GAIN = 1.4f;

static const uint8_t OLED_TITLE_H = 16;
static const uint8_t OLED_TITLE_TEXT_Y = 4;
static const signed char orientationDefault[9] = {0, 1, 0, 0, 0, 1, 1, 0, 0};
// SparkFun DMP path can hang on some ESP32-C3 + MPU setups during boot.
// Keep fallback to register fusion if DMP init fails.
#define USE_MPU_DMP 0

// ================================================================
// DISPLAY & TIMING
// ================================================================
static const uint8_t  SCREEN_W      = 128;
static const uint8_t  SCREEN_H      = 64;
static const uint8_t  NUM_PAGES     = 11;
static const uint16_t DEBOUNCE_MS   = 200;

// ================================================================
// OBJECTS
// ================================================================
static Adafruit_SSD1306 oled(SCREEN_W, SCREEN_H, &Wire, -1);
static Adafruit_INA219  ina;
static RTC_DS1307       rtc;
static Thermistor       thermistor(R_NOMINAL, R_FIXED, B_COEFF, 12, 3.3, 298.15);
static Preferences      prefs;
static MPU9250_DMP      mpu;

// === BLE Objects ===
static BLEServer*       pBleServer = nullptr;
static BLEService*      pService = nullptr;
static BLECharacteristic* pCharSensorData = nullptr;
static BLECharacteristic* pCharTemperature = nullptr;
static BLECharacteristic* pCharGyro = nullptr;
static BLECharacteristic* pCharBattery = nullptr;
static BLECharacteristic* pCharIMU = nullptr;
static bool bleConnected = false;
static bool oldBleConnected = false;
static uint32_t gBleSensorNotifyCount = 0;
static uint32_t gBleImuNotifyCount = 0;
static uint32_t gBleImuSkipLogMs = 0;

// ================================================================
// DEVICE STATUS FLAGS (set during setup, never change after)
// ================================================================
static bool hasOLED   = false;
static bool hasINA219 = false;
static bool hasRTC    = false;
static bool hasGyro   = false;
static uint8_t gRtcAddr = 0;

// ================================================================
// SENSOR DATA
// ================================================================
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
static bool     gHasMag     = false;
static int16_t  gMagRawX    = 0;
static int16_t  gMagRawY    = 0;
static int16_t  gMagRawZ    = 0;
static float    gMagX       = 0.0f;
static float    gMagY       = 0.0f;
static float    gMagZ       = 0.0f;
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
static uint32_t gLastMagReadMs = 0;
static float    gMagAdjX    = 1.0f;
static float    gMagAdjY    = 1.0f;
static float    gMagAdjZ    = 1.0f;
static uint8_t  gImuAddr    = 0;
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

// ================================================================
// INTERNAL STATE
// ================================================================
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

// Sensor read interval tracking
static uint32_t lastSensorRead = 0;    // Other sensors: 1Hz
static uint32_t lastOtherSensorRead = 0;
static uint32_t lastImuPublishMs = 0;

// Button ISR flags
static volatile bool isrNextFlag = false;
static volatile bool isrPrevFlag = false;

// Gyro ISR event counter (data ready from MPU9250/6500)
static volatile uint16_t gyroDataReadyCount = 0;
static uint32_t gLastGyroReadMs = 0;
static uint32_t gIgnoreButtonsUntilMs = 0;
static bool gMpuDmpActive = false;

// MQTT client ID (removed — no longer used with BLE)
// static String mqttClientId;

// Module enable/disable (toggled via BLE or default)
static bool moduleTemp    = true;
static bool moduleGyro    = true;
static bool moduleCPU     = true;
static bool moduleCurrent = true;
static volatile bool moduleCpuStress = false;

struct KalmanAxis {
  float angle;
  float bias;
  float P00;
  float P01;
  float P10;
  float P11;
};

static KalmanAxis gKalRoll = {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f};
static KalmanAxis gKalPitch = {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f};

// Forward declarations for functions used before their definitions.
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
static bool readMagnetometer();
static float kalmanUpdateAxis(KalmanAxis& k, float measuredAngle, float measuredRate, float dt);
static void updateOrientationFusion(float dt, float gx, float gy, float gz, float ax, float ay, float az);
static void initBLE();
static void publishSensorDataBLE();
static void publishIMUDataBLE();

static float wrapAngle180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

static float normalize360(float a) {
  while (a >= 360.0f) a -= 360.0f;
  while (a < 0.0f) a += 360.0f;
  return a;
}

static void updateEulerFromQuaternion() {
  const float q0 = gQuat0;
  const float q1 = gQuat1;
  const float q2 = gQuat2;
  const float q3 = gQuat3;

  const float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
  const float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
  gRoll = atan2f(sinr_cosp, cosr_cosp) * 57.29578f;

  const float sinp = 2.0f * (q0 * q2 - q3 * q1);
  if (fabsf(sinp) >= 1.0f) {
    gPitch = copysignf(90.0f, sinp);
  } else {
    gPitch = asinf(sinp) * 57.29578f;
  }

  const float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
  const float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
  gYaw = normalize360(atan2f(siny_cosp, cosy_cosp) * 57.29578f);
}

static void updateMotionEstimate(float dt) {
  if (dt <= 0.0f) return;
  gMotionDtMs = dt * 1000.0f;

  const float q0 = gQuat0;
  const float q1 = gQuat1;
  const float q2 = gQuat2;
  const float q3 = gQuat3;

  const float gravX = 2.0f * (q1 * q3 - q0 * q2);
  const float gravY = 2.0f * (q0 * q1 + q2 * q3);
  const float gravZ = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

  gGravityX = gravX;
  gGravityY = gravY;
  gGravityZ = gravZ;

  const float linBodyX = gAccelX - gravX;
  const float linBodyY = gAccelY - gravY;
  const float linBodyZ = gAccelZ - gravZ;

  const float r00 = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
  const float r01 = 2.0f * (q1 * q2 - q0 * q3);
  const float r02 = 2.0f * (q1 * q3 + q0 * q2);
  const float r10 = 2.0f * (q1 * q2 + q0 * q3);
  const float r11 = 1.0f - 2.0f * (q1 * q1 + q3 * q3);
  const float r12 = 2.0f * (q2 * q3 - q0 * q1);
  const float r20 = 2.0f * (q1 * q3 - q0 * q2);
  const float r21 = 2.0f * (q2 * q3 + q0 * q1);
  const float r22 = 1.0f - 2.0f * (q1 * q1 + q2 * q2);

  const float worldLinGx = r00 * linBodyX + r01 * linBodyY + r02 * linBodyZ;
  const float worldLinGy = r10 * linBodyX + r11 * linBodyY + r12 * linBodyZ;
  const float worldLinGz = r20 * linBodyX + r21 * linBodyY + r22 * linBodyZ;

  constexpr float G_TO_MPS2 = 9.80665f;
  gLinAccX = worldLinGx * G_TO_MPS2;
  gLinAccY = worldLinGy * G_TO_MPS2;
  gLinAccZ = worldLinGz * G_TO_MPS2;

  const float accMagG = sqrtf(worldLinGx * worldLinGx + worldLinGy * worldLinGy + worldLinGz * worldLinGz);
  const bool nearlyStill = accMagG < STATIONARY_WORLD_ACC_G && gGyroAbs < GYRO_STILL_DPS;

  if (!gMotionSeeded) {
    gMotionSeeded = true;
    gVelX = 0.0f;
    gVelY = 0.0f;
    gVelZ = 0.0f;
    gPosX = 0.0f;
    gPosY = 0.0f;
    gPosZ = 0.0f;
  }

  if (nearlyStill) {
    if (gMotionStillCount < 255) gMotionStillCount++;
  } else {
    gMotionStillCount = 0;
  }

  const bool zuptActive = (gMotionStillCount >= ZUPT_STILL_REQUIRED_SAMPLES);

  if (zuptActive) {
    if (!gMotionAnchorValid) {
      gMotionAnchorX = gPosX;
      gMotionAnchorY = gPosY;
      gMotionAnchorZ = gPosZ;
      gMotionAnchorValid = true;
    }

    gVelX = 0.0f;
    gVelY = 0.0f;
    gVelZ = 0.0f;

    gPosX = gPosX * (1.0f - ZUPT_POS_HOLD_ALPHA) + gMotionAnchorX * ZUPT_POS_HOLD_ALPHA;
    gPosY = gPosY * (1.0f - ZUPT_POS_HOLD_ALPHA) + gMotionAnchorY * ZUPT_POS_HOLD_ALPHA;
    gPosZ = gPosZ * (1.0f - ZUPT_POS_HOLD_ALPHA) + gMotionAnchorZ * ZUPT_POS_HOLD_ALPHA;
  } else {
    gMotionAnchorValid = false;
    gVelX = (gVelX + gLinAccX * dt) * 0.995f;
    gVelY = (gVelY + gLinAccY * dt) * 0.995f;
    gVelZ = (gVelZ + gLinAccZ * dt) * 0.995f;

    gPosX += gVelX * dt;
    gPosY += gVelY * dt;
    gPosZ += gVelZ * dt;
  }

  if (!isfinite(gPosX)) gPosX = 0.0f;
  if (!isfinite(gPosY)) gPosY = 0.0f;
  if (!isfinite(gPosZ)) gPosZ = 0.0f;
}

static void updateInertialBiases(bool stationary, float axMeas, float ayMeas, float azMeas, float gxRate, float gyRate, float gzRate) {
  uint32_t nowMs = millis();
  if (!stationary) return;
  if (gLastBiasUpdateMs != 0 && nowMs - gLastBiasUpdateMs < BIAS_RECAL_INTERVAL_MS) return;

  gLastBiasUpdateMs = nowMs;

  float gyroAlpha = (gMotionStillCount >= (ZUPT_STILL_REQUIRED_SAMPLES * 2)) ? GYRO_BIAS_RECAL_ALPHA : GYRO_BIAS_STILL_ALPHA;
  float accelAlpha = (gMotionStillCount >= (ZUPT_STILL_REQUIRED_SAMPLES * 2)) ? ACCEL_BIAS_RECAL_ALPHA : ACCEL_BIAS_STILL_ALPHA;

  if (!gAccelBiasSeeded) {
    gAccelBiasX = axMeas - gGravityX;
    gAccelBiasY = ayMeas - gGravityY;
    gAccelBiasZ = azMeas - gGravityZ;
    gAccelBiasSeeded = true;
  } else {
    gGyroBiasX += gyroAlpha * (gxRate - gGyroBiasX);
    gGyroBiasY += gyroAlpha * (gyRate - gGyroBiasY);
    gGyroBiasZ += gyroAlpha * (gzRate - gGyroBiasZ);

    gAccelBiasX += accelAlpha * ((axMeas - gGravityX) - gAccelBiasX);
    gAccelBiasY += accelAlpha * ((ayMeas - gGravityY) - gAccelBiasY);
    gAccelBiasZ += accelAlpha * ((azMeas - gGravityZ) - gAccelBiasZ);
  }
}

static float kalmanUpdateAxis(KalmanAxis& k, float measuredAngle, float measuredRate, float dt) {
  float rate = measuredRate - k.bias;
  k.angle += dt * rate;

  k.P00 += dt * (dt * k.P11 - k.P01 - k.P10 + KALMAN_Q_ANGLE);
  k.P01 -= dt * k.P11;
  k.P10 -= dt * k.P11;
  k.P11 += KALMAN_Q_BIAS * dt;

  float S = k.P00 + KALMAN_R_MEASURE;
  float K0 = k.P00 / S;
  float K1 = k.P10 / S;

  float y = measuredAngle - k.angle;
  k.angle += K0 * y;
  k.bias += K1 * y;

  float P00_temp = k.P00;
  float P01_temp = k.P01;
  k.P00 -= K0 * P00_temp;
  k.P01 -= K0 * P01_temp;
  k.P10 -= K1 * P00_temp;
  k.P11 -= K1 * P01_temp;

  return k.angle;
}

static void updateOrientationFusion(float dt, float gx, float gy, float gz, float ax, float ay, float az) {
  const float DEG_TO_RAD_F = 0.0174532925f;
  float wx = gx * DEG_TO_RAD_F;
  float wy = gy * DEG_TO_RAD_F;
  float wz = gz * DEG_TO_RAD_F;

  float accNorm = sqrtf(ax * ax + ay * ay + az * az);
  bool accelValid = accNorm > 0.3f;

  float q0 = gQuat0;
  float q1 = gQuat1;
  float q2 = gQuat2;
  float q3 = gQuat3;

  if (!gFusionSeeded) {
    if (accelValid) {
      float invAcc = 1.0f / accNorm;
      float axn = ax * invAcc;
      float ayn = ay * invAcc;
      float azn = az * invAcc;

      gRoll = atan2f(ayn, azn) * 57.29578f;
      gPitch = atan2f(-axn, sqrtf(ayn * ayn + azn * azn)) * 57.29578f;

      const float hr = gRoll * 0.00872664626f;
      const float hp = gPitch * 0.00872664626f;
      const float hy = 0.0f;
      const float cr = cosf(hr);
      const float sr = sinf(hr);
      const float cp = cosf(hp);
      const float sp = sinf(hp);
      const float cy = cosf(hy);
      const float sy = sinf(hy);
      q0 = cr * cp * cy + sr * sp * sy;
      q1 = sr * cp * cy - cr * sp * sy;
      q2 = cr * sp * cy + sr * cp * sy;
      q3 = cr * cp * sy - sr * sp * cy;
    }
    gFusionSeeded = true;
  }

  if (accelValid) {
    float invAcc = 1.0f / accNorm;
    float axn = ax * invAcc;
    float ayn = ay * invAcc;
    float azn = az * invAcc;

    float gravX = 2.0f * (q1 * q3 - q0 * q2);
    float gravY = 2.0f * (q0 * q1 + q2 * q3);
    float gravZ = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // Quaternion-first correction using gravity alignment error.
    float ex = (ayn * gravZ - azn * gravY);
    float ey = (azn * gravX - axn * gravZ);
    float ez = (axn * gravY - ayn * gravX);
    wx += QUAT_ACC_GAIN * ex;
    wy += QUAT_ACC_GAIN * ey;
    wz += QUAT_ACC_GAIN * ez;
  }

  // Integrate quaternion using corrected angular rate.
  float dq0 = 0.5f * (-q1 * wx - q2 * wy - q3 * wz);
  float dq1 = 0.5f * ( q0 * wx + q2 * wz - q3 * wy);
  float dq2 = 0.5f * ( q0 * wy - q1 * wz + q3 * wx);
  float dq3 = 0.5f * ( q0 * wz + q1 * wy - q2 * wx);

  q0 += dq0 * dt;
  q1 += dq1 * dt;
  q2 += dq2 * dt;
  q3 += dq3 * dt;

  float qNorm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  if (qNorm > 1e-6f) {
    q0 /= qNorm;
    q1 /= qNorm;
    q2 /= qNorm;
    q3 /= qNorm;
  } else {
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
  }

  gQuat0 = q0;
  gQuat1 = q1;
  gQuat2 = q2;
  gQuat3 = q3;

  updateEulerFromQuaternion();

  if (gHasMag) {
    float rollRad = gRoll * DEG_TO_RAD_F;
    float pitchRad = gPitch * DEG_TO_RAD_F;
    float mxComp = gMagX * cosf(pitchRad) + gMagZ * sinf(pitchRad);
    float myComp = gMagX * sinf(rollRad) * sinf(pitchRad) + gMagY * cosf(rollRad) - gMagZ * sinf(rollRad) * cosf(pitchRad);
    gHeading = normalize360(atan2f(myComp, mxComp) * 57.29578f);

    float yawErr = wrapAngle180(gHeading - gYaw);
    float absRate = sqrtf(gx * gx + gy * gy + gz * gz);
    float dynamicBlend = YAW_MAG_BLEND_MAX;
    if (absRate > 220.0f) {
      dynamicBlend = YAW_MAG_BLEND_MIN;
    } else if (absRate > 0.0f) {
      float t = absRate / 220.0f;
      dynamicBlend = YAW_MAG_BLEND_MAX - (YAW_MAG_BLEND_MAX - YAW_MAG_BLEND_MIN) * t;
    }
    float yawCorr = (dynamicBlend + QUAT_MAG_GAIN * 0.01f) * yawErr;

    float halfYaw = 0.5f * yawCorr * DEG_TO_RAD_F;
    float cq = cosf(halfYaw);
    float sq = sinf(halfYaw);
    float nq0 = q0 * cq - q3 * sq;
    float nq1 = q1 * cq - q2 * sq;
    float nq2 = q2 * cq + q1 * sq;
    float nq3 = q3 * cq + q0 * sq;

    float nNorm = sqrtf(nq0 * nq0 + nq1 * nq1 + nq2 * nq2 + nq3 * nq3);
    if (nNorm > 1e-6f) {
      gQuat0 = nq0 / nNorm;
      gQuat1 = nq1 / nNorm;
      gQuat2 = nq2 / nNorm;
      gQuat3 = nq3 / nNorm;
      updateEulerFromQuaternion();
    }
  } else {
    gHeading = gYaw;
  }
}

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
  static bool cpuLoadFilteredSeeded = false;
  static float cpuLoadFiltered = 0.0f;

  uint32_t nowMs = millis();
  if (cpuLastLoadSampleMs == 0) {
    cpuLastLoadSampleMs = nowMs;
    cpuPrevIdleHookCount = cpuIdleHookCount;
  }

  uint32_t elapsedMs = nowMs - cpuLastLoadSampleMs;
  if (elapsedMs < 1000U) {
    return gCpuLoad;
  }

  float loadOut = 0.0f;
  bool measured = false;
  if (getCpuLoadFromRuntimeStats(loadOut)) {
    measured = true;
  } else {
    uint32_t idleNow = cpuIdleHookCount;
    uint32_t idleDelta = idleNow - cpuPrevIdleHookCount;
    cpuPrevIdleHookCount = idleNow;

    float idleRate = (elapsedMs > 0U) ? ((float)idleDelta / (float)elapsedMs) : 0.0f;

    if (cpuIdleRateRef < 0.001f) {
      cpuIdleRateRef = idleRate;
      loadOut = 0.0f;
      measured = true;
    } else {
      if (idleRate > cpuIdleRateRef) {
        cpuIdleRateRef = idleRate;
      } else {
        // Very slow adaptation down to avoid drift from transient peaks.
        cpuIdleRateRef = cpuIdleRateRef * 0.999f + idleRate * 0.001f;
      }

      if (cpuIdleRateRef < 0.001f) {
        loadOut = 0.0f;
      } else {
        float idlePct = (100.0f * idleRate) / cpuIdleRateRef;
        loadOut = constrain(100.0f - idlePct, 0.0f, 100.0f);
      }
      measured = true;
    }
  }

  cpuLastLoadSampleMs = nowMs;
  if (measured) {
    float targetLoad = constrain(loadOut, 0.0f, 100.0f);
    if (!cpuLoadFilteredSeeded) {
      cpuLoadFiltered = targetLoad;
      cpuLoadFilteredSeeded = true;
    } else {
      float alpha = (targetLoad > cpuLoadFiltered) ? 0.35f : 0.20f;
      float next = cpuLoadFiltered + alpha * (targetLoad - cpuLoadFiltered);

      // Limit abrupt 1-second jumps caused by scheduler jitter.
      float maxStepPerSample = 3.0f;
      float step = next - cpuLoadFiltered;
      if (step > maxStepPerSample) step = maxStepPerSample;
      if (step < -maxStepPerSample) step = -maxStepPerSample;
      cpuLoadFiltered += step;
    }

    if (cpuLoadFiltered < 0.3f) cpuLoadFiltered = 0.0f;
    return constrain(cpuLoadFiltered, 0.0f, 100.0f);
  }
  return gCpuLoad;
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

// WebSocket handler removed - now using BLE notifications instead

// ================================================================
// ISRs (buttons only — sensor read uses millis() for portability)
// ================================================================
void IRAM_ATTR isrBtnNext() { isrNextFlag = true; }
void IRAM_ATTR isrBtnPrev() { isrPrevFlag = true; }
void IRAM_ATTR isrGyroDataReady() {
  if (gyroDataReadyCount < 1000) {
    gyroDataReadyCount++;
  }
}  // MPU data ready interrupt

// ================================================================
// THERMISTOR — ADC TO TEMPERATURE (Battery-Powered Divider)
// ================================================================
static float readThermistor() {
  uint16_t mvSamples[THERM_SAMPLE_COUNT];
  for (uint8_t i = 0; i < THERM_SAMPLE_COUNT; i++) {
    mvSamples[i] = (uint16_t)analogReadMilliVolts(THERM_PIN);
    delay(2); 
  }

  std::sort(mvSamples, mvSamples + THERM_SAMPLE_COUNT);
  const uint16_t medianMv = mvSamples[THERM_SAMPLE_COUNT / 2];

  uint32_t sumMv = 0;
  uint8_t kept = 0;
  for (uint8_t i = 0; i < THERM_SAMPLE_COUNT; i++) {
    const int32_t dMv = (int32_t)mvSamples[i] - (int32_t)medianMv;
    if (abs(dMv) <= (int32_t)THERM_TRIM_WINDOW_MV) {
      sumMv += mvSamples[i];
      kept++;
    }
  }

  const uint16_t filteredMv = (kept > 0) ? (uint16_t)(sumMv / kept) : medianMv;
  const float vAdc = (float)filteredMv / 1000.0f;
  
  // Update raw variable for MQTT (scaled to theoretical 3.3V max for backwards compatibility)
  gThermRaw = (uint16_t)((uint32_t)filteredMv * 4095U / 3300U);

  // =======================================================
  // DYNAMIC VOLTAGE COMPENSATION
  // Since the top resistor is wired to the battery, 
  // the divider's supply voltage IS the battery voltage.
  // =======================================================
  float vSupply = 3.3f; // Fallback in case INA219 fails

  if (hasINA219 && moduleCurrent && isfinite(gVoltage) && gVoltage > 2.0f) {
    vSupply = gVoltage; 
  }

  // Safety check to prevent division by zero or negative resistance
  if (vAdc <= 0.05f || vAdc >= (vSupply - 0.05f)) {
    gThermValid = false;
    return gThermFilterSeeded ? gThermFiltered : -99.0f;
  }

  // Calculate NTC resistance using the live battery voltage
  float rNtc = R_FIXED * (vAdc / (vSupply - vAdc));
  
  if (!isfinite(rNtc) || rNtc < 100.0f || rNtc > 1000000.0f) {
    gThermValid = false;
    return gThermFilterSeeded ? gThermFiltered : -99.0f;
  }

  float tempC = NAN;
  if (!thermistorTempFromResistance(rNtc, R_NOMINAL, tempC)) {
    gThermValid = false;
    return gThermFilterSeeded ? gThermFiltered : -99.0f;
  }

  if (!isfinite(tempC) || tempC < -40.0f || tempC > 125.0f) {
    gThermValid = false;
    return gThermFilterSeeded ? gThermFiltered : -99.0f;
  }

  // Apply baseline calibration
  tempC = tempC * THERM_CAL_GAIN + THERM_CAL_OFFSET_C;

  // =======================================================
  // SMOOTHING FILTER
  // =======================================================
  static uint8_t outlierCount = 0;
  if (gThermFilterSeeded) {
    // Reject sudden impossible jumps (e.g., >12 degrees in one second)
    if (fabsf(tempC - gThermFiltered) > 12.0f) {
      outlierCount++;
      if (outlierCount < 5) {
        gThermValid = true;
        return gThermFiltered;
      }
    }
    outlierCount = 0;
  }

  if (!gThermFilterSeeded) {
    gThermFiltered = tempC;
    gThermFilterSeeded = true;
  } else {
    const float deltaC = tempC - gThermFiltered;
    if (fabsf(deltaC) < THERM_FILTER_DEADBAND_C) {
      gThermFiltered = gThermFiltered + 0.05f * deltaC;
      gThermValid = true;
      return gThermFiltered;
    }

    const float alpha = (fabsf(deltaC) > THERM_FILTER_FAST_DELTA_C)
      ? THERM_FILTER_ALPHA_FAST
      : THERM_FILTER_ALPHA;
    gThermFiltered = gThermFiltered + alpha * deltaC;
  }

  gThermValid = true;
  return gThermFiltered;
}
static bool thermistorTempFromResistance(float rNtc, float r25, float& tempC) {
  const float t0K = T_NOMINAL + 273.15f;
  const float invTK = (1.0f / t0K) + (logf(rNtc / r25) / B_COEFF);
  if (!isfinite(invTK) || invTK <= 0.0f) return false;

  tempC = (1.0f / invTK) - 273.15f;
  if (!isfinite(tempC) || tempC < -40.0f || tempC > 125.0f) return false;
  return true;
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

static uint8_t bcdToBin(uint8_t v) {
  return (uint8_t)((v & 0x0F) + ((v >> 4) * 10));
}

static uint8_t binToBcd(uint8_t v) {
  return (uint8_t)(((v / 10) << 4) | (v % 10));
}

static bool rtcWriteDateTime(int year, int month, int day, int hour, int minute, int second) {
  if (!hasRTC || gRtcAddr == 0) return false;

  uint8_t yy = (uint8_t)constrain(year - 2000, 0, 99);
  uint8_t mo = (uint8_t)constrain(month, 1, 12);
  uint8_t dd = (uint8_t)constrain(day, 1, 31);
  uint8_t hh = (uint8_t)constrain(hour, 0, 23);
  uint8_t mm = (uint8_t)constrain(minute, 0, 59);
  uint8_t ss = (uint8_t)constrain(second, 0, 59);

  uint8_t regs[7];
  regs[0] = binToBcd(ss);
  regs[1] = binToBcd(mm);
  regs[2] = binToBcd(hh);
  regs[3] = 1;
  regs[4] = binToBcd(dd);
  regs[5] = binToBcd(mo);
  regs[6] = binToBcd(yy);

  Wire.beginTransmission(gRtcAddr);
  Wire.write((uint8_t)0x00);
  for (uint8_t i = 0; i < sizeof(regs); i++) {
    Wire.write(regs[i]);
  }
  return Wire.endTransmission() == 0;
}

static bool rtcReadDateTime(int& year, int& month, int& day, int& hour, int& minute, int& second) {
  if (!hasRTC || gRtcAddr == 0) return false;

  uint8_t regs[7] = {0};
  if (!i2cReadRegs(gRtcAddr, 0x00, regs, sizeof(regs))) return false;

  second = bcdToBin(regs[0] & 0x7F);
  minute = bcdToBin(regs[1] & 0x7F);
  hour = bcdToBin(regs[2] & 0x3F);
  day = bcdToBin(regs[4] & 0x3F);
  month = bcdToBin(regs[5] & 0x1F);
  year = 2000 + bcdToBin(regs[6]);

  if (year < 2020 || year > 2099) return false;
  if (month < 1 || month > 12) return false;
  if (day < 1 || day > 31) return false;
  if (hour > 23 || minute > 59 || second > 59) return false;
  return true;
}

static bool calibrateMpuGyroBias(uint8_t mpuAddr) {
  int32_t sx = 0;
  int32_t sy = 0;
  int32_t sz = 0;
  uint8_t buf[6] = {0};
  int16_t minX = INT16_MAX, minY = INT16_MAX, minZ = INT16_MAX;
  int16_t maxX = INT16_MIN, maxY = INT16_MIN, maxZ = INT16_MIN;

  for (uint8_t i = 0; i < MPU_BIAS_SAMPLES; i++) {
    if (!i2cReadRegs(mpuAddr, MPU_GYRO_XOUT_H, buf, sizeof(buf))) return false;
    int16_t x = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t y = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t z = (int16_t)((buf[4] << 8) | buf[5]);

    sx += x;
    sy += y;
    sz += z;

    if (x < minX) minX = x;
    if (y < minY) minY = y;
    if (z < minZ) minZ = z;
    if (x > maxX) maxX = x;
    if (y > maxY) maxY = y;
    if (z > maxZ) maxZ = z;
    delay(3);
  }

  const int16_t spanX = maxX - minX;
  const int16_t spanY = maxY - minY;
  const int16_t spanZ = maxZ - minZ;
  if (spanX > 1200 || spanY > 1200 || spanZ > 1200) {
    // Device moved during calibration; keep estimate and let runtime bias tracking refine it.
  }

  gGyroBiasX = (float)sx / MPU_BIAS_SAMPLES;
  gGyroBiasY = (float)sy / MPU_BIAS_SAMPLES;
  gGyroBiasZ = (float)sz / MPU_BIAS_SAMPLES;
  return true;
}

static bool initGyro() {
  uint8_t who = 0;

  // Try MPU at both valid addresses (AD0 selects 0x68/0x69).
  uint8_t mpuAddr = 0;
  if (i2cProbe(IMU_ADDR_MPU1) && i2cReadRegs(IMU_ADDR_MPU1, MPU_WHO_AM_I_REG, &who, 1)) {
    mpuAddr = IMU_ADDR_MPU1;
  } else if (i2cProbe(IMU_ADDR_MPU0) && i2cReadRegs(IMU_ADDR_MPU0, MPU_WHO_AM_I_REG, &who, 1)) {
    mpuAddr = IMU_ADDR_MPU0;
  }

  if (mpuAddr != 0) {
    if (who == MPU_WHO_AM_I_6500 || who == MPU_WHO_AM_I_9250) {
#if USE_MPU_DMP
      // DMP-first path based on provided MPU example.
      if (mpu.begin() == INV_SUCCESS) {
        mpu.enableInterrupt();
        mpu.setIntLevel(INT_ACTIVE_LOW);
        mpu.setIntLatched(INT_LATCHED);

        if (mpu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL, 10) == INV_SUCCESS) {
          mpu.dmpSetOrientation(orientationDefault);
          gImuAddr = mpuAddr;
          gImuIsMpu = true;
          gMpuDmpActive = true;
          gHasMag = false;
          gGyroFilterSeeded = false;
          gFusionSeeded = false;
          gLastFusionUs = 0;
          gQuat0 = 1.0f;
          gQuat1 = 0.0f;
          gQuat2 = 0.0f;
          gQuat3 = 0.0f;
          return true;
        }
      }
#endif

      // Safe fallback: existing register-based path.
      if (!i2cWriteReg(mpuAddr, MPU_PWR_MGMT_1, 0x01)) return false;  // wake up + auto clock source
      if (!i2cWriteReg(mpuAddr, MPU_PWR_MGMT_2, 0x00)) return false;  // enable all accel/gyro axes
      delay(20);
      if (!i2cWriteReg(mpuAddr, MPU_CONFIG, 0x04)) return false;       // DLPF ~21Hz
      if (!i2cWriteReg(mpuAddr, MPU_SMPLRT_DIV, 0x00)) return false;   // 200Hz sample rate
      if (!i2cWriteReg(mpuAddr, MPU_GYRO_CONFIG, 0x08)) return false;  // +-500 dps
      if (!i2cWriteReg(mpuAddr, MPU_ACCEL_CONFIG, 0x00)) return false; // +-2g
      // Configure interrupt: active high, push-pull + BYPASS_EN for AK8963 direct I2C access.
      if (!i2cWriteReg(mpuAddr, MPU_INT_PIN_CFG, 0x02)) return false;  // BYPASS_EN
      if (!i2cWriteReg(mpuAddr, MPU_INT_ENABLE, 0x01)) return false;   // enable data ready interrupt (bit 0)
      (void)calibrateMpuGyroBias(mpuAddr);

      // Optional magnetometer path (MPU9250 has AK8963 @ 0x0C, MPU6500 does not).
      gHasMag = false;
      uint8_t magWho = 0;
      if (i2cProbe(AK8963_ADDR) && i2cReadRegs(AK8963_ADDR, AK8963_WHO_AM_I, &magWho, 1) && magWho == AK8963_WHO_AM_I_VAL) {
        // Enter fuse ROM mode to read sensitivity adjustment values.
        if (i2cWriteReg(AK8963_ADDR, AK8963_CNTL1, 0x00)) {
          delay(10);
          if (i2cWriteReg(AK8963_ADDR, AK8963_CNTL1, 0x0F)) {
            delay(10);
            uint8_t asa[3] = {0};
            if (i2cReadRegs(AK8963_ADDR, AK8963_ASAX, asa, 3)) {
              gMagAdjX = ((float)asa[0] - 128.0f) / 256.0f + 1.0f;
              gMagAdjY = ((float)asa[1] - 128.0f) / 256.0f + 1.0f;
              gMagAdjZ = ((float)asa[2] - 128.0f) / 256.0f + 1.0f;
            }
          }
        }

        // Continuous measurement mode 2 (100Hz), 16-bit output.
        if (i2cWriteReg(AK8963_ADDR, AK8963_CNTL1, 0x00)) {
          delay(10);
          if (i2cWriteReg(AK8963_ADDR, AK8963_CNTL1, 0x16)) {
            delay(10);
            gHasMag = true;
          }
        }
      }

      gImuAddr = mpuAddr;
      gImuIsMpu = true;
      gMpuDmpActive = false;
      gGyroFilterSeeded = false;
      gFusionSeeded = false;
      gLastFusionUs = 0;
      return true;
    }
  }

  // Fallback to LSM6DS3 compatible at 0x6A.
  if (!i2cProbe(IMU_ADDR_LSM6)) return false;
  if (!i2cReadRegs(IMU_ADDR_LSM6, LSM_WHO_AM_I_REG, &who, 1)) return false;
  if (who != LSM_WHO_AM_I_VAL) return false;
  if (!i2cWriteReg(IMU_ADDR_LSM6, LSM_CTRL2_G, 0x4C)) return false;  // 104Hz, 2000 dps
  delay(10);
  gImuAddr = IMU_ADDR_LSM6;
  gImuIsMpu = false;
  gHasMag = false;
  gFusionSeeded = false;
  gLastFusionUs = 0;
  return true;
}

static bool readMagnetometer() {
  if (!gHasMag) return false;

  uint8_t st1 = 0;
  if (!i2cReadRegs(AK8963_ADDR, AK8963_ST1, &st1, 1)) return false;
  if ((st1 & 0x01) == 0) return false;  // data not ready

  uint8_t data[7] = {0};  // HXL..HZH + ST2
  if (!i2cReadRegs(AK8963_ADDR, AK8963_XOUT_L, data, 7)) return false;
  uint8_t st2 = data[6];
  if (st2 & 0x08) return false;  // magnetic sensor overflow

  gMagRawX = (int16_t)((data[1] << 8) | data[0]);
  gMagRawY = (int16_t)((data[3] << 8) | data[2]);
  gMagRawZ = (int16_t)((data[5] << 8) | data[4]);

  // Convert to uT-like scaled units (0.15 uT/LSB @16-bit), include factory adjustment.
  gMagX = gMagRawX * 0.15f * gMagAdjX;
  gMagY = gMagRawY * 0.15f * gMagAdjY;
  gMagZ = gMagRawZ * 0.15f * gMagAdjZ;
  return true;
}

static bool readGyro() {
  if (gImuAddr == 0) return false;

  if (gImuIsMpu) {
    if (gMpuDmpActive) {
      unsigned short fifoCnt = mpu.fifoAvailable();
      if (fifoCnt == 0) return false;

      inv_error_t result = mpu.dmpUpdateFifo();
      if (result != INV_SUCCESS) return false;

      mpu.computeEulerAngles();

      gQuat0 = mpu.calcQuat(mpu.qw);
      gQuat1 = mpu.calcQuat(mpu.qx);
      gQuat2 = mpu.calcQuat(mpu.qy);
      gQuat3 = mpu.calcQuat(mpu.qz);

      gGyroRawX = mpu.gx;
      gGyroRawY = mpu.gy;
      gGyroRawZ = mpu.gz;
      gAccelRawX = mpu.ax;
      gAccelRawY = mpu.ay;
      gAccelRawZ = mpu.az;

      float axMeas = mpu.calcAccel(mpu.ax);
      float ayMeas = mpu.calcAccel(mpu.ay);
      float azMeas = mpu.calcAccel(mpu.az);

      gGyroX = mpu.calcGyro(mpu.gx);
      gGyroY = mpu.calcGyro(mpu.gy);
      gGyroZ = mpu.calcGyro(mpu.gz);

      gRoll = mpu.roll;
      gPitch = mpu.pitch;
      gYaw = normalize360(mpu.yaw);
      gHeading = gYaw;
      gGyroAbs = sqrtf(gGyroX * gGyroX + gGyroY * gGyroY + gGyroZ * gGyroZ);

      uint32_t nowUs = micros();
      float dt = 0.01f;
      if (gLastMotionUs != 0) {
        dt = (nowUs - gLastMotionUs) * 1e-6f;
        if (dt < 0.001f) dt = 0.001f;
        if (dt > 0.05f) dt = 0.05f;
      }
      gLastMotionUs = nowUs;

      gAccelX = axMeas - gAccelBiasX;
      gAccelY = ayMeas - gAccelBiasY;
      gAccelZ = azMeas - gAccelBiasZ;

      updateMotionEstimate(dt);
      updateInertialBiases(gMotionStillCount >= ZUPT_STILL_REQUIRED_SAMPLES, axMeas, ayMeas, azMeas, gGyroX, gGyroY, gGyroZ);
      return true;
    }

    uint8_t buf[14] = {0};
    if (!i2cReadRegs(gImuAddr, MPU_ACCEL_XOUT_H, buf, sizeof(buf))) return false;

    gAccelRawX = (int16_t)((buf[0] << 8) | buf[1]);
    gAccelRawY = (int16_t)((buf[2] << 8) | buf[3]);
    gAccelRawZ = (int16_t)((buf[4] << 8) | buf[5]);
    gGyroRawX = (int16_t)((buf[8] << 8) | buf[9]);
    gGyroRawY = (int16_t)((buf[10] << 8) | buf[11]);
    gGyroRawZ = (int16_t)((buf[12] << 8) | buf[13]);

    float gx = ((float)gGyroRawX - gGyroBiasX) * MPU_GYRO_SCALE_DPS;
    float gy = ((float)gGyroRawY - gGyroBiasY) * MPU_GYRO_SCALE_DPS;
    float gz = ((float)gGyroRawZ - gGyroBiasZ) * MPU_GYRO_SCALE_DPS;

    // Update bias only while mostly still, otherwise movement corrupts zero reference.
    float absRate = sqrtf(gx * gx + gy * gy + gz * gz);
    if (absRate < GYRO_STILL_DPS) {
      if (gGyroStillCount < 255) gGyroStillCount++;
    } else {
      gGyroStillCount = 0;
    }

    float axMeas = gAccelRawX * MPU_ACCEL_SCALE_G;
    float ayMeas = gAccelRawY * MPU_ACCEL_SCALE_G;
    float azMeas = gAccelRawZ * MPU_ACCEL_SCALE_G;

    gAccelX = axMeas - gAccelBiasX;
    gAccelY = ayMeas - gAccelBiasY;
    gAccelZ = azMeas - gAccelBiasZ;

    if (!gGyroFilterSeeded) {
      gGyroX = gx;
      gGyroY = gy;
      gGyroZ = gz;
      gGyroFilterSeeded = true;
    } else {
      gGyroX += GYRO_FILTER_ALPHA * (gx - gGyroX);
      gGyroY += GYRO_FILTER_ALPHA * (gy - gGyroY);
      gGyroZ += GYRO_FILTER_ALPHA * (gz - gGyroZ);
    }

    if (fabsf(gGyroX) < GYRO_DEADBAND_DPS) gGyroX = 0.0f;
    if (fabsf(gGyroY) < GYRO_DEADBAND_DPS) gGyroY = 0.0f;
    if (fabsf(gGyroZ) < GYRO_DEADBAND_DPS) gGyroZ = 0.0f;

    // Magnetometer runs at up to 100Hz; read it at a lower duty than gyro.
    if (gHasMag && (millis() - gLastMagReadMs >= 10)) {
      (void)readMagnetometer();
      gLastMagReadMs = millis();
    }

    uint32_t nowUs = micros();
    float dt = 0.005f;
    if (gLastFusionUs != 0) {
      dt = (nowUs - gLastFusionUs) * 1e-6f;
      if (dt < 0.001f) dt = 0.001f;
      if (dt > 0.05f) dt = 0.05f;
    }
    gLastFusionUs = nowUs;
    updateOrientationFusion(dt, gx, gy, gz, gAccelX, gAccelY, gAccelZ);

    uint32_t motionNowUs = micros();
    float motionDt = 0.01f;
    if (gLastMotionUs != 0) {
      motionDt = (motionNowUs - gLastMotionUs) * 1e-6f;
      if (motionDt < 0.001f) motionDt = 0.001f;
      if (motionDt > 0.05f) motionDt = 0.05f;
    }
    gLastMotionUs = motionNowUs;
    updateMotionEstimate(motionDt);

    updateInertialBiases(gMotionStillCount >= ZUPT_STILL_REQUIRED_SAMPLES, axMeas, ayMeas, azMeas, gx, gy, gz);
  } else {
    uint8_t buf[6] = {0};
    if (!i2cReadRegs(gImuAddr, (uint8_t)(LSM_OUTX_L_G | 0x80), buf, sizeof(buf))) {
      return false;
    }

    gGyroRawX = (int16_t)((buf[1] << 8) | buf[0]);
    gGyroRawY = (int16_t)((buf[3] << 8) | buf[2]);
    gGyroRawZ = (int16_t)((buf[5] << 8) | buf[4]);
    gAccelRawX = gAccelRawY = gAccelRawZ = 0;

    gGyroX = gGyroRawX * LSM_GYRO_SCALE_DPS;
    gGyroY = gGyroRawY * LSM_GYRO_SCALE_DPS;
    gGyroZ = gGyroRawZ * LSM_GYRO_SCALE_DPS;
    gAccelX = gAccelY = gAccelZ = 0.0f;

    uint32_t nowUs = micros();
    float dt = 0.005f;
    if (gLastFusionUs != 0) {
      dt = (nowUs - gLastFusionUs) * 1e-6f;
      if (dt < 0.001f) dt = 0.001f;
      if (dt > 0.05f) dt = 0.05f;
    }
    gLastFusionUs = nowUs;
    updateOrientationFusion(dt, gGyroX, gGyroY, gGyroZ, 0.0f, 0.0f, 0.0f);

    uint32_t motionNowUs = micros();
    float motionDt = 0.01f;
    if (gLastMotionUs != 0) {
      motionDt = (motionNowUs - gLastMotionUs) * 1e-6f;
      if (motionDt < 0.001f) motionDt = 0.001f;
      if (motionDt > 0.05f) motionDt = 0.05f;
    }
    gLastMotionUs = motionNowUs;
    updateMotionEstimate(motionDt);
  }

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

static const char* i2cDeviceName(uint8_t addr) {
  switch (addr) {
    case 0x3C: return "SSD1306";
    case 0x0C: return "AK8963";
    case 0x40: return "INA219";
    case 0x50: return "AT24C32";
    case 0x60: return "RTC";
    case 0x68: return "DS1307";
    case 0x69: return "MPU6500";
    case 0x6A: return "LSM6DS3";
    default: return "Unknown";
  }
}

static void scanI2cDevices() {
  gI2cFoundCount = 0;
  gI2cScanOverflow = false;

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      if (gI2cFoundCount < I2C_SCAN_MAX_DEVICES) {
        gI2cFound[gI2cFoundCount++] = addr;
      } else {
        gI2cScanOverflow = true;
      }
    }
  }

  gLastI2cScanMs = millis();
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
// BLE SERVER CALLBACKS AND INITIALIZATION
// ================================================================

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    bleConnected = true;
    Serial.println("[BLE] Client connected");
  }

  void onDisconnect(BLEServer* pServer) {
    bleConnected = false;
    Serial.println("[BLE] Client disconnected");
    pServer->startAdvertising();  // Restart advertising
  }
};


// ---> PASTE THE NEW CLASS RIGHT HERE <---
class MyCharacteristicCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();
    if (rxValue.length() > 0) {
      Serial.printf("[BLE] Received %d bytes: ", (int)rxValue.length());
      
      // Convert to Arduino String for easy parsing
      String cmd = "";
      for (int i = 0; i < rxValue.length(); i++) {
        cmd += rxValue[i];
      }
      Serial.println(cmd);

      cmd.trim();
      cmd.toUpperCase();

      // Timer Controls via BLE
      if (cmd == "START") {
        if (!gTimerRunning) {
          gTimerStartMs = millis() - gTimerElapsedMs; // Resumes from paused time
          gTimerRunning = true;
          Serial.println("[TIMER] Started!");
        }
      } 
      else if (cmd == "STOP") {
        if (gTimerRunning) {
          gTimerElapsedMs = millis() - gTimerStartMs;
          gTimerRunning = false;
          Serial.println("[TIMER] Stopped!");
        }
      } 
      else if (cmd == "RESET") {
        gTimerRunning = false;
        gTimerElapsedMs = 0;
        Serial.println("[TIMER] Reset!");
      }
    }
  }
};

static void initBLE() {
  Serial.println("[BLE] Initializing BLE...");

  // Raise ATT MTU so JSON notifications are less likely to be truncated.
  BLEDevice::setMTU(247);
  
  // Initialize BLE device
  BLEDevice::init(BLE_DEVICE_NAME);
  
  // Create BLE server
  pBleServer = BLEDevice::createServer();
  pBleServer->setCallbacks(new MyServerCallbacks());
  
  // Create BLE service
  pService = pBleServer->createService(SERVICE_UUID);
  
  // Create BLE characteristics for sensor data
  pCharSensorData = pService->createCharacteristic(
    CHARACTERISTIC_NOTIFY_UUID,
    BLECharacteristic::PROPERTY_NOTIFY |
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_WRITE_NR
  );
  pCharSensorData->addDescriptor(new BLE2902());
  pCharSensorData->setCallbacks(new MyCharacteristicCallbacks());
  
  // Temperature characteristic
  pCharTemperature = pService->createCharacteristic(
    CHAR_TEMPERATURE_UUID,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
  );
  pCharTemperature->addDescriptor(new BLE2902());
  
  // Gyro characteristic
  pCharGyro = pService->createCharacteristic(
    CHAR_GYRO_UUID,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
  );
  pCharGyro->addDescriptor(new BLE2902());
  
  // Battery characteristic
  pCharBattery = pService->createCharacteristic(
    CHAR_BATTERY_UUID,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
  );
  pCharBattery->addDescriptor(new BLE2902());
  
  // IMU characteristic (custom UUID for full IMU data)
  pCharIMU = pService->createCharacteristic(
    CHARACTERISTIC_IMU_UUID,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
  );
  pCharIMU->addDescriptor(new BLE2902());
  
  // Start service
  pService->start();
  
  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();
  
  Serial.printf("[BLE] BLE Advertising started - Device Name: %s\n", BLE_DEVICE_NAME);
}

static void publishSensorDataBLE() {
  if (!bleConnected || !pCharSensorData) return;
  
  // Build JSON with sensor data
  char buf[384];
  JsonDocument doc;
  
  doc["ts"] = gTimestamp;
  doc["up"] = gUptime;
  
  if (moduleTemp && gThermValid) {
    float ntcTemp = roundf(gTemp * 10.0f) / 10.0f;
    doc["temp"] = ntcTemp;
  }
  
  if (moduleGyro && hasGyro) {
    doc["gx"] = roundf(gGyroX * 10.0f) / 10.0f;
    doc["gy"] = roundf(gGyroY * 10.0f) / 10.0f;
    doc["gz"] = roundf(gGyroZ * 10.0f) / 10.0f;

    // Mirror orientation into the main stream so apps that only subscribe
    // to a0000001 still receive quaternion and Euler angles.
    float q0 = isfinite(gQuat0) ? gQuat0 : 1.0f;
    float q1 = isfinite(gQuat1) ? gQuat1 : 0.0f;
    float q2 = isfinite(gQuat2) ? gQuat2 : 0.0f;
    float q3 = isfinite(gQuat3) ? gQuat3 : 0.0f;
    float qn = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (qn > 1e-6f) {
      q0 /= qn;
      q1 /= qn;
      q2 /= qn;
      q3 /= qn;
    } else {
      q0 = 1.0f;
      q1 = 0.0f;
      q2 = 0.0f;
      q3 = 0.0f;
    }

    doc["q0"] = roundf(q0 * 10000.0f) / 10000.0f;
    doc["q1"] = roundf(q1 * 10000.0f) / 10000.0f;
    doc["q2"] = roundf(q2 * 10000.0f) / 10000.0f;
    doc["q3"] = roundf(q3 * 10000.0f) / 10000.0f;
    doc["roll"] = roundf(gRoll * 100.0f) / 100.0f;
    doc["pitch"] = roundf(gPitch * 100.0f) / 100.0f;
    doc["yaw"] = roundf(gYaw * 100.0f) / 100.0f;
    doc["stationary"] = (gMotionStillCount >= ZUPT_STILL_REQUIRED_SAMPLES);
    doc["imu_seq"] = gBleImuNotifyCount;
  }
  
  if (moduleCurrent) {
    doc["v"] = roundf(gVoltage * 100.0f) / 100.0f;
    doc["ma"] = roundf(gCurrentMA * 10.0f) / 10.0f;
    doc["batt"] = roundf(gBattPct * 10.0f) / 10.0f;
  }

  if (moduleCPU) {
    doc["cpu"] = roundf(gCpuLoad * 10.0f) / 10.0f;
  }
  
  size_t needed = measureJson(doc);
  if (needed >= sizeof(buf)) {
    Serial.printf("[BLE] Sensor payload too large: needed=%u, max=%u (drop)\n",
                  (unsigned)needed,
                  (unsigned)(sizeof(buf) - 1));
    return;
  }

  size_t len = serializeJson(doc, buf, sizeof(buf));
  if (len == needed) {
    pCharSensorData->setValue((uint8_t*)buf, len);
    pCharSensorData->notify();
    gBleSensorNotifyCount++;

    // Mirror selected values to standard characteristics for generic BLE apps.
    if (pCharTemperature && moduleTemp && gThermValid) {
      char tbuf[16];
      int tlen = snprintf(tbuf, sizeof(tbuf), "%.1f", gTemp);
      if (tlen > 0) {
        pCharTemperature->setValue((uint8_t*)tbuf, (size_t)tlen);
        pCharTemperature->notify();
      }
    }

    if (pCharBattery) {
      uint8_t batt = (uint8_t)constrain((int)roundf(gBattPct), 0, 100);
      pCharBattery->setValue(&batt, 1);
      pCharBattery->notify();
    }

    if (pCharGyro && moduleGyro && hasGyro) {
      float gabs = sqrtf(gGyroX * gGyroX + gGyroY * gGyroY + gGyroZ * gGyroZ);
      char gbuf[16];
      int glen = snprintf(gbuf, sizeof(gbuf), "%.1f", gabs);
      if (glen > 0) {
        pCharGyro->setValue((uint8_t*)gbuf, (size_t)glen);
        pCharGyro->notify();
      }
    }

    if ((gBleSensorNotifyCount % 10) == 0) {
      Serial.printf("[BLE] Sensor notify #%lu, len=%u\n",
                    (unsigned long)gBleSensorNotifyCount,
                    (unsigned)len);
    }
  } else {
    Serial.printf("[BLE] Sensor serialize mismatch: len=%u needed=%u (drop)\n",
                  (unsigned)len,
                  (unsigned)needed);
  }
}

static void publishIMUDataBLE() {
  if (!bleConnected) return;
  if (!pCharIMU) {
    uint32_t now = millis();
    if (now - gBleImuSkipLogMs >= 3000) {
      gBleImuSkipLogMs = now;
      Serial.println("[BLE] IMU notify skipped: pCharIMU is null");
    }
    return;
  }
  
  char buf[512];
  JsonDocument doc;
  
  float q0 = isfinite(gQuat0) ? gQuat0 : 1.0f;
  float q1 = isfinite(gQuat1) ? gQuat1 : 0.0f;
  float q2 = isfinite(gQuat2) ? gQuat2 : 0.0f;
  float q3 = isfinite(gQuat3) ? gQuat3 : 0.0f;
  
  float qn = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  if (qn > 1e-6f) {
    q0 /= qn; q1 /= qn; q2 /= qn; q3 /= qn;
  } else {
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
  }
  
  doc["up"] = gUptime;
  doc["q0"] = roundf(q0 * 10000.0f) / 10000.0f;
  doc["q1"] = roundf(q1 * 10000.0f) / 10000.0f;
  doc["q2"] = roundf(q2 * 10000.0f) / 10000.0f;
  doc["q3"] = roundf(q3 * 10000.0f) / 10000.0f;
  doc["roll"] = roundf(gRoll * 100.0f) / 100.0f;
  doc["pitch"] = roundf(gPitch * 100.0f) / 100.0f;
  doc["yaw"] = roundf(gYaw * 100.0f) / 100.0f;
  doc["stationary"] = (gMotionStillCount >= ZUPT_STILL_REQUIRED_SAMPLES);
  
  size_t len = serializeJson(doc, buf, sizeof(buf));
  if (len < sizeof(buf)) {
    pCharIMU->setValue((uint8_t*)buf, len);
    pCharIMU->notify();
    gBleImuNotifyCount++;
    if ((gBleImuNotifyCount % 20) == 0) {
      Serial.printf("[BLE] IMU notify #%lu, len=%u\n",
                    (unsigned long)gBleImuNotifyCount,
                    (unsigned)len);
    }
  }
}


// ================================================================
// READ ALL SENSORS
// ================================================================
static void readOtherSensors() {
  // --- Time source priority: NTP/system -> RTC -> uptime fallback ---
  if (systemTimeValid()) {
    struct tm ti;
    if (getLocalTime(&ti, 50)) {
      snprintf(gTimestamp, sizeof(gTimestamp), "%04d-%02d-%02d %02d:%02d:%02d",
               ti.tm_year + 1900, ti.tm_mon + 1, ti.tm_mday,
               ti.tm_hour, ti.tm_min, ti.tm_sec);
    }
  } else if (hasRTC) {
    int year = 0;
    int month = 0;
    int day = 0;
    int hour = 0;
    int minute = 0;
    int second = 0;
    if (rtcReadDateTime(year, month, day, hour, minute, second)) {
      snprintf(gTimestamp, sizeof(gTimestamp), "%04d-%02d-%02d %02d:%02d:%02d",
               year, month, day, hour, minute, second);
    } else {
      uint32_t s = millis() / 1000;
      uint32_t hh = (s / 3600) % 24;
      uint32_t mm = (s / 60) % 60;
      uint32_t ss = s % 60;
      snprintf(gTimestamp, sizeof(gTimestamp), "1970-01-01 %02lu:%02lu:%02lu",
               (unsigned long)hh, (unsigned long)mm, (unsigned long)ss);
    }
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
    
    // Small +/- current around zero is usually INA noise.
    if (fabsf(gCurrentMA) < BATT_CURRENT_DEADBAND_MA) {
      gCurrentMA = 0.0f;
    }
  } else if (!moduleCurrent) {
    // Module OFF: stop current/power processing and battery integration.
    gCurrentMA = 0.0f;
    gPowerMW = 0.0f;
    gBattLife = -1.0f;
    gBatteryPresent = false;
  }

  if (moduleTemp) {
    gTemp = readThermistor();
    if (!gThermValid) {
      appendOledLog("TH ERR GPIO%u", THERM_PIN);
    }
  }

  // NOTE: Gyroscope is now read via interrupt (see isrGyroDataReady)
  // Do not read it here - it's handled in the main loop on gyroDataReadyFlag

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
    // Debounced battery-presence detection with hysteresis to avoid transient resets.
    if (gVoltage <= BATT_PRESENT_OFF_V) {
      gBattLowAccumMs += dt_ms;
    } else {
      gBattLowAccumMs = 0;
    }

    if (gVoltage >= BATT_PRESENT_ON_V) {
      gBattHighAccumMs += dt_ms;
    } else {
      gBattHighAccumMs = 0;
    }

    if (gBatteryPresent) {
      if (gBattLowAccumMs >= BATT_PRESENCE_DEBOUNCE_MS) {
        gBatteryPresent = false;
      }
    } else {
      if (gBattHighAccumMs >= BATT_PRESENCE_DEBOUNCE_MS) {
        gBatteryPresent = true;
      }
    }
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

    // Base OCV trust by operating mode (more conservative under load).
    float ocvWeight = 0.05f;
    if (nearRest) ocvWeight = 0.15f;
    if (longRest) ocvWeight = 0.35f;

    // Recover quickly if persisted/current counter state disagrees strongly with voltage.
    if (mismatchPct >= BATT_OCV_MISMATCH_RECOVER_PCT) {
      ocvWeight = max(ocvWeight, longRest ? 0.50f : 0.25f);
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
    // Keep last SOC/coulomb estimate; do not wipe persistent state on transient disconnects.
    gBattLife = -1.0f;
  }

  // --- WiFi --- (Removed - now BLE-only)
  gRSSI = 0;  // Not applicable for BLE

  // --- CPU load ---
  gCpuLoad = measureCpuLoadPct();

  // --- OLED log feed uses same raw sources ---
  updateOledRawLog();
}

// ================================================================
// BUILD JSON — compact, only real data
// ================================================================
static size_t buildJSON(char* buf, size_t bufSize) {
  JsonDocument doc;

  doc["ts"]   = gTimestamp;
  doc["up"]   = gUptime;
  if (moduleTemp && gThermValid) {
    float ntcTemp = roundf(gTemp * 10.0f) / 10.0f;
    doc["temp"] = ntcTemp;
    doc["temperature"] = ntcTemp;  // Alias for web clients expecting "temperature"
    doc["ntc_c"] = ntcTemp;
  } else {
    doc["temp"] = nullptr;
    doc["temperature"] = nullptr;
    doc["ntc_c"] = nullptr;
  }
  doc["temp_raw"] = (moduleTemp && gThermValid) ? gThermRaw : 0;
  doc["ntc_raw"] = (moduleTemp && gThermValid) ? gThermRaw : 0;

  if (moduleGyro && hasGyro) {
    doc["gx"] = roundf(gGyroX * 10.0f) / 10.0f;
    doc["gy"] = roundf(gGyroY * 10.0f) / 10.0f;
    doc["gz"] = roundf(gGyroZ * 10.0f) / 10.0f;
    doc["gabs"] = roundf(gGyroAbs * 10.0f) / 10.0f;
    doc["ax"] = roundf(gAccelX * 1000.0f) / 1000.0f;
    doc["ay"] = roundf(gAccelY * 1000.0f) / 1000.0f;
    doc["az"] = roundf(gAccelZ * 1000.0f) / 1000.0f;
    doc["roll"] = roundf(gRoll * 100.0f) / 100.0f;
    doc["pitch"] = roundf(gPitch * 100.0f) / 100.0f;
    doc["yaw"] = roundf(gYaw * 100.0f) / 100.0f;
    doc["heading"] = roundf(gHeading * 100.0f) / 100.0f;
    doc["q0"] = roundf(gQuat0 * 10000.0f) / 10000.0f;
    doc["q1"] = roundf(gQuat1 * 10000.0f) / 10000.0f;
    doc["q2"] = roundf(gQuat2 * 10000.0f) / 10000.0f;
    doc["q3"] = roundf(gQuat3 * 10000.0f) / 10000.0f;
    if (gHasMag) {
      doc["mx"] = roundf(gMagX * 10.0f) / 10.0f;
      doc["my"] = roundf(gMagY * 10.0f) / 10.0f;
      doc["mz"] = roundf(gMagZ * 10.0f) / 10.0f;
    } else {
      doc["mx"] = nullptr;
      doc["my"] = nullptr;
      doc["mz"] = nullptr;
    }
    doc["mag_present"] = gHasMag;
    doc["imu_addr"] = gImuAddr;
  } else {
    doc["gx"] = nullptr;
    doc["gy"] = nullptr;
    doc["gz"] = nullptr;
    doc["gabs"] = nullptr;
    doc["ax"] = nullptr;
    doc["ay"] = nullptr;
    doc["az"] = nullptr;
    doc["roll"] = nullptr;
    doc["pitch"] = nullptr;
    doc["yaw"] = nullptr;
    doc["heading"] = nullptr;
    doc["q0"] = nullptr;
    doc["q1"] = nullptr;
    doc["q2"] = nullptr;
    doc["q3"] = nullptr;
    doc["mx"] = nullptr;
    doc["my"] = nullptr;
    doc["mz"] = nullptr;
    doc["mag_present"] = false;
    doc["imu_addr"] = nullptr;
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
  
  // BLE connection status instead of WiFi
  doc["ble_connected"] = bleConnected;
  doc["device_name"] = BLE_DEVICE_NAME;

  return serializeJson(doc, buf, bufSize);
}

static size_t buildImuJSON(char* buf, size_t bufSize) {
  JsonDocument doc;
  float q0 = isfinite(gQuat0) ? gQuat0 : 1.0f;
  float q1 = isfinite(gQuat1) ? gQuat1 : 0.0f;
  float q2 = isfinite(gQuat2) ? gQuat2 : 0.0f;
  float q3 = isfinite(gQuat3) ? gQuat3 : 0.0f;

  float qn = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  if (qn > 1e-6f) {
    q0 /= qn;
    q1 /= qn;
    q2 /= qn;
    q3 /= qn;
  } else {
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
  }

  doc["up"] = gUptime;
  doc["q0"] = roundf(q0 * 10000.0f) / 10000.0f;
  doc["q1"] = roundf(q1 * 10000.0f) / 10000.0f;
  doc["q2"] = roundf(q2 * 10000.0f) / 10000.0f;
  doc["q3"] = roundf(q3 * 10000.0f) / 10000.0f;
  doc["roll"] = roundf(gRoll * 100.0f) / 100.0f;
  doc["pitch"] = roundf(gPitch * 100.0f) / 100.0f;
  doc["yaw"] = roundf(gYaw * 100.0f) / 100.0f;
  doc["gx"] = roundf(gGyroX * 10.0f) / 10.0f;
  doc["gy"] = roundf(gGyroY * 10.0f) / 10.0f;
  doc["gz"] = roundf(gGyroZ * 10.0f) / 10.0f;
  doc["ax"] = roundf(gAccelX * 1000.0f) / 1000.0f;
  doc["ay"] = roundf(gAccelY * 1000.0f) / 1000.0f;
  doc["az"] = roundf(gAccelZ * 1000.0f) / 1000.0f;
  doc["gravity_x"] = roundf(gGravityX * 1000.0f) / 1000.0f;
  doc["gravity_y"] = roundf(gGravityY * 1000.0f) / 1000.0f;
  doc["gravity_z"] = roundf(gGravityZ * 1000.0f) / 1000.0f;
  doc["gyro_bias_x"] = roundf(gGyroBiasX * 10.0f) / 10.0f;
  doc["gyro_bias_y"] = roundf(gGyroBiasY * 10.0f) / 10.0f;
  doc["gyro_bias_z"] = roundf(gGyroBiasZ * 10.0f) / 10.0f;
  doc["acc_bias_x"] = roundf(gAccelBiasX * 1000.0f) / 1000.0f;
  doc["acc_bias_y"] = roundf(gAccelBiasY * 1000.0f) / 1000.0f;
  doc["acc_bias_z"] = roundf(gAccelBiasZ * 1000.0f) / 1000.0f;

  // Optional relative motion fields (can drift, use for visual effect only).
  doc["lin_ax"] = roundf(gLinAccX * 1000.0f) / 1000.0f;
  doc["lin_ay"] = roundf(gLinAccY * 1000.0f) / 1000.0f;
  doc["lin_az"] = roundf(gLinAccZ * 1000.0f) / 1000.0f;
  doc["vel_x"] = roundf(gVelX * 1000.0f) / 1000.0f;
  doc["vel_y"] = roundf(gVelY * 1000.0f) / 1000.0f;
  doc["vel_z"] = roundf(gVelZ * 1000.0f) / 1000.0f;
  doc["pos_x"] = roundf(gPosX * 1000.0f) / 1000.0f;
  doc["pos_y"] = roundf(gPosY * 1000.0f) / 1000.0f;
  doc["pos_z"] = roundf(gPosZ * 1000.0f) / 1000.0f;
  doc["dt_ms"] = roundf(gMotionDtMs * 100.0f) / 100.0f;
  doc["stationary"] = (gMotionStillCount >= ZUPT_STILL_REQUIRED_SAMPLES);
  doc["zupt"] = (gMotionStillCount >= ZUPT_STILL_REQUIRED_SAMPLES);
  doc["mode"] = gMpuDmpActive ? "mpu_dmp" : "fusion";
  doc["frame"] = "right-handed";
  doc["axes"] = "x:right,y:forward,z:up";
  doc["angles"] = "degrees";
  doc["quat_order"] = "wxyz";
  return serializeJson(doc, buf, bufSize);
}

// ================================================================
// BUILD RAW JSON — exact GPIO + raw sensor values
// ================================================================
static size_t buildRawGpioJSON(char* buf, size_t bufSize) {
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

  doc["therm_raw"] = (moduleTemp && gThermValid) ? gThermRaw : 0;

  JsonObject gyroRaw = doc["gyro_raw"].to<JsonObject>();
  if (moduleGyro && hasGyro) {
    gyroRaw["x"] = gGyroRawX;
    gyroRaw["y"] = gGyroRawY;
    gyroRaw["z"] = gGyroRawZ;
    gyroRaw["addr"] = gImuAddr;
  } else {
    gyroRaw["x"] = nullptr;
    gyroRaw["y"] = nullptr;
    gyroRaw["z"] = nullptr;
    gyroRaw["addr"] = nullptr;
  }

  JsonObject accelRaw = doc["accel_raw"].to<JsonObject>();
  if (moduleGyro && hasGyro) {
    accelRaw["x"] = gAccelRawX;
    accelRaw["y"] = gAccelRawY;
    accelRaw["z"] = gAccelRawZ;
  } else {
    accelRaw["x"] = nullptr;
    accelRaw["y"] = nullptr;
    accelRaw["z"] = nullptr;
  }

  JsonObject orient = doc["orientation"].to<JsonObject>();
  if (moduleGyro && hasGyro) {
    orient["roll"] = roundf(gRoll * 100.0f) / 100.0f;
    orient["pitch"] = roundf(gPitch * 100.0f) / 100.0f;
    orient["yaw"] = roundf(gYaw * 100.0f) / 100.0f;
    orient["heading"] = roundf(gHeading * 100.0f) / 100.0f;
    orient["q0"] = roundf(gQuat0 * 10000.0f) / 10000.0f;
    orient["q1"] = roundf(gQuat1 * 10000.0f) / 10000.0f;
    orient["q2"] = roundf(gQuat2 * 10000.0f) / 10000.0f;
    orient["q3"] = roundf(gQuat3 * 10000.0f) / 10000.0f;
  } else {
    orient["roll"] = nullptr;
    orient["pitch"] = nullptr;
    orient["yaw"] = nullptr;
    orient["heading"] = nullptr;
    orient["q0"] = nullptr;
    orient["q1"] = nullptr;
    orient["q2"] = nullptr;
    orient["q3"] = nullptr;
  }

  JsonObject magRaw = doc["mag_raw"].to<JsonObject>();
  if (moduleGyro && hasGyro && gHasMag) {
    magRaw["x"] = gMagRawX;
    magRaw["y"] = gMagRawY;
    magRaw["z"] = gMagRawZ;
    magRaw["present"] = true;
  } else {
    magRaw["x"] = nullptr;
    magRaw["y"] = nullptr;
    magRaw["z"] = nullptr;
    magRaw["present"] = false;
  }

  JsonObject inaRaw = doc["ina219_raw"].to<JsonObject>();
  if (moduleCurrent) inaRaw["bus"] = gInaBusRaw;
  else               inaRaw["bus"] = nullptr;
  if (moduleCurrent) inaRaw["current"] = gInaCurrRaw;
  else               inaRaw["current"] = nullptr;
  if (moduleCurrent) inaRaw["power"] = gInaPowRaw;
  else               inaRaw["power"] = nullptr;

  return serializeJson(doc, buf, bufSize);
}

// ================================================================
// MQTT CONNECT
// ================================================================
// ================================================================
// BLE PUBLISH (replaces MQTT)
// ================================================================
// Note: actual BLE publishing is handled by publishSensorDataBLE() and publishIMUDataBLE()
// which are called from the loop

static void publishState() {
  // In BLE mode, module state is communicated via BLE characteristics
  Serial.println("[BLE] Module state updated");
}

// ================================================================
// BUTTON HANDLING (ISR-driven, debounced)
// ================================================================
static void handleButtons() {
  uint32_t now = millis();

  if ((int32_t)(gIgnoreButtonsUntilMs - now) > 0) {
    isrNextFlag = false;
    isrPrevFlag = false;
    return;
  }

  static uint32_t lastBtnN = 0;
  static uint32_t lastBtnP = 0;

  if (isrNextFlag) {
    isrNextFlag = false;
    isrPrevFlag = false;
    if (now - lastBtnN > DEBOUNCE_MS) {
      lastBtnN = now;
      gIgnoreButtonsUntilMs = now + DEBOUNCE_MS;
      page = (page + 1) % NUM_PAGES;
      drawPage();
      return;
    }
  }

  if (isrPrevFlag) {
    isrPrevFlag = false;
    isrNextFlag = false;
    if (now - lastBtnP > DEBOUNCE_MS) {
      lastBtnP = now;
      gIgnoreButtonsUntilMs = now + DEBOUNCE_MS;
      page = (page - 1 + NUM_PAGES) % NUM_PAGES;
      drawPage();
      return;
    }
  }
}
// ================================================================
// OLED UTILITIES (HUD Redesign)
// ================================================================
static void drawTitleBar(const char* title) {
  // Inverted heavy header for clear page separation
  oled.fillRect(0, 0, 128, 14, SSD1306_WHITE);
  oled.setTextColor(SSD1306_BLACK);
  oled.setTextSize(1);
  
  // If disconnected, flash a warning every second (except on page 3 where it's already obvious)
  if (!bleConnected && page != 3 && (millis() / 1000) % 2 == 0) {
    int wx = (128 - 13 * 6) / 2; // "! LINK LOST !" is 13 chars
    oled.setCursor(max(2, wx), 3);
    oled.print("! LINK LOST !");
  } else {
    // Normal title
    int x = (128 - (int)strlen(title) * 6) / 2;
    oled.setCursor(max(2, x), 3);
    oled.print(title);
  }

  // Page indicator in top right
  oled.setCursor(104, 3);
  oled.printf("%d/%d", page + 1, NUM_PAGES);
  
  oled.setTextColor(SSD1306_WHITE); // Reset for rest of screen
}
// --- Custom UI Graphics ---

static void drawDynamicBattery(int x, int y, int w, int h, float pct) {
  pct = constrain(pct, 0.0f, 100.0f);
  oled.drawRect(x, y, w - 2, h, SSD1306_WHITE); // Main body
  oled.fillRect(x + w - 2, y + h / 4, 2, h / 2, SSD1306_WHITE); // Positive terminal nub
  int fillW = (int)((pct / 100.0f) * (w - 6));
  if (fillW > 0) {
    oled.fillRect(x + 2, y + 2, fillW, h - 4, SSD1306_WHITE);
  }
}

static void drawBluetoothLogo(int x, int y) {
  // Classic Bluetooth rune drawn with lines
  oled.drawLine(x + 4, y, x + 4, y + 12, SSD1306_WHITE);     // Center vertical
  oled.drawLine(x + 4, y, x + 8, y + 3, SSD1306_WHITE);      // Top right triangle
  oled.drawLine(x + 8, y + 3, x, y + 9, SSD1306_WHITE);      // Cross down-left
  oled.drawLine(x + 4, y + 12, x + 8, y + 9, SSD1306_WHITE); // Bottom right triangle
  oled.drawLine(x + 8, y + 9, x, y + 3, SSD1306_WHITE);      // Cross up-left
}

static void drawThermometerIcon(int x, int y, float pct) {
  pct = constrain(pct, 0.0f, 100.0f);
  oled.drawRoundRect(x, y, 6, 24, 3, SSD1306_WHITE); // Glass tube
  oled.fillCircle(x + 2, y + 26, 5, SSD1306_WHITE);  // Bulb
  int fillH = (int)(pct / 100.0f * 18.0f);
  if (fillH > 0) {
    oled.fillRect(x + 2, y + 22 - fillH, 2, fillH, SSD1306_WHITE); // Mercury level
  }
}

static void drawArtificialHorizon(int cx, int cy, int r, float pitch, float roll) {
  oled.drawCircle(cx, cy, r, SSD1306_WHITE); // Gauge bezel
  
  float rad = roll * 0.0174533f; // Convert to radians
  float s = sin(rad);
  float c = cos(rad);
  
  // Offset the line based on pitch (clamped to keep inside gauge)
  int p_off = constrain((int)(pitch * 0.5f), -(r - 2), (r - 2));
  
  int x1 = cx - (r - 2) * c + p_off * s;
  int y1 = cy - (r - 2) * s - p_off * c;
  int x2 = cx + (r - 2) * c + p_off * s;
  int y2 = cy + (r - 2) * s - p_off * c;
  
  oled.drawLine(x1, y1, x2, y2, SSD1306_WHITE); // Horizon line
  oled.drawPixel(cx, cy, SSD1306_WHITE);        // Center reticle
}

static void drawCentered(const char* text, int y, int sz) {
  oled.setTextSize(sz);
  int x = (128 - (int)strlen(text) * 6 * sz) / 2;
  oled.setCursor(max(0, x), y);
  oled.print(text);
}

// ================================================================
// OLED PAGES
// ================================================================
static void pageDashboard() {
  drawTitleBar("DASHBOARD HUD");

  // Strict HUD division lines (Prevents text crossover)
  oled.drawLine(60, 14, 60, 64, SSD1306_WHITE); // Vertical split
  oled.drawLine(60, 39, 128, 39, SSD1306_WHITE); // Right-side horizontal split

  // ==========================================
  // LEFT PANEL: Huge Temperature
  // ==========================================
  oled.setTextSize(1);
  oled.setCursor(4, 18); 
  oled.print("TEMP C");
  
  oled.setTextSize(2);
  char tBuf[8]; 
  snprintf(tBuf, sizeof(tBuf), "%.1f", gTemp);
  // Auto-center the temperature in the 60px wide left box
  int tx = (60 - strlen(tBuf) * 12) / 2;
  oled.setCursor(max(2, tx), 36); 
  oled.print(tBuf);

  // ==========================================
  // TOP RIGHT PANEL: Battery & Status
  // ==========================================
  drawDynamicBattery(65, 18, 20, 8, gBattPct);
  oled.setTextSize(1);
  oled.setCursor(89, 18); 
  oled.printf("%.0f%%", gBattPct);
  
  oled.setCursor(65, 29); 
  if (gCurrentMA > 10.0f) oled.print("DISCHRG");
  else if (gCurrentMA < -10.0f) oled.print("CHARGING");
  else oled.print("STABLE");

  // ==========================================
  // BOTTOM RIGHT PANEL: Comms & CPU
  // ==========================================
  drawBluetoothLogo(65, 46);
  oled.setCursor(76, 44); 
  oled.print(bleConnected ? "SYNCED" : "SEARCH");
  
  oled.setCursor(76, 54); 
  oled.printf("CPU:%d%%", (int)gCpuLoad);
}
static void pageRTC() {
  drawTitleBar("MISSION TIMER");

  // ==========================================
  // CALCULARE TIMP LIVE
  // ==========================================
  uint32_t currentElapsed = gTimerElapsedMs;
  if (gTimerRunning) {
    currentElapsed = millis() - gTimerStartMs;
  }

  uint32_t totalSec = currentElapsed / 1000;
  uint32_t m = totalSec / 60;
  uint32_t s = totalSec % 60;
  uint32_t ms10 = (currentElapsed % 1000) / 100; // Zecimi de secundă

  // ==========================================
  // MAIN HUD: Stopwatch Split-Typography
  // ==========================================
  char mainTime[6];
  snprintf(mainTime, sizeof(mainTime), "%02lu:%02lu", m, s);
  
  char decTime[3];
  snprintf(decTime, sizeof(decTime), ".%lu", ms10);

  // Calcul lățime: main = 5 caractere @ Size 3 (90px), dec = 2 caractere @ Size 2 (24px)
  // Lățime totală = 114px. Spațiu centrare = (128 - 114) / 2 = 7px.
  int tx = 7;
  int ty = 22; // Poziția pe axa Y

  // Printează Minutele și Secundele URIAȘ
  oled.setTextSize(3);
  oled.setCursor(tx, ty);
  oled.print(mainTime);

  // Printează Zecimile de secundă mai mici, aliniate jos
  oled.setTextSize(2);
  oled.setCursor(tx + 90, ty + 7); // Coborât 7 pixeli pentru aliniere perfectă la bază
  oled.print(decTime);

  // ==========================================
  // INDICATOR VIZUAL DE STARE (HUD ICONS)
  // ==========================================
  if (gTimerRunning) {
    // Punct care clipește la jumătate de secundă
    if ((millis() / 500) % 2 == 0) {
      oled.fillCircle(120, 26, 3, SSD1306_WHITE); 
    }
  } else if (!gTimerRunning && currentElapsed > 0) {
    // Iconiță de PAUZĂ (două bare verticale)
    oled.fillRect(117, 24, 2, 6, SSD1306_WHITE);
    oled.fillRect(121, 24, 2, 6, SSD1306_WHITE);
  }

  // ==========================================
  // BOTTOM BAR: Ora locală & Status
  // ==========================================
  oled.drawLine(0, 48, 128, 48, SSD1306_WHITE); // Linie separatoare elegantă

  oled.setTextSize(1);
  
  // Stânga: Status text
  oled.setCursor(2, 54);
  if (gTimerRunning) {
    oled.print("ACTIVE");
  } else if (currentElapsed == 0) {
    oled.print("READY");
  } else {
    oled.print("PAUSED");
  }

  // Dreapta: Ora locală (Informație secundară)
  char timeStr[9] = "--:--:--";
  if (strlen(gTimestamp) >= 19) {
    memcpy(timeStr, gTimestamp + 11, 8);
    timeStr[8] = '\0';
  }
  
  int lx = 128 - (8 * 6) - 2; // Aliniat la marginea din dreapta
  oled.setCursor(lx, 54);
  oled.print(timeStr);
}
static void pageCPU() {
  drawTitleBar("SYSTEM CORE");
  
  // Massive centered percentage
  char buf[10];
  snprintf(buf, sizeof(buf), "%d%%", (int)roundf(gCpuLoad));
  
  oled.setTextSize(3);
  int offset = (128 - strlen(buf) * 18) / 2;
  oled.setCursor(max(0, offset), 18);
  oled.print(buf);

  // CPU Load Bar (Visual indicator)
  float pct = constrain(gCpuLoad, 0.0f, 100.0f);
  oled.drawRect(14, 44, 100, 6, SSD1306_WHITE);
  oled.fillRect(16, 46, (int)(pct / 100.0f * 96), 2, SSD1306_WHITE);

  // Status texts spread out cleanly on the bottom
  oled.setTextSize(1);
  oled.setCursor(0, 54);
  oled.printf("Up: %lus", (unsigned long)gUptime);
  
  oled.setCursor(72, 54);
  oled.printf("Str: %s", moduleCpuStress ? "ON" : "OFF");
}

static void pageRawLog() {
  drawTitleBar("TELEMETRY");

  // Declumped by shifting Y coords down slightly and dropping one unneeded stat
  oled.setTextSize(1);
  oled.setCursor(8, 20); 
  oled.printf("Therm ADC : %4u", gThermRaw);
  oled.setCursor(8, 32); 
  oled.printf("Gyro X Raw: %6d", gGyroRawX);
  oled.setCursor(8, 44); 
  oled.printf("Gyro Y Raw: %6d", gGyroRawY);
  oled.setCursor(8, 56); 
  oled.printf("Gyro Z Raw: %6d", gGyroRawZ);
}

static void pageTemp() {
  drawTitleBar("TEMPERATURE");
  
  // Custom large thermometer icon for this specific page
  // Scaled from -20C to +40C for realistic skydive visual fill
  float visualHeat = constrain((gTemp + 20.0f) / 60.0f * 100.0f, 0, 100); 
  int tx = 10, ty = 20;
  oled.drawRoundRect(tx, ty, 10, 32, 4, SSD1306_WHITE); // Glass tube
  oled.fillCircle(tx + 4, ty + 34, 7, SSD1306_WHITE);  // Bulb
  int fillH = (int)(visualHeat / 100.0f * 26.0f);
  if (fillH > 0) {
    oled.fillRect(tx + 2, ty + 30 - fillH, 6, fillH, SSD1306_WHITE); // Mercury
  }

  // Split-typography formatting
  int wholeC = (int)gTemp;
  int decC = (int)(abs(gTemp) * 10) % 10; // Get first decimal digit
  
  // Print giant whole number
  oled.setTextSize(3);
  char wBuf[6]; 
  snprintf(wBuf, sizeof(wBuf), "%d", wholeC);
  int wLen = strlen(wBuf) * 18;
  oled.setCursor(38, 28);
  oled.print(wBuf);
  
  // Print smaller decimal right next to it (baseline aligned)
  oled.setTextSize(2);
  oled.setCursor(38 + wLen, 35); 
  oled.printf(".%d", decC);
  
  // Print Degree C symbol perfectly spaced
  oled.drawCircle(38 + wLen + 26, 28, 3, SSD1306_WHITE); 
  oled.setCursor(38 + wLen + 32, 28);
  oled.print("C");
}

static void pageGyro() {
  drawTitleBar("ATTITUDE HUD");
  
  // Center Gauge (Moved down safely away from title/yaw)
  drawArtificialHorizon(64, 42, 20, gPitch, gRoll);
  
  // Top Center: Magnetic/Yaw Heading
  char yBuf[16]; 
  snprintf(yBuf, sizeof(yBuf), "HDG: %03.0f", gYaw);
  drawCentered(yBuf, 16, 1);
  
  // ==========================================
  // LEFT EDGE: Pitch Vertical Slider
  // ==========================================
  oled.drawRect(0, 22, 5, 40, SSD1306_WHITE); // Track
  // Map Pitch (-90 to +90) into the physical slider space (Y: 22 to 62)
  int pY = 42 - (int)(constrain(gPitch, -90.0f, 90.0f) / 90.0f * 18.0f);
  oled.fillRect(0, pY - 2, 8, 5, SSD1306_WHITE); // Moving cursor block
  
  oled.setTextSize(1);
  oled.setCursor(10, 32); oled.print("P");
  oled.setCursor(10, 42); oled.printf("%.0f", gPitch);

  // ==========================================
  // RIGHT EDGE: Roll Vertical Slider
  // ==========================================
  oled.drawRect(123, 22, 5, 40, SSD1306_WHITE); // Track
  // Map Roll (-180 to +180) into the physical slider space
  int rY = 42 - (int)(constrain(gRoll, -180.0f, 180.0f) / 180.0f * 18.0f);
  oled.fillRect(120, rY - 2, 8, 5, SSD1306_WHITE); // Moving cursor block
  
  // Text pushed inward so it doesn't get cut off
  int rightX = 94; 
  if (gRoll <= -100 || gRoll >= 100) rightX = 88; // Shift left if 3 digits
  oled.setCursor(114, 32); oled.print("R");
  oled.setCursor(rightX, 42); oled.printf("%.0f", gRoll);
}
static void pageWiFi() {
  drawTitleBar("BLUETOOTH LINK");

  oled.setTextSize(1);
  if (bleConnected) {
    // Massive centered Bluetooth rune
    int bx = 58; int by = 22;
    oled.drawLine(bx + 6, by, bx + 6, by + 18, SSD1306_WHITE); 
    oled.drawLine(bx + 6, by, bx + 12, by + 4, SSD1306_WHITE); 
    oled.drawLine(bx + 12, by + 4, bx, by + 13, SSD1306_WHITE); 
    oled.drawLine(bx + 6, by + 18, bx + 12, by + 13, SSD1306_WHITE); 
    oled.drawLine(bx + 12, by + 13, bx, by + 4, SSD1306_WHITE); 
    
    drawCentered("CONNECTED", 48, 1);
    drawCentered("Streaming Telemetry", 56, 1);
  } else {
    // Giant X to show broken connection
    oled.drawLine(44, 20, 84, 40, SSD1306_WHITE);
    oled.drawLine(44, 21, 84, 41, SSD1306_WHITE); // Double thickness
    
    oled.drawLine(84, 20, 44, 40, SSD1306_WHITE);
    oled.drawLine(84, 21, 44, 41, SSD1306_WHITE); // Double thickness

    drawCentered("DISCONNECTED", 48, 1);
    drawCentered("Check Phone App", 56, 1);
  }
}
static void pageVoltage() {
  drawTitleBar("POWER (VOLTS)");
  
  char buf[10];
  snprintf(buf, sizeof(buf), "%.2f", gVoltage);
  int numW = strlen(buf) * 18;
  int sx = max(0, (128 - numW - 14) / 2);
  
  oled.setTextSize(3); oled.setCursor(sx, 22); oled.print(buf);
  oled.setTextSize(2); oled.setCursor(sx + numW + 4, 28); oled.print("V");

  // Gauge line from 3.0V (empty) to 4.2V (full)
  float pct = constrain((gVoltage - 3.0f) / 1.2f * 100.0f, 0, 100);
  oled.drawRect(14, 54, 100, 6, SSD1306_WHITE);
  oled.fillRect(16, 56, (int)(pct / 100.0f * 96), 2, SSD1306_WHITE);
}

static void pageCurrent() {
  drawTitleBar("POWER (AMPS)");
  
  char buf[16];
  snprintf(buf, sizeof(buf), "%.0f", gCurrentMA);
  int numW = strlen(buf) * 18;
  int sx = max(0, (128 - numW - 24) / 2);

  oled.setTextSize(3); oled.setCursor(sx, 22); oled.print(buf);
  oled.setTextSize(2); oled.setCursor(sx + numW + 4, 28); oled.print("mA");

  oled.setTextSize(1);
  char pow[24];
  snprintf(pow, sizeof(pow), "Draw: %.0f mW", gPowerMW);
  drawCentered(pow, 54, 1);
}

static void pageBattery() {
  drawTitleBar("BATTERY CELL");
  
  drawDynamicBattery(34, 18, 60, 24, gBattPct);

  // Put percentage inside the massive battery block if we can, or just below
  char buf[10];
  snprintf(buf, sizeof(buf), "%.0f%%", gBattPct);
  drawCentered(buf, 48, 2);
}

static void pageI2CScanner() {
  drawTitleBar("I2C DIAGNOSTICS");
  oled.setTextSize(1);

  if (gI2cFoundCount == 0) {
    drawCentered("BUS DEAD / MISO", 32, 1);
    return;
  }

  const uint8_t maxRows = 4;
  const uint8_t totalPages = (gI2cFoundCount + maxRows - 1) / maxRows;
  const uint8_t pageIdx = (totalPages > 1) ? (uint8_t)((millis() / 2000UL) % totalPages) : 0;
  const uint8_t firstIndex = pageIdx * maxRows;

  for (uint8_t i = 0; i < maxRows; i++) {
    const uint8_t idx = firstIndex + i;
    if (idx >= gI2cFoundCount) break;
    const uint8_t addr = gI2cFound[idx];
    oled.setCursor(12, 18 + i * 11);
    oled.printf("[-] 0x%02X %s", addr, i2cDeviceName(addr));
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
    case 9: pageI2CScanner(); break;
    case 10: pageRTC();      break; // <--- ADD THIS LINE
  }
  oled.display();}

// ================================================================
// SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== Hard&Soft Task 3 — Skydiver Monitor (BLE-Only) ===\n");

  // --- I2C at 100kHz (DS1307-compatible) ---
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_FREQ);
  Wire.setTimeOut(50);

analogReadResolution(12);
analogSetPinAttenuation(THERM_PIN, ADC_11db); // FIX: Permite citirea până la 3.3V!
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
  cpuPrevIdleHookCount = cpuIdleHookCount;
  cpuLastLoadSampleMs = millis();

  moduleCpuStress = false;
  if (xTaskCreate(cpuStressTask, "cpu_stress", 3072, nullptr, 1, nullptr) != pdPASS) {
    Serial.println("[CPU] Stress task create FAILED");
  }

  // --- Buttons (interrupt-driven) ---
  pinMode(BTN_NEXT, INPUT_PULLUP);
  pinMode(BTN_PREV, INPUT_PULLUP);
  // Attach early so skip can work during initial Wi-Fi connection.
  attachInterrupt(digitalPinToInterrupt(BTN_NEXT), isrBtnNext, FALLING);
  attachInterrupt(digitalPinToInterrupt(BTN_PREV), isrBtnPrev, FALLING);

  // --- Gyroscope INT pin (data ready interrupt) ---
  pinMode(GYRO_INT, INPUT_PULLUP);  // Robust for boards where INT behaves open-drain/floating
  // NOTE: Interrupt attached after initGyro() since we need to know if MPU is present

  // --- I2C scan ---
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
  gRtcAddr = 0;
  if (i2cProbe(RTC_ADDR_DS1307)) gRtcAddr = RTC_ADDR_DS1307;
  else if (i2cProbe(RTC_ADDR_ALT)) gRtcAddr = RTC_ADDR_ALT;

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

  // --- Gyroscope ---
  hasGyro = initGyro();
  if (!hasGyro) moduleGyro = false;
  Serial.printf("[GYRO]   %s", hasGyro ? "OK" : "MISSING/UNKNOWN");
  if (hasGyro) {
    Serial.printf(" @0x%02X %s\n", gImuAddr, gImuIsMpu ? "(MPU6500/9250)" : "(LSM6DS3)");
    // Attach interrupt for data-ready if MPU is present
    if (gImuIsMpu) {
      attachInterrupt(digitalPinToInterrupt(GYRO_INT), isrGyroDataReady,
                      gMpuDmpActive ? FALLING : RISING);
      Serial.printf("[GYRO-INT] Attached to GPIO%u (data ready interrupt)\n", GYRO_INT);
      Serial.printf("[GYRO]   MPU mode: %s\n", gMpuDmpActive ? "DMP quaternion" : "register fusion");
    }
    Serial.printf("[MAG]    %s\n", gHasMag ? "AK8963 detected" : "Not present");
  } else {
    Serial.println();
  }

  
  // --- BLE (Bluetooth Low Energy) ---
  Serial.println("[BLE] Initializing BLE communication...");
  initBLE();
  
  // =======================================================
  // BOOT ANIMATION & BLE PAIRING WAIT
  // =======================================================
  if (hasOLED) {
    int radius = 0;
    
    // Block execution and play radar animation until a device connects
    while (!bleConnected) {
      oled.clearDisplay();
      
      // Inverted header
      oled.fillRect(0, 0, 128, 14, SSD1306_WHITE);
      oled.setTextColor(SSD1306_BLACK);
      oled.setTextSize(1);
      int tx = (128 - 11 * 6) / 2;
      oled.setCursor(max(0, tx), 3);
      oled.print("BLE PAIRING");
      oled.setTextColor(SSD1306_WHITE);

      // Radar pulse effect
      oled.drawCircle(64, 32, 6 + radius, SSD1306_WHITE);
      if (radius > 8) oled.drawCircle(64, 32, radius - 8, SSD1306_WHITE);
      
      // Center Bluetooth Rune
      oled.drawLine(64, 26, 64, 38, SSD1306_WHITE);
      oled.drawLine(64, 26, 68, 29, SSD1306_WHITE);
      oled.drawLine(68, 29, 60, 35, SSD1306_WHITE);
      oled.drawLine(64, 38, 68, 35, SSD1306_WHITE);
      oled.drawLine(68, 35, 60, 29, SSD1306_WHITE);

      // Blinking text
      if ((millis() / 500) % 2 == 0) {
        int w = 18 * 6; // "Waiting for App..." is 18 chars
        oled.setCursor((128 - w) / 2, 54);
        oled.print("Waiting for App...");
      }
      
      oled.display();
      
      radius++;
      if (radius > 20) radius = 0; // Reset animation loop
      
      delay(40); // Yield to ESP32 background tasks (prevents watchdog crash)
    }

    // SUCCESS ANIMATION (Flashes screen inverted)
    oled.clearDisplay();
    oled.fillRect(0, 0, 128, 64, SSD1306_WHITE); // Full white screen
    oled.setTextColor(SSD1306_BLACK);
    
    oled.setTextSize(2);
    int lx = (128 - 4 * 12) / 2; // "LINK"
    oled.setCursor(lx, 16);
    oled.print("LINK");
    
    oled.setTextSize(1);
    int ex = (128 - 11 * 6) / 2; // "ESTABLISHED"
    oled.setCursor(ex, 38);
    oled.print("ESTABLISHED");
    
    oled.display();
    oled.setTextColor(SSD1306_WHITE); // Reset text color for main loop
    delay(1500); // Hold success screen for 1.5s
  }
}

// ================================================================
// LOOP — minimal: only react to flags, no polling
// ================================================================
void loop() {
  // Buttons (ISR-driven, just check flags)
  handleButtons();

  // FAST: Gyroscope via interrupt (GPIO10 INT pin), drain a few queued events per loop.
  if (hasGyro && moduleGyro) {
    uint16_t pending = 0;
    noInterrupts();
    pending = gyroDataReadyCount;
    gyroDataReadyCount = 0;
    interrupts();

    if (pending > 3) pending = 3;  // avoid starving the rest of the loop

    while (pending--) {
      (void)readGyro();  // Fast, non-blocking gyro read only
      gLastGyroReadMs = millis();
    }
  }

  // Fallback: if INT wiring/signal is missing, keep gyro alive via light polling.
  if (hasGyro && moduleGyro && (millis() - gLastGyroReadMs >= 20)) {
    (void)readGyro();
    gLastGyroReadMs = millis();
  }

  // FAST IMU publish via BLE (when connected).
  if (hasGyro && moduleGyro && bleConnected && (millis() - lastImuPublishMs >= IMU_PUBLISH_INTERVAL_MS)) {
    lastImuPublishMs = millis();
    publishIMUDataBLE();
  }

  if (hasGyro && moduleGyro && !bleConnected && (millis() - lastImuPublishMs >= IMU_PUBLISH_IDLE_INTERVAL_MS)) {
    lastImuPublishMs = millis();
  }

  // SLOW: Other sensors (1Hz) + BLE publish
  if (millis() - lastOtherSensorRead >= 1000) {
    lastOtherSensorRead = millis();

    readOtherSensors();  // Read temp, current, time, cpu

    // BLE publish (1Hz when connected)
    if (bleConnected) {
      publishSensorDataBLE();
      publishIMUDataBLE();
    }

    drawPage();

    // Serial log
    Serial.printf("[%s] T=%.1f TH=%u GX=%.1f GY=%.1f GZ=%.1f V=%.2f I=%.1f CPU=%.1f%%\n",
      gTimestamp, gTemp, gThermRaw, gGyroX, gGyroY, gGyroZ, gVoltage, gCurrentMA, gCpuLoad);
  }

  // Refresh scanner data in background for the dedicated OLED I2C page.
  if (millis() - gLastI2cScanMs >= 3000) {
    scanI2cDevices();
  }

  // Update BLE connection state for OLED display
// Update BLE connection state for OLED display
  if (oldBleConnected != bleConnected) {
    oldBleConnected = bleConnected;
    if (bleConnected) {
      Serial.println("[BLE] Client connected");
    } else {
      Serial.println("[BLE] Client disconnected");
      
      // Auto-switch to the Bluetooth page so the user sees the giant X immediately
      page = 3; 
      drawPage();
    }
  }
// Force rapid screen redraw ONLY if we are looking at the running timer
  if (hasOLED && page == 10 && gTimerRunning) {
    drawPage();
  }
  delay(2);  // Small delay to yield to other tasks
}