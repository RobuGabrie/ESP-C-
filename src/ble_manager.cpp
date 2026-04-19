#include "ble_manager.h"

#include <ArduinoJson.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <cmath>

namespace {

static constexpr const char* BLE_DEVICE_NAME = "Skydiver-Monitor";
static constexpr const char* SERVICE_UUID = "180D";
static constexpr const char* CHAR_TEMPERATURE_UUID = "2A1C";
static constexpr const char* CHAR_GYRO_UUID = "2A4E";
static constexpr const char* CHAR_BATTERY_UUID = "2A19";
static constexpr const char* CHARACTERISTIC_NOTIFY_UUID = "a0000001-1000-1000-8000-00805f9b34fb";
static constexpr const char* CHARACTERISTIC_IMU_UUID = "a0000002-1000-1000-8000-00805f9b34fb";

BLEServer* gBleServer = nullptr;
BLEService* gService = nullptr;
BLECharacteristic* gCharSensorData = nullptr;
BLECharacteristic* gCharTemperature = nullptr;
BLECharacteristic* gCharGyro = nullptr;
BLECharacteristic* gCharBattery = nullptr;
BLECharacteristic* gCharIMU = nullptr;

bool gBleConnected = false;
uint32_t gBleSensorNotifyCount = 0;
uint32_t gBleImuNotifyCount = 0;
uint32_t gBleImuSkipLogMs = 0;
BleManager::CommandHandler gCommandHandler = nullptr;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) override {
    (void)server;
    gBleConnected = true;
    Serial.println("[BLE] Client connected");
  }

  void onDisconnect(BLEServer* server) override {
    gBleConnected = false;
    Serial.println("[BLE] Client disconnected");
    server->startAdvertising();
  }
};

class CharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
    std::string rxValue = characteristic->getValue();
    if (rxValue.empty()) {
      return;
    }

    Serial.printf("[BLE] Received %d bytes: ", (int)rxValue.length());
    String cmd;
    cmd.reserve(rxValue.length());
    for (size_t i = 0; i < rxValue.length(); i++) {
      cmd += rxValue[i];
    }
    Serial.println(cmd);

    cmd.trim();
    cmd.toUpperCase();
    if (gCommandHandler != nullptr) {
      gCommandHandler(cmd);
    }
  }
};

static void normalizeQuat(float& q0, float& q1, float& q2, float& q3) {
  float n = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  if (n > 1e-6f) {
    q0 /= n;
    q1 /= n;
    q2 /= n;
    q3 /= n;
  } else {
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
  }
}

}  // namespace

namespace BleManager {

void setCommandHandler(CommandHandler handler) {
  gCommandHandler = handler;
}

void init() {
  Serial.println("[BLE] Initializing BLE...");

  BLEDevice::setMTU(247);
  BLEDevice::init(BLE_DEVICE_NAME);

  gBleServer = BLEDevice::createServer();
  gBleServer->setCallbacks(new ServerCallbacks());

  gService = gBleServer->createService(SERVICE_UUID);

  gCharSensorData = gService->createCharacteristic(
    CHARACTERISTIC_NOTIFY_UUID,
    BLECharacteristic::PROPERTY_NOTIFY |
      BLECharacteristic::PROPERTY_READ |
      BLECharacteristic::PROPERTY_WRITE |
      BLECharacteristic::PROPERTY_WRITE_NR);
  gCharSensorData->addDescriptor(new BLE2902());
  gCharSensorData->setCallbacks(new CharacteristicCallbacks());

  gCharTemperature = gService->createCharacteristic(
    CHAR_TEMPERATURE_UUID,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  gCharTemperature->addDescriptor(new BLE2902());

  gCharGyro = gService->createCharacteristic(
    CHAR_GYRO_UUID,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  gCharGyro->addDescriptor(new BLE2902());

  gCharBattery = gService->createCharacteristic(
    CHAR_BATTERY_UUID,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  gCharBattery->addDescriptor(new BLE2902());

  gCharIMU = gService->createCharacteristic(
    CHARACTERISTIC_IMU_UUID,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  gCharIMU->addDescriptor(new BLE2902());

  gService->start();

  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.printf("[BLE] BLE Advertising started - Device Name: %s\n", BLE_DEVICE_NAME);
}

bool isConnected() {
  return gBleConnected;
}

const char* deviceName() {
  return BLE_DEVICE_NAME;
}

uint32_t sensorNotifyCount() {
  return gBleSensorNotifyCount;
}

uint32_t imuNotifyCount() {
  return gBleImuNotifyCount;
}

void publishSensorData(const SensorData& data) {
  if (!gBleConnected || gCharSensorData == nullptr) {
    return;
  }

  char buf[384];
  JsonDocument doc;

  doc["ts"] = data.timestamp;
  doc["up"] = data.uptime;

  if (data.moduleTemp && data.thermValid) {
    doc["temp"] = roundf(data.tempC * 10.0f) / 10.0f;
  }

  if (data.moduleGyro && data.hasGyro) {
    doc["gx"] = roundf(data.gyroX * 10.0f) / 10.0f;
    doc["gy"] = roundf(data.gyroY * 10.0f) / 10.0f;
    doc["gz"] = roundf(data.gyroZ * 10.0f) / 10.0f;

    float q0 = isfinite(data.quat0) ? data.quat0 : 1.0f;
    float q1 = isfinite(data.quat1) ? data.quat1 : 0.0f;
    float q2 = isfinite(data.quat2) ? data.quat2 : 0.0f;
    float q3 = isfinite(data.quat3) ? data.quat3 : 0.0f;
    normalizeQuat(q0, q1, q2, q3);

    doc["q0"] = roundf(q0 * 10000.0f) / 10000.0f;
    doc["q1"] = roundf(q1 * 10000.0f) / 10000.0f;
    doc["q2"] = roundf(q2 * 10000.0f) / 10000.0f;
    doc["q3"] = roundf(q3 * 10000.0f) / 10000.0f;
    doc["roll"] = roundf(data.roll * 100.0f) / 100.0f;
    doc["pitch"] = roundf(data.pitch * 100.0f) / 100.0f;
    doc["yaw"] = roundf(data.yaw * 100.0f) / 100.0f;
    doc["stationary"] = (data.motionStillCount >= data.zuptStillRequiredSamples);
    doc["imu_seq"] = gBleImuNotifyCount;
  }

  if (data.moduleCurrent) {
    doc["v"] = roundf(data.voltage * 100.0f) / 100.0f;
    doc["ma"] = roundf(data.currentMA * 10.0f) / 10.0f;
    doc["batt"] = roundf(data.battPct * 10.0f) / 10.0f;
  }

  if (data.moduleCPU) {
    doc["cpu"] = roundf(data.cpuLoad * 10.0f) / 10.0f;
  }

  size_t needed = measureJson(doc);
  if (needed >= sizeof(buf)) {
    Serial.printf("[BLE] Sensor payload too large: needed=%u, max=%u (drop)\n",
                  (unsigned)needed,
                  (unsigned)(sizeof(buf) - 1));
    return;
  }

  size_t len = serializeJson(doc, buf, sizeof(buf));
  if (len != needed) {
    Serial.printf("[BLE] Sensor serialize mismatch: len=%u needed=%u (drop)\n",
                  (unsigned)len, (unsigned)needed);
    return;
  }

  gCharSensorData->setValue((uint8_t*)buf, len);
  gCharSensorData->notify();
  gBleSensorNotifyCount++;

  if (gCharTemperature != nullptr && data.moduleTemp && data.thermValid) {
    char tbuf[16];
    int tlen = snprintf(tbuf, sizeof(tbuf), "%.1f", data.tempC);
    if (tlen > 0) {
      gCharTemperature->setValue((uint8_t*)tbuf, (size_t)tlen);
      gCharTemperature->notify();
    }
  }

  if (gCharBattery != nullptr) {
    uint8_t batt = (uint8_t)constrain((int)roundf(data.battPct), 0, 100);
    gCharBattery->setValue(&batt, 1);
    gCharBattery->notify();
  }

  if (gCharGyro != nullptr && data.moduleGyro && data.hasGyro) {
    float gabs = sqrtf(data.gyroX * data.gyroX + data.gyroY * data.gyroY + data.gyroZ * data.gyroZ);
    char gbuf[16];
    int glen = snprintf(gbuf, sizeof(gbuf), "%.1f", gabs);
    if (glen > 0) {
      gCharGyro->setValue((uint8_t*)gbuf, (size_t)glen);
      gCharGyro->notify();
    }
  }

  if ((gBleSensorNotifyCount % 10) == 0) {
    Serial.printf("[BLE] Sensor notify #%lu, len=%u\n",
                  (unsigned long)gBleSensorNotifyCount,
                  (unsigned)len);
  }
}

void publishImuData(const ImuData& data) {
  if (!gBleConnected) {
    return;
  }

  if (gCharIMU == nullptr) {
    uint32_t now = millis();
    if (now - gBleImuSkipLogMs >= 3000) {
      gBleImuSkipLogMs = now;
      Serial.println("[BLE] IMU notify skipped: pCharIMU is null");
    }
    return;
  }

  char buf[512];
  JsonDocument doc;

  float q0 = isfinite(data.quat0) ? data.quat0 : 1.0f;
  float q1 = isfinite(data.quat1) ? data.quat1 : 0.0f;
  float q2 = isfinite(data.quat2) ? data.quat2 : 0.0f;
  float q3 = isfinite(data.quat3) ? data.quat3 : 0.0f;
  normalizeQuat(q0, q1, q2, q3);

  doc["up"] = data.uptime;
  doc["q0"] = roundf(q0 * 10000.0f) / 10000.0f;
  doc["q1"] = roundf(q1 * 10000.0f) / 10000.0f;
  doc["q2"] = roundf(q2 * 10000.0f) / 10000.0f;
  doc["q3"] = roundf(q3 * 10000.0f) / 10000.0f;
  doc["roll"] = roundf(data.roll * 100.0f) / 100.0f;
  doc["pitch"] = roundf(data.pitch * 100.0f) / 100.0f;
  doc["yaw"] = roundf(data.yaw * 100.0f) / 100.0f;
  doc["stationary"] = (data.motionStillCount >= data.zuptStillRequiredSamples);

  size_t len = serializeJson(doc, buf, sizeof(buf));
  if (len < sizeof(buf)) {
    gCharIMU->setValue((uint8_t*)buf, len);
    gCharIMU->notify();
    gBleImuNotifyCount++;
    if ((gBleImuNotifyCount % 20) == 0) {
      Serial.printf("[BLE] IMU notify #%lu, len=%u\n",
                    (unsigned long)gBleImuNotifyCount,
                    (unsigned)len);
    }
  }
}

}  // namespace BleManager
