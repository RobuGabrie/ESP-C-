#include "ble_manager.h"

#include <NimBLEDevice.h>
#include <algorithm>
#include <cctype>
#include <cmath>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace {

static constexpr const char* BLE_DEVICE_NAME = "skywatch.";
static constexpr const char* SERVICE_UUID = "12345678-1234-1234-1234-123456789abc";
static constexpr const char* FAST_CHAR_UUID = "12345678-1234-1234-1234-123456789001";
static constexpr const char* SLOW_CHAR_UUID = "12345678-1234-1234-1234-123456789002";
static constexpr const char* COMMAND_CHAR_UUID = "12345678-1234-1234-1234-123456789003";
static constexpr uint32_t FAST_NOTIFY_INTERVAL_MS = 20;   // 50 Hz
static constexpr uint32_t SLOW_NOTIFY_INTERVAL_MS = 250;  // 4 Hz

NimBLEServer* gBleServer = nullptr;
NimBLEService* gService = nullptr;
NimBLECharacteristic* gFastChar = nullptr;
NimBLECharacteristic* gSlowChar = nullptr;
NimBLECharacteristic* gCommandChar = nullptr;

bool gBleConnected = false;
uint32_t gBleSensorNotifyCount = 0;
uint32_t gBleImuNotifyCount = 0;
BleManager::CommandHandler gCommandHandler = nullptr;
TaskHandle_t gFastTaskHandle = nullptr;
TaskHandle_t gSlowTaskHandle = nullptr;
portMUX_TYPE gPacketMux = portMUX_INITIALIZER_UNLOCKED;
BleManager::FastPacket gFastPacket = {};
BleManager::SlowPacket gSlowPacket = {};
uint8_t gFastSeq = 0;
uint8_t gSlowSeq = 0;
bool gFastPrimed = false;
bool gSlowPrimed = false;

static float clampf(float v, float lo, float hi) {
  if (v < lo) {
    return lo;
  }
  if (v > hi) {
    return hi;
  }
  return v;
}

static int16_t toInt16Scaled(float value, float scale, float lo, float hi) {
  const float clamped = clampf(value * scale, lo, hi);
  return static_cast<int16_t>(lroundf(clamped));
}

static uint16_t toUint16Scaled(float value, float scale, float lo, float hi) {
  const float clamped = clampf(value * scale, lo, hi);
  return static_cast<uint16_t>(lroundf(clamped));
}

static uint8_t toUint8Clamped(float value, float lo, float hi) {
  const float clamped = clampf(value, lo, hi);
  return static_cast<uint8_t>(lroundf(clamped));
}

static void parseTimestamp(const char* ts,
                           uint16_t& year,
                           uint8_t& month,
                           uint8_t& day,
                           uint8_t& hour,
                           uint8_t& minute,
                           uint8_t& second) {
  year = 0;
  month = 0;
  day = 0;
  hour = 0;
  minute = 0;
  second = 0;

  if (ts == nullptr) {
    return;
  }

  int y = 0;
  int mo = 0;
  int d = 0;
  int h = 0;
  int mi = 0;
  int s = 0;
  const int parsed = sscanf(ts, "%d-%d-%d %d:%d:%d", &y, &mo, &d, &h, &mi, &s);
  if (parsed != 6) {
    return;
  }

  year = static_cast<uint16_t>(clampf(static_cast<float>(y), 0.0f, 65535.0f));
  month = static_cast<uint8_t>(clampf(static_cast<float>(mo), 0.0f, 12.0f));
  day = static_cast<uint8_t>(clampf(static_cast<float>(d), 0.0f, 31.0f));
  hour = static_cast<uint8_t>(clampf(static_cast<float>(h), 0.0f, 23.0f));
  minute = static_cast<uint8_t>(clampf(static_cast<float>(mi), 0.0f, 59.0f));
  second = static_cast<uint8_t>(clampf(static_cast<float>(s), 0.0f, 59.0f));
}

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* server, NimBLEConnInfo& connInfo) override {
    gBleConnected = true;
    // Ask for a low-latency link right after connect (7.5-15 ms interval).
    server->updateConnParams(connInfo.getConnHandle(), 6, 12, 0, 200);
    Serial.println("[BLE] Client connected");
  }

  void onDisconnect(NimBLEServer* server, NimBLEConnInfo& connInfo, int reason) override {
    (void)connInfo;
    (void)reason;
    gBleConnected = false;
    Serial.println("[BLE] Client disconnected");
    NimBLEDevice::startAdvertising();
  }
};

static void fastNotifyTask(void* pv) {
  (void)pv;
  TickType_t xLastWake = xTaskGetTickCount();

  while (true) {
    BleManager::FastPacket packetCopy;
    bool canSend = false;

    portENTER_CRITICAL(&gPacketMux);
    packetCopy = gFastPacket;
    canSend = gFastPrimed;
    portEXIT_CRITICAL(&gPacketMux);

    if (canSend && gBleConnected && gFastChar != nullptr) {
      gFastChar->setValue(reinterpret_cast<uint8_t*>(&packetCopy), sizeof(packetCopy));
      gFastChar->notify();
      gBleImuNotifyCount++;

      if ((gBleImuNotifyCount % 40) == 0) {
        Serial.printf("[BLE] FAST notify #%lu, %u bytes\n",
                      static_cast<unsigned long>(gBleImuNotifyCount),
                      static_cast<unsigned>(sizeof(packetCopy)));
      }
    }

    vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(FAST_NOTIFY_INTERVAL_MS));
  }
}

static void slowNotifyTask(void* pv) {
  (void)pv;

  while (true) {
    BleManager::SlowPacket packetCopy;
    bool canSend = false;

    portENTER_CRITICAL(&gPacketMux);
    packetCopy = gSlowPacket;
    canSend = gSlowPrimed;
    portEXIT_CRITICAL(&gPacketMux);

    if (canSend && gBleConnected && gSlowChar != nullptr) {
      gSlowChar->setValue(reinterpret_cast<uint8_t*>(&packetCopy), sizeof(packetCopy));
      gSlowChar->notify();
      gBleSensorNotifyCount++;

      if ((gBleSensorNotifyCount % 10) == 0) {
        Serial.printf("[BLE] SLOW notify #%lu, %u bytes\n",
                      static_cast<unsigned long>(gBleSensorNotifyCount),
                      static_cast<unsigned>(sizeof(packetCopy)));
      }
    }

    vTaskDelay(pdMS_TO_TICKS(SLOW_NOTIFY_INTERVAL_MS));
  }
}

static uint8_t buildSlowFlags(const BleManager::SensorData& data) {
  uint8_t flags = 0;
  if (data.thermValid) {
    flags |= 0x01;
  }
  if (data.moduleGyro && data.hasGyro) {
    flags |= 0x02;
  }
  if (data.moduleCurrent) {
    flags |= 0x04;
  }
  if (data.moduleCPU) {
    flags |= 0x08;
  }
  if (data.motionStillCount >= data.zuptStillRequiredSamples) {
    flags |= 0x10;
  }
  if (gBleConnected) {
    flags |= 0x20;
  }
  if (data.modulePulse) {
    flags |= 0x40;
  }
  if (data.pulseValid) {
    flags |= 0x80;
  }
  return flags;
}

static String normalizeBleCommand(const std::string& raw) {
  String out;
  out.reserve(raw.size());

  for (char c : raw) {
    const unsigned char uc = static_cast<unsigned char>(c);
    if (uc == 0) {
      continue;
    }
    out += static_cast<char>(std::toupper(uc));
  }

  out.trim();
  return out;
}

class CommandCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo& connInfo) override {
    (void)connInfo;

    if (characteristic == nullptr || gCommandHandler == nullptr) {
      return;
    }

    const std::string raw = characteristic->getValue();
    const String cmd = normalizeBleCommand(raw);
    if (cmd.length() == 0) {
      return;
    }

    if (cmd == "START" || cmd == "STOP" || cmd == "RESET") {
      Serial.printf("[BLE] Command received: %s\n", cmd.c_str());
      gCommandHandler(cmd);
      return;
    }

    Serial.printf("[BLE] Ignored unknown command: %s\n", cmd.c_str());
  }
};

}  // namespace

namespace BleManager {

void setCommandHandler(CommandHandler handler) {
  gCommandHandler = handler;
}

void init() {
  Serial.println("[BLE] Initializing NimBLE...");
  Serial.printf("[BLE] Packet sizes: FAST=%u SLOW=%u\n",
                static_cast<unsigned>(sizeof(BleManager::FastPacket)),
                static_cast<unsigned>(sizeof(BleManager::SlowPacket)));

  NimBLEDevice::init(BLE_DEVICE_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setMTU(247);

  gBleServer = NimBLEDevice::createServer();
  gBleServer->setCallbacks(new ServerCallbacks());

  gService = gBleServer->createService(SERVICE_UUID);

  gFastChar = gService->createCharacteristic(FAST_CHAR_UUID, NIMBLE_PROPERTY::NOTIFY);
  gSlowChar = gService->createCharacteristic(SLOW_CHAR_UUID, NIMBLE_PROPERTY::NOTIFY);
  gCommandChar = gService->createCharacteristic(COMMAND_CHAR_UUID,
                                                NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);

  if (gCommandChar != nullptr) {
    gCommandChar->setCallbacks(new CommandCallbacks());
  }

  NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->enableScanResponse(true);
  advertising->start();

  if (gFastTaskHandle == nullptr) {
    xTaskCreate(fastNotifyTask, "ble_fast", 3072, nullptr, 2, &gFastTaskHandle);
  }
  if (gSlowTaskHandle == nullptr) {
    xTaskCreate(slowNotifyTask, "ble_slow", 3072, nullptr, 1, &gSlowTaskHandle);
  }

  if (gCommandHandler != nullptr) {
    Serial.printf("[BLE] Command characteristic enabled: %s\n", COMMAND_CHAR_UUID);
  }

  Serial.printf("[BLE] NimBLE advertising started - Device Name: %s\n", BLE_DEVICE_NAME);
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
  BleManager::SlowPacket pkt = {};

  pkt.uptimeMs = data.uptime;
  uint16_t rtcYear = 0;
  uint8_t rtcMonth = 0;
  uint8_t rtcDay = 0;
  uint8_t rtcHour = 0;
  uint8_t rtcMinute = 0;
  uint8_t rtcSecond = 0;
  parseTimestamp(data.timestamp, rtcYear, rtcMonth, rtcDay, rtcHour, rtcMinute, rtcSecond);
  pkt.rtcYear = rtcYear;
  pkt.rtcMonth = rtcMonth;
  pkt.rtcDay = rtcDay;
  pkt.rtcHour = rtcHour;
  pkt.rtcMinute = rtcMinute;
  pkt.rtcSecond = rtcSecond;

  if (data.moduleTemp && data.thermValid) {
    pkt.tempCenti = toInt16Scaled(data.tempC, 100.0f, -32768.0f, 32767.0f);
  }
  pkt.voltageCenti = toUint16Scaled(data.voltage, 100.0f, 0.0f, 65535.0f);
  pkt.currentMa = toInt16Scaled(data.currentMA, 1.0f, -32768.0f, 32767.0f);
  pkt.battPct10 = toUint16Scaled(data.battPct, 10.0f, 0.0f, 1000.0f);
  pkt.cpuPct10 = toUint16Scaled(data.cpuLoad, 10.0f, 0.0f, 1000.0f);
  pkt.bpm10 = toUint16Scaled(data.bpm, 10.0f, 0.0f, 3000.0f);
  pkt.spo210 = toUint16Scaled(data.spo2, 10.0f, 0.0f, 1000.0f);
  pkt.stressPct10 = toUint16Scaled(data.stressPct, 10.0f, 0.0f, 1000.0f);
  pkt.flags = buildSlowFlags(data);

  portENTER_CRITICAL(&gPacketMux);
  gSlowSeq++;
  pkt.seq = gSlowSeq;
  gSlowPacket = pkt;
  gSlowPrimed = true;
  if (data.moduleGyro && data.hasGyro) {
    gFastPacket.gxDps10 = toInt16Scaled(data.gyroX, 10.0f, -32768.0f, 32767.0f);
    gFastPacket.gyDps10 = toInt16Scaled(data.gyroY, 10.0f, -32768.0f, 32767.0f);
    gFastPacket.gzDps10 = toInt16Scaled(data.gyroZ, 10.0f, -32768.0f, 32767.0f);
  }
  portEXIT_CRITICAL(&gPacketMux);
}

void publishImuData(const ImuData& data) {
  BleManager::FastPacket pkt = {};
  pkt.uptimeMs = data.uptime;
  pkt.rollCdeg = toInt16Scaled(data.roll, 100.0f, -32768.0f, 32767.0f);
  pkt.pitchCdeg = toInt16Scaled(data.pitch, 100.0f, -32768.0f, 32767.0f);
  pkt.yawCdeg = toInt16Scaled(data.yaw, 100.0f, -32768.0f, 32767.0f);
  pkt.gxDps10 = toInt16Scaled(data.gyroX, 10.0f, -32768.0f, 32767.0f);
  pkt.gyDps10 = toInt16Scaled(data.gyroY, 10.0f, -32768.0f, 32767.0f);
  pkt.gzDps10 = toInt16Scaled(data.gyroZ, 10.0f, -32768.0f, 32767.0f);
  pkt.axMg = toInt16Scaled(data.accelX, 1000.0f, -32768.0f, 32767.0f);
  pkt.ayMg = toInt16Scaled(data.accelY, 1000.0f, -32768.0f, 32767.0f);
  pkt.azMg = toInt16Scaled(data.accelZ, 1000.0f, -32768.0f, 32767.0f);
  pkt.q0e4 = toInt16Scaled(data.quat0, 10000.0f, -32768.0f, 32767.0f);
  pkt.q1e4 = toInt16Scaled(data.quat1, 10000.0f, -32768.0f, 32767.0f);
  pkt.q2e4 = toInt16Scaled(data.quat2, 10000.0f, -32768.0f, 32767.0f);
  pkt.q3e4 = toInt16Scaled(data.quat3, 10000.0f, -32768.0f, 32767.0f);
  pkt.stationary = (data.motionStillCount >= data.zuptStillRequiredSamples) ? 1 : 0;
  pkt.stillCount = data.motionStillCount;

  portENTER_CRITICAL(&gPacketMux);
  gFastSeq++;
  pkt.seq = gFastSeq;
  gFastPacket = pkt;
  gFastPrimed = true;
  portEXIT_CRITICAL(&gPacketMux);
}

}  // namespace BleManager
