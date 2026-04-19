#pragma once

#include <Arduino.h>

namespace BleManager {

struct SensorData {
  const char* timestamp;
  uint32_t uptime;

  bool moduleTemp;
  bool thermValid;
  float tempC;

  bool moduleGyro;
  bool hasGyro;
  float gyroX;
  float gyroY;
  float gyroZ;
  float roll;
  float pitch;
  float yaw;
  float quat0;
  float quat1;
  float quat2;
  float quat3;
  uint8_t motionStillCount;
  uint8_t zuptStillRequiredSamples;

  bool moduleCurrent;
  float voltage;
  float currentMA;
  float battPct;

  bool moduleCPU;
  float cpuLoad;
};

struct ImuData {
  uint32_t uptime;
  float quat0;
  float quat1;
  float quat2;
  float quat3;
  float roll;
  float pitch;
  float yaw;
  uint8_t motionStillCount;
  uint8_t zuptStillRequiredSamples;
};

using CommandHandler = void (*)(const String& cmd);

void setCommandHandler(CommandHandler handler);
void init();
bool isConnected();
const char* deviceName();
uint32_t sensorNotifyCount();
uint32_t imuNotifyCount();

void publishSensorData(const SensorData& data);
void publishImuData(const ImuData& data);

}  // namespace BleManager
