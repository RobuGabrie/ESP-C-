#pragma once

#include <Arduino.h>

namespace BleManager {

#pragma pack(push, 1)

struct __attribute__((packed)) FastPacket {
  uint32_t uptimeMs;
  int16_t rollCdeg;
  int16_t pitchCdeg;
  int16_t yawCdeg;
  int16_t gxDps10;
  int16_t gyDps10;
  int16_t gzDps10;
  int16_t axMg;
  int16_t ayMg;
  int16_t azMg;
  int16_t q0e4;
  int16_t q1e4;
  int16_t q2e4;
  int16_t q3e4;
  uint8_t stationary;
  uint8_t stillCount;
  uint8_t seq;
};

struct __attribute__((packed)) SlowPacket {
  uint32_t uptimeMs;
  uint16_t rtcYear;
  uint8_t rtcMonth;
  uint8_t rtcDay;
  uint8_t rtcHour;
  uint8_t rtcMinute;
  uint8_t rtcSecond;
  int16_t tempCenti;
  uint16_t voltageCenti;
  int16_t currentMa;
  uint16_t battPct10;
  uint16_t cpuPct10;
  uint16_t bpm10;
  uint16_t spo210;
  uint16_t stressPct10;
  uint8_t flags;
  uint8_t seq;
};

#pragma pack(pop)

#ifndef __INTELLISENSE__
static_assert(sizeof(SlowPacket) == 29, "SlowPacket size mismatch: expected 29 bytes");
#endif

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

  bool modulePulse;
  bool pulseValid;
  float bpm;
  float spo2;
  float stressPct;

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
  float accelX;
  float accelY;
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;
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
