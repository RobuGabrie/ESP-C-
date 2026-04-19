#ifndef HS_MAIN_COMPOSED
void __hs_utils_json_stub() {}
#else

static size_t buildJSON(char* buf, size_t bufSize) {
  JsonDocument doc;

  doc["ts"] = gTimestamp;
  doc["up"] = gUptime;
  if (moduleTemp && gThermValid) {
    float ntcTemp = roundf(gTemp * 10.0f) / 10.0f;
    doc["temp"] = ntcTemp;
    doc["temperature"] = ntcTemp;
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

  doc["rssi"] = gRSSI;
  doc["cpu"] = (int)roundf(gCpuLoad);

  if (moduleCurrent) doc["v"] = roundf(gVoltage * 100.0f) / 100.0f;
  else doc["v"] = nullptr;

  if (moduleCurrent) doc["ma"] = roundf(gCurrentMA * 10.0f) / 10.0f;
  else doc["ma"] = nullptr;

  if (moduleCurrent) doc["mw"] = (int)roundf(gPowerMW);
  else doc["mw"] = nullptr;

  if (moduleCurrent) doc["mah"] = roundf((gBatteryPresent ? gTotalMAh : 0.0f) * 100.0f) / 100.0f;
  else doc["mah"] = nullptr;

  if (moduleCurrent) doc["batt"] = roundf(gBattPct * 10.0f) / 10.0f;
  else doc["batt"] = nullptr;

  doc["batt_min"] = (moduleCurrent && gBatteryPresent && gBattLife > 0) ? (int)gBattLife : -1;
  doc["ble_connected"] = BleManager::isConnected();
  doc["device_name"] = BleManager::deviceName();

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
  else inaRaw["bus"] = nullptr;
  if (moduleCurrent) inaRaw["current"] = gInaCurrRaw;
  else inaRaw["current"] = nullptr;
  if (moduleCurrent) inaRaw["power"] = gInaPowRaw;
  else inaRaw["power"] = nullptr;

  return serializeJson(doc, buf, bufSize);
}

#endif
