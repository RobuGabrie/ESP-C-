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

  uint8_t mpuAddr = 0;
  if (i2cProbe(IMU_ADDR_MPU1) && i2cReadRegs(IMU_ADDR_MPU1, MPU_WHO_AM_I_REG, &who, 1)) {
    mpuAddr = IMU_ADDR_MPU1;
  } else if (i2cProbe(IMU_ADDR_MPU0) && i2cReadRegs(IMU_ADDR_MPU0, MPU_WHO_AM_I_REG, &who, 1)) {
    mpuAddr = IMU_ADDR_MPU0;
  }

  if (mpuAddr != 0) {
    if (who == MPU_WHO_AM_I_6500 || who == MPU_WHO_AM_I_9250) {
      gImuWhoAmI = who;
#if USE_MPU_DMP
      if (mpu.begin() == INV_SUCCESS) {
        mpu.enableInterrupt();
        mpu.setIntLevel(INT_ACTIVE_LOW);
        mpu.setIntLatched(INT_LATCHED);

        if (mpu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL, 10) == INV_SUCCESS) {
          mpu.dmpSetOrientation(orientationDefault);
          gImuAddr = mpuAddr;
          gImuIsMpu = true;
          gMpuDmpActive = true;
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

      if (!i2cWriteReg(mpuAddr, MPU_PWR_MGMT_1, 0x01)) return false;
      if (!i2cWriteReg(mpuAddr, MPU_PWR_MGMT_2, 0x00)) return false;
      delay(20);
      if (!i2cWriteReg(mpuAddr, MPU_CONFIG, 0x04)) return false;
      if (!i2cWriteReg(mpuAddr, MPU_SMPLRT_DIV, 0x00)) return false;
      if (!i2cWriteReg(mpuAddr, MPU_GYRO_CONFIG, 0x08)) return false;
      if (!i2cWriteReg(mpuAddr, MPU_ACCEL_CONFIG, 0x00)) return false;
      if (!i2cWriteReg(mpuAddr, MPU_INT_PIN_CFG, 0x02)) return false;
      if (!i2cWriteReg(mpuAddr, MPU_INT_ENABLE, 0x01)) return false;
      (void)calibrateMpuGyroBias(mpuAddr);

      gImuAddr = mpuAddr;
      gImuIsMpu = true;
      gMpuDmpActive = false;
      gGyroFilterSeeded = false;
      gFusionSeeded = false;
      gLastFusionUs = 0;
      return true;
    }
  }

  if (!i2cProbe(IMU_ADDR_LSM6)) return false;
  if (!i2cReadRegs(IMU_ADDR_LSM6, LSM_WHO_AM_I_REG, &who, 1)) return false;
  if (who != LSM_WHO_AM_I_VAL) return false;
  if (!i2cWriteReg(IMU_ADDR_LSM6, LSM_CTRL2_G, 0x4C)) return false;
  delay(10);
  gImuAddr = IMU_ADDR_LSM6;
  gImuWhoAmI = who;
  gImuIsMpu = false;
  gFusionSeeded = false;
  gLastFusionUs = 0;
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
