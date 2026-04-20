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

    float ex = (ayn * gravZ - azn * gravY);
    float ey = (azn * gravX - axn * gravZ);
    float ez = (axn * gravY - ayn * gravX);
    wx += QUAT_ACC_GAIN * ex;
    wy += QUAT_ACC_GAIN * ey;
    wz += QUAT_ACC_GAIN * ez;
  }

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
  gHeading = gYaw;
}
