#ifndef HS_MAIN_COMPOSED
void __hs_sensor_max3010x_stub() {}
#else

#include "sensors/sensor_max3010x.h"

static const uint8_t  MAX3010X_REG_INTR_STATUS_1 = 0x00;
static const uint8_t  MAX3010X_REG_INTR_STATUS_2 = 0x01;
static const uint8_t  MAX3010X_REG_FIFO_WR_PTR   = 0x04;
static const uint8_t  MAX3010X_REG_OVF_COUNTER   = 0x05;
static const uint8_t  MAX3010X_REG_FIFO_RD_PTR   = 0x06;
static const uint8_t  MAX3010X_REG_FIFO_DATA     = 0x07;
static const uint8_t  MAX3010X_REG_FIFO_CONFIG    = 0x08;
static const uint8_t  MAX3010X_REG_MODE_CONFIG    = 0x09;
static const uint8_t  MAX3010X_REG_SPO2_CONFIG    = 0x0A;
static const uint8_t  MAX3010X_REG_LED1_PA        = 0x0C;
static const uint8_t  MAX3010X_REG_LED2_PA        = 0x0D;
static const uint8_t  MAX3010X_REG_LED3_PA        = 0x0E;
static const uint8_t  MAX3010X_REG_PART_ID        = 0xFF;

static const uint8_t  MAX3010X_ADDR          = 0x57;
static const uint32_t MAX3010X_POLL_MS        = 8;
static const uint32_t MAX3010X_FINGER_IR_MIN  = 3000;
static const uint8_t  MAX3010X_INTERVAL_BUF   = 8;
static const uint16_t MAX3010X_IBI_MIN_MS      = 430;  // ~139 bpm max
static const uint16_t MAX3010X_IBI_MAX_MS      = 2000; // 30 bpm min

static uint32_t gMaxLastPollMs     = 0;
static float    gMaxDcIr           = 0.0f;
static float    gMaxDcRed          = 0.0f;
static float    gMaxAcIr           = 0.0f;
static float    gMaxAcRed          = 0.0f;
static uint32_t gMaxLastBeatMs     = 0;
static uint16_t gBeatIntervalsMs[MAX3010X_INTERVAL_BUF] = {0};
static uint8_t  gBeatIntervalCount = 0;
static uint8_t  gBeatIntervalHead  = 0;
static float    gPrevAcIr          = 0.0f;
static float    gPrevDerivIr       = 0.0f;
static bool     gPeakDetectorSeeded = false;
static float    gLastValidStress   = NAN;
static uint32_t gLastValidStressMs = 0;

// =============================================================================
// Direct raw-I2C reader with peak-based beat detection.
// The older library path truncated 18-bit samples through a 16-bit estimator;
// this version keeps the 18-bit FIFO data intact and uses the same peak
// detector shape that previously gave usable BPM.
// =============================================================================
static bool writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MAX3010X_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

static bool readReg(uint8_t reg, uint8_t& value) {
  Wire.beginTransmission(MAX3010X_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom((int)MAX3010X_ADDR, 1) != 1) {
    return false;
  }
  value = (uint8_t)Wire.read();
  return true;
}

static bool readBurst(uint8_t reg, uint8_t* data, size_t len) {
  Wire.beginTransmission(MAX3010X_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  const int requested = Wire.requestFrom((int)MAX3010X_ADDR, (int)len);
  if (requested != (int)len) {
    return false;
  }
  for (size_t i = 0; i < len; i++) {
    data[i] = (uint8_t)Wire.read();
  }
  return true;
}

static bool readFifoPointers(uint8_t& wrPtr, uint8_t& rdPtr) {
  return readReg(MAX3010X_REG_FIFO_WR_PTR, wrPtr) &&
         readReg(MAX3010X_REG_FIFO_RD_PTR, rdPtr);
}

static bool clearFifoPointers() {
  return writeReg(MAX3010X_REG_FIFO_WR_PTR, 0x00) &&
         writeReg(MAX3010X_REG_OVF_COUNTER, 0x00) &&
         writeReg(MAX3010X_REG_FIFO_RD_PTR, 0x00);
}

static float clampfLocal(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static bool detectBeat(float acIr, uint32_t nowMs) {
  if (!gPeakDetectorSeeded) {
    gPeakDetectorSeeded = true;
    gPrevAcIr = acIr;
    gPrevDerivIr = 0.0f;
    return false;
  }

  const float deriv = acIr - gPrevAcIr;
  const bool isPeak = (gPrevDerivIr > 0.0f) && (deriv <= 0.0f);
  const float peakThreshold = clampfLocal(gMaxAcIr * 0.35f, 80.0f, 20000.0f);
  const bool amplitudeOk = gPrevAcIr > peakThreshold;
  const bool refractoryOk = (gMaxLastBeatMs == 0) || ((nowMs - gMaxLastBeatMs) >= 300);

  gPrevDerivIr = deriv;
  gPrevAcIr = acIr;
  return isPeak && amplitudeOk && refractoryOk;
}

static void resetBeatDetection() {
  gPrevAcIr = 0.0f;
  gPrevDerivIr = 0.0f;
  gPeakDetectorSeeded = false;
}

// =============================================================================

static float medianIbiMs() {
  if (gBeatIntervalCount == 0) return NAN;

  float values[MAX3010X_INTERVAL_BUF] = {};
  for (uint8_t i = 0; i < gBeatIntervalCount; i++) {
    const uint8_t idx = (gBeatIntervalHead + MAX3010X_INTERVAL_BUF - gBeatIntervalCount + i) % MAX3010X_INTERVAL_BUF;
    values[i] = (float)gBeatIntervalsMs[idx];
  }

  for (uint8_t i = 0; i < gBeatIntervalCount; i++) {
    for (uint8_t j = i + 1; j < gBeatIntervalCount; j++) {
      if (values[j] < values[i]) {
        const float tmp = values[i];
        values[i] = values[j];
        values[j] = tmp;
      }
    }
  }

  if (gBeatIntervalCount & 1) {
    return values[gBeatIntervalCount / 2];
  }
  const uint8_t mid = gBeatIntervalCount / 2;
  return 0.5f * (values[mid - 1] + values[mid]);
}

static float computeRmssdMs() {
  if (gBeatIntervalCount < 3) return NAN;
  float sumSq = 0.0f;
  uint8_t diffCount = 0;
  for (uint8_t i = 1; i < gBeatIntervalCount; i++) {
    const uint8_t idxA = (gBeatIntervalHead + MAX3010X_INTERVAL_BUF - i)     % MAX3010X_INTERVAL_BUF;
    const uint8_t idxB = (gBeatIntervalHead + MAX3010X_INTERVAL_BUF - i - 1) % MAX3010X_INTERVAL_BUF;
    const float d = (float)gBeatIntervalsMs[idxA] - (float)gBeatIntervalsMs[idxB];
    sumSq += d * d;
    diffCount++;
  }
  if (diffCount == 0) return NAN;
  return sqrtf(sumSq / (float)diffCount);
}

static float computeAverageIbiMs() {
  if (gBeatIntervalCount == 0) return NAN;
  float sum = 0.0f;
  for (uint8_t i = 0; i < gBeatIntervalCount; i++) {
    const uint8_t idx = (gBeatIntervalHead + MAX3010X_INTERVAL_BUF - gBeatIntervalCount + i) % MAX3010X_INTERVAL_BUF;
    sum += (float)gBeatIntervalsMs[idx];
  }
  return sum / (float)gBeatIntervalCount;
}

static bool intervalLooksLikeOutlier(uint16_t ibiMs) {
  if (gBeatIntervalCount < 3) {
    return false;
  }

  const float med = medianIbiMs();
  if (!isfinite(med) || med <= 0.0f) {
    return false;
  }

  // Reject intervals that jump too far from the current rhythm estimate.
  return fabsf((float)ibiMs - med) > (0.35f * med);
}

static void registerBeatAt(uint32_t beatNowMs) {
  if (gMaxLastBeatMs != 0) {
    const uint32_t ibiMs = beatNowMs - gMaxLastBeatMs;
    if (ibiMs >= MAX3010X_IBI_MIN_MS && ibiMs <= MAX3010X_IBI_MAX_MS &&
        !intervalLooksLikeOutlier((uint16_t)ibiMs)) {
      gBeatIntervalsMs[gBeatIntervalHead] = (uint16_t)ibiMs;
      gBeatIntervalHead = (gBeatIntervalHead + 1) % MAX3010X_INTERVAL_BUF;
      if (gBeatIntervalCount < MAX3010X_INTERVAL_BUF) gBeatIntervalCount++;

      const float robustIbiMs = medianIbiMs();
      if (isfinite(robustIbiMs) && robustIbiMs > 0.0f) {
        const float bpmNow = 60000.0f / robustIbiMs;
        if (gBeatIntervalCount >= 3) {
          gBpm = isfinite(gBpm) ? 0.88f * gBpm + 0.12f * bpmNow : bpmNow;
        }
      }
    }
  }
  gMaxLastBeatMs = beatNowMs;
}

bool initMax3010x() {
  if (!modulePulse) return false;

  hasMax3010x = i2cProbe(MAX3010X_ADDR);
  if (!hasMax3010x) { gBpm = NAN; gSpo2 = NAN; gStressPct = NAN; return false; }

  uint8_t partId = 0;
  if (!readReg(MAX3010X_REG_PART_ID, partId)) {
    gBpm = NAN; gSpo2 = NAN; gStressPct = NAN; return false;
  }

  // Soft reset.
  if (!writeReg(MAX3010X_REG_MODE_CONFIG, 0x40)) {
    gBpm = NAN; gSpo2 = NAN; gStressPct = NAN; return false;
  }
  delay(50);

  // Configure for RED + IR pulse oximetry.
  if (!writeReg(MAX3010X_REG_INTR_STATUS_1, 0x00) ||
      !writeReg(MAX3010X_REG_INTR_STATUS_2, 0x00) ||
      !writeReg(MAX3010X_REG_FIFO_CONFIG, 0x5F) ||
      !writeReg(MAX3010X_REG_MODE_CONFIG, 0x03) ||
      !writeReg(MAX3010X_REG_SPO2_CONFIG, 0x27) ||
      !writeReg(MAX3010X_REG_LED1_PA, 0x08) ||
      !writeReg(MAX3010X_REG_LED2_PA, 0x40) ||
      !writeReg(MAX3010X_REG_LED3_PA, 0x00) ||
      !clearFifoPointers()) {
    gBpm = NAN; gSpo2 = NAN; gStressPct = NAN; return false;
  }

  resetBeatDetection();
  gMaxLastPollMs     = 0;
  gMaxDcIr           = 0.0f;
  gMaxDcRed          = 0.0f;
  gMaxAcIr           = 0.0f;
  gMaxAcRed          = 0.0f;
  gMaxLastBeatMs     = 0;
  gBeatIntervalCount = 0;
  gBeatIntervalHead  = 0;
  gLastValidStress   = NAN;
  gLastValidStressMs = 0;
  gBpm = NAN; gSpo2 = NAN; gStressPct = NAN;
  return true;
}

bool readMax3010x() {
  if (!modulePulse || !hasMax3010x) return false;

  const uint32_t nowMs = millis();
  if (nowMs - gMaxLastPollMs < MAX3010X_POLL_MS) return true;
  gMaxLastPollMs = nowMs;

  // Rate-limited diagnostic: print every 2 s so we can see what the sensor
  // is actually reporting without flooding the serial port.
  static uint32_t sDbgMs = 0;
  const bool printDbg = (nowMs - sDbgMs >= 2000);
  if (printDbg) sDbgMs = nowMs;

  uint8_t wrPtr = 0;
  uint8_t rdPtr = 0;
  if (!readFifoPointers(wrPtr, rdPtr)) {
    return false;
  }

  uint8_t samplesAvailable = (wrPtr - rdPtr) & 0x1F;
  while (samplesAvailable--) {
    uint8_t raw[6] = {};
    if (!readBurst(MAX3010X_REG_FIFO_DATA, raw, sizeof(raw))) {
      break;
    }

    const uint32_t red = (((uint32_t)raw[0] << 16) |
                          ((uint32_t)raw[1] << 8) |
                          (uint32_t)raw[2]) & 0x03FFFFu;
    const uint32_t ir  = (((uint32_t)raw[3] << 16) |
                         ((uint32_t)raw[4] << 8) |
                         (uint32_t)raw[5]) & 0x03FFFFu;

    // Seed the floating-point DC on the very first sample (no-finger level).
    if (gMaxDcIr <= 1.0f) {
      gMaxDcIr  = (float)ir;
      gMaxDcRed = (float)red;
      continue;
    }

    gMaxDcIr  += 0.03f * ((float)ir  - gMaxDcIr);
    gMaxDcRed += 0.03f * ((float)red - gMaxDcRed);

    const float acIr  = (float)ir  - gMaxDcIr;
    const float acRed = (float)red - gMaxDcRed;
    gMaxAcIr  += 0.12f * (fabsf(acIr)  - gMaxAcIr);
    gMaxAcRed += 0.12f * (fabsf(acRed) - gMaxAcRed);

    const uint32_t fingerMin = (uint32_t)fmaxf((float)MAX3010X_FINGER_IR_MIN,
                          gMaxDcIr * 0.35f);
    const bool fingerPresent = (gMaxDcIr >= (float)MAX3010X_FINGER_IR_MIN) ||
                   (ir >= fingerMin);

    if (printDbg) {
      Serial.printf("[MAX] ir=%lu red=%lu dc=%.0f fingerMin=%lu finger=%d "
                    "acIr=%.0f bpm=%.1f spo2=%.1f\n",
                    ir, red, gMaxDcIr, fingerMin, (int)fingerPresent,
                    gMaxAcIr,
                    isfinite(gBpm) ? gBpm : -1.0f,
                    isfinite(gSpo2) ? gSpo2 : -1.0f);
    }

    if (!fingerPresent) {
      gBpm = NAN; gSpo2 = NAN; gStressPct = NAN;
      gLastValidStress   = NAN;
      gLastValidStressMs = 0;
      gBeatIntervalCount = 0;
      gMaxLastBeatMs     = 0;
      resetBeatDetection();
      continue;
    }

    const bool beatDetected = detectBeat(acIr, nowMs);

    if (beatDetected) {
      const uint32_t nowBeat = millis();
      Serial.printf("[MAX] BEAT ibi=%lums bpm=%.1f\n",
                    gMaxLastBeatMs ? (nowBeat - gMaxLastBeatMs) : 0UL,
                    isfinite(gBpm) ? gBpm : 0.0f);
      if (gMaxLastBeatMs == 0 || (nowBeat - gMaxLastBeatMs) >= 280) {
        registerBeatAt(nowBeat);
      }
    }

    // SpO2 from ratio of AC/DC for red and IR.
    if (gMaxDcIr > 1.0f && gMaxDcRed > 1.0f && gMaxAcIr > 1.0f) {
      const float ratio = (gMaxAcRed / gMaxDcRed) / (gMaxAcIr / gMaxDcIr);
      float spo2 = 110.0f - 25.0f * ratio;
      spo2 = clampfLocal(spo2, 70.0f, 100.0f);
      gSpo2 = isfinite(gSpo2) ? 0.94f * gSpo2 + 0.06f * spo2 : spo2;
    }

    if (isfinite(gBpm)) {
      const float hrPart  = clampfLocal((gBpm - 62.0f) * 1.4f, 0.0f, 100.0f);
      const float rmssd   = computeRmssdMs();
      const float hrvPart = isfinite(rmssd)
        ? clampfLocal(100.0f - ((rmssd - 12.0f) / 58.0f) * 100.0f, 0.0f, 100.0f)
        : hrPart;
      const float stressNow = clampfLocal(0.65f * hrPart + 0.35f * hrvPart, 0.0f, 100.0f);
      gStressPct         = isfinite(gStressPct) ? 0.9f * gStressPct + 0.1f * stressNow : stressNow;
      gLastValidStress   = gStressPct;
      gLastValidStressMs = millis();
    }
  }

  // Timeout: if no beat for 9 s, treat as signal lost.
  const uint32_t nowCheck = millis();
  if (gMaxLastBeatMs != 0 && (nowCheck - gMaxLastBeatMs) > 9000) {
    gBpm = NAN;
    gBeatIntervalCount = 0;
    gMaxLastBeatMs     = 0;
  }
  // Hold last valid stress for 4 s after BPM is lost to avoid a NAN flash.
  if (!isfinite(gBpm)) {
    gStressPct = (gLastValidStressMs != 0 && (nowCheck - gLastValidStressMs) <= 4000)
               ? gLastValidStress : NAN;
  }

  return true;
}

#endif
