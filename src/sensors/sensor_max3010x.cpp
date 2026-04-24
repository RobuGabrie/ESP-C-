#ifndef HS_MAIN_COMPOSED
void __hs_sensor_max3010x_stub() {}
#else

#include "sensors/sensor_max3010x.h"
#include "heartRate.h"

static const uint8_t MAX3010X_ADDR = 0x57;
static const uint32_t MAX3010X_POLL_MS = 8;
static const uint32_t MAX3010X_FINGER_IR_MIN = 8000;
static const uint8_t MAX3010X_INTERVAL_BUF = 8;

static uint32_t gMaxLastPollMs = 0;
static float gMaxDcIr = 0.0f;
static float gMaxDcRed = 0.0f;
static float gMaxAcIr = 0.0f;
static float gMaxAcRed = 0.0f;
static uint32_t gMaxLastBeatMs = 0;
static uint16_t gBeatIntervalsMs[MAX3010X_INTERVAL_BUF] = {0};
static uint8_t gBeatIntervalCount = 0;
static uint8_t gBeatIntervalHead = 0;
static float gPrevAcIr = 0.0f;
static float gPrevDerivIr = 0.0f;
static bool gPeakDetectorSeeded = false;

static float clampfLocal(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static float computeRmssdMs() {
  if (gBeatIntervalCount < 3) {
    return NAN;
  }

  float sumSq = 0.0f;
  uint8_t diffCount = 0;
  for (uint8_t i = 1; i < gBeatIntervalCount; i++) {
    const uint8_t idxA = (uint8_t)((gBeatIntervalHead + MAX3010X_INTERVAL_BUF - i) % MAX3010X_INTERVAL_BUF);
    const uint8_t idxB = (uint8_t)((gBeatIntervalHead + MAX3010X_INTERVAL_BUF - i - 1) % MAX3010X_INTERVAL_BUF);
    const float d = (float)gBeatIntervalsMs[idxA] - (float)gBeatIntervalsMs[idxB];
    sumSq += d * d;
    diffCount++;
  }

  if (diffCount == 0) {
    return NAN;
  }
  return sqrtf(sumSq / (float)diffCount);
}

static void registerBeatAt(uint32_t beatNowMs) {
  if (gMaxLastBeatMs != 0) {
    const uint32_t ibiMs = beatNowMs - gMaxLastBeatMs;
    if (ibiMs >= 300 && ibiMs <= 2000) {
      gBeatIntervalsMs[gBeatIntervalHead] = (uint16_t)ibiMs;
      gBeatIntervalHead = (uint8_t)((gBeatIntervalHead + 1) % MAX3010X_INTERVAL_BUF);
      if (gBeatIntervalCount < MAX3010X_INTERVAL_BUF) {
        gBeatIntervalCount++;
      }

      const float bpmNow = 60000.0f / (float)ibiMs;
      if (isfinite(gBpm)) {
        gBpm = 0.7f * gBpm + 0.3f * bpmNow;
      } else {
        gBpm = bpmNow;
      }
    }
  }
  gMaxLastBeatMs = beatNowMs;
}

bool initMax3010x() {
  if (!modulePulse) {
    return false;
  }

  hasMax3010x = i2cProbe(MAX3010X_ADDR);
  if (!hasMax3010x) {
    gBpm = NAN;
    gSpo2 = NAN;
    gStressPct = NAN;
    return false;
  }

  // LED mode 2 enables both RED and IR channels.
  const bool started = max3010x.begin(Wire, I2C_SPEED_STANDARD, MAX3010X_ADDR);
  hasMax3010x = started;
  if (!hasMax3010x) {
    gBpm = NAN;
    gSpo2 = NAN;
    gStressPct = NAN;
    return false;
  }

  max3010x.setup(
    0x0C,   // ledBrightness (0x00..0xFF)
    4,      // sampleAverage
    2,      // ledMode: RED + IR
    100,    // sampleRate (Hz)
    411,    // pulseWidth
    4096    // adcRange
  );

  // Keep visible red LED dim while preserving SpO2 capability.
  max3010x.setPulseAmplitudeRed(0x08);
  max3010x.setPulseAmplitudeIR(0x40);
  max3010x.setPulseAmplitudeGreen(0x00);

  max3010x.clearFIFO();

  gMaxLastPollMs = 0;
  gMaxDcIr = 0.0f;
  gMaxDcRed = 0.0f;
  gMaxAcIr = 0.0f;
  gMaxAcRed = 0.0f;
  gMaxLastBeatMs = 0;
  gBeatIntervalCount = 0;
  gBeatIntervalHead = 0;
  gPrevAcIr = 0.0f;
  gPrevDerivIr = 0.0f;
  gPeakDetectorSeeded = false;

  gBpm = NAN;
  gSpo2 = NAN;
  gStressPct = NAN;

  return true;
}

bool readMax3010x() {
  if (!modulePulse || !hasMax3010x) {
    return false;
  }

  const uint32_t nowMs = millis();
  if (nowMs - gMaxLastPollMs < MAX3010X_POLL_MS) {
    return true;
  }
  gMaxLastPollMs = nowMs;

  max3010x.check();

  while (max3010x.available()) {
    const uint32_t red = max3010x.getRed();
    const uint32_t ir = max3010x.getIR();
    max3010x.nextSample();

    if (gMaxDcIr <= 1.0f) {
      gMaxDcIr = (float)ir;
      gMaxDcRed = (float)red;
      continue;
    }

    gMaxDcIr += 0.03f * ((float)ir - gMaxDcIr);
    gMaxDcRed += 0.03f * ((float)red - gMaxDcRed);

    const float acIr = (float)ir - gMaxDcIr;
    const float acRed = (float)red - gMaxDcRed;
    gMaxAcIr += 0.12f * (fabsf(acIr) - gMaxAcIr);
    gMaxAcRed += 0.12f * (fabsf(acRed) - gMaxAcRed);

    const bool fingerPresent = (ir >= MAX3010X_FINGER_IR_MIN);
    if (!fingerPresent) {
      gBpm = NAN;
      gSpo2 = NAN;
      gStressPct = NAN;
      gBeatIntervalCount = 0;
      gMaxLastBeatMs = 0;
      gPeakDetectorSeeded = false;
      gPrevAcIr = 0.0f;
      gPrevDerivIr = 0.0f;
      continue;
    }

    bool beatDetected = checkForBeat(ir);

    if (gPeakDetectorSeeded) {
      const float deriv = acIr - gPrevAcIr;
      const bool isPeak = (gPrevDerivIr > 0.0f) && (deriv <= 0.0f);
      const float peakThreshold = clampfLocal(gMaxAcIr * 0.35f, 80.0f, 20000.0f);
      const bool amplitudeOk = gPrevAcIr > peakThreshold;
      const uint32_t nowBeatMs = millis();
      const bool refractoryOk = (gMaxLastBeatMs == 0) || ((nowBeatMs - gMaxLastBeatMs) >= 300);

      if (isPeak && amplitudeOk && refractoryOk) {
        beatDetected = true;
      }

      gPrevDerivIr = deriv;
      gPrevAcIr = acIr;
    } else {
      gPeakDetectorSeeded = true;
      gPrevAcIr = acIr;
      gPrevDerivIr = 0.0f;
    }

    if (beatDetected) {
      registerBeatAt(millis());
    }

    if (gMaxDcIr > 1.0f && gMaxDcRed > 1.0f && gMaxAcIr > 1.0f) {
      const float ratio = (gMaxAcRed / gMaxDcRed) / (gMaxAcIr / gMaxDcIr);
      float spo2 = 110.0f - 25.0f * ratio;
      spo2 = clampfLocal(spo2, 70.0f, 100.0f);
      if (isfinite(gSpo2)) {
        gSpo2 = 0.75f * gSpo2 + 0.25f * spo2;
      } else {
        gSpo2 = spo2;
      }
    }

    if (isfinite(gBpm)) {
      const float hrPart = clampfLocal((gBpm - 62.0f) * 1.4f, 0.0f, 100.0f);
      const float rmssd = computeRmssdMs();
      const float hrvPart = isfinite(rmssd)
        ? clampfLocal(100.0f - ((rmssd - 12.0f) / (70.0f - 12.0f)) * 100.0f, 0.0f, 100.0f)
        : hrPart;
      gStressPct = clampfLocal(0.65f * hrPart + 0.35f * hrvPart, 0.0f, 100.0f);
    }
  }

  if (gMaxLastBeatMs != 0 && (millis() - gMaxLastBeatMs) > 5000) {
    gBpm = NAN;
    gStressPct = NAN;
    gBeatIntervalCount = 0;
    gMaxLastBeatMs = 0;
  }

  return true;
}

#endif
