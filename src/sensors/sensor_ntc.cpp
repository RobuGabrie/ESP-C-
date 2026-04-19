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

  gThermRaw = (uint16_t)((uint32_t)filteredMv * 4095U / 3300U);

  float vSupply = 3.3f;

  if (vAdc <= 0.05f || vAdc >= (vSupply - 0.05f)) {
    gThermValid = false;
    return gThermFilterSeeded ? gThermFiltered : -99.0f;
  }

  float rNtc = R_FIXED * ((vSupply - vAdc) / vAdc);

  Serial.printf("vAdc=%.3fV  rNtc=%.0f ohm  vSupply=%.3fV\n", vAdc, rNtc, vSupply);
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

  static uint8_t outlierCount = 0;
  if (gThermFilterSeeded) {
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
