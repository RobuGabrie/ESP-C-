#ifndef HS_MAIN_COMPOSED
void __hs_utils_battery_stub() {}
#else

static void loadBatteryState() {
  if (!prefs.begin(PREF_NS, false)) return;
  gHasPersistedBatteryState = prefs.isKey(PREF_KEY_USED_MAH);
  if (gHasPersistedBatteryState) {
    gTotalMAh = prefs.getFloat(PREF_KEY_USED_MAH, gTotalMAh);
  }
  prefs.end();
  gTotalMAh = constrain(gTotalMAh, 0.0f, BATTERY_CAPACITY);

  // If no persisted value exists, SOC will be bootstrapped from measured voltage.
  if (!gHasPersistedBatteryState) {
    gBootstrapFromVoltageDone = false;
  }
}

static void persistBatteryStateIfDue(uint32_t nowMs) {
  if (nowMs - gLastBattPersistMs < BATT_SAVE_INTERVAL_MS) return;
  gLastBattPersistMs = nowMs;

  if (!prefs.begin(PREF_NS, false)) return;
  prefs.putFloat(PREF_KEY_USED_MAH, gTotalMAh);
  prefs.end();
}

static float ocvToSocPct(float v) {
  static const float vPts[] = {4.20f, 4.10f, 4.00f, 3.90f, 3.80f, 3.70f, 3.60f, 3.50f, 3.40f, 3.30f, 3.20f};
  static const float sPts[] = {100.0f, 90.0f, 75.0f, 60.0f, 45.0f, 30.0f, 18.0f, 10.0f, 5.0f, 2.0f, 0.0f};

  if (v >= vPts[0]) return 100.0f;
  if (v <= vPts[10]) return 0.0f;

  for (int i = 0; i < 10; i++) {
    if (v <= vPts[i] && v >= vPts[i + 1]) {
      float t = (v - vPts[i + 1]) / (vPts[i] - vPts[i + 1]);
      return sPts[i + 1] + t * (sPts[i] - sPts[i + 1]);
    }
  }
  return 0.0f;
}

static void updateBatteryStateFromIna(uint32_t nowMs, uint32_t dtMs) {
  // If Preferences was reset/missing, seed SOC from OCV instead of defaulting to 100%.
  if (!gHasPersistedBatteryState && !gBootstrapFromVoltageDone && gVoltage > 2.5f) {
    float socFromOcv = ocvToSocPct(gVoltage);
    gSocEstimatePct = constrain(socFromOcv, 0.0f, 100.0f);
    gTotalMAh = BATTERY_CAPACITY * (1.0f - gSocEstimatePct / 100.0f);
    gBattPctShown = (int)roundf(gSocEstimatePct);
    gBootstrapFromVoltageDone = true;
  }

  // Coulomb counting
  if (moduleCurrent) {
    float dtHours = dtMs / 3600000.0f;
    float deltaMAh = gCurrentMA * dtHours;  // positive = discharge, negative = charging
    gTotalMAh = constrain(gTotalMAh + deltaMAh, 0.0f, BATTERY_CAPACITY);
    persistBatteryStateIfDue(nowMs);
  }

  // Battery SOC estimation (hybrid: coulomb + adaptive OCV correction)
  if (moduleCurrent) {
    if (gVoltage <= BATT_PRESENT_OFF_V) {
      gBattLowAccumMs += dtMs;
    } else {
      gBattLowAccumMs = 0;
    }

    if (gVoltage >= BATT_PRESENT_ON_V) {
      gBattHighAccumMs += dtMs;
    } else {
      gBattHighAccumMs = 0;
    }

    if (gBatteryPresent) {
      if (gBattLowAccumMs >= BATT_PRESENCE_DEBOUNCE_MS) {
        gBatteryPresent = false;
      }
    } else {
      if (gBattHighAccumMs >= BATT_PRESENCE_DEBOUNCE_MS) {
        gBatteryPresent = true;
      }
    }
  }

  float socFromCount = 100.0f - (gTotalMAh / BATTERY_CAPACITY) * 100.0f;
  socFromCount = constrain(socFromCount, 0.0f, 100.0f);

  bool nearRest = fabsf(gCurrentMA) <= BATT_REST_CURRENT_MA;
  if (nearRest) gRestAccumMs += dtMs;
  else gRestAccumMs = 0;

  if (moduleCurrent && gBatteryPresent) {
    float socFromOcv = ocvToSocPct(gVoltage);
    float mismatchPct = fabsf(socFromCount - socFromOcv);
    bool longRest = nearRest && gRestAccumMs >= BATT_REST_TIME_MS;

    float ocvWeight = 0.05f;
    if (nearRest) ocvWeight = 0.15f;
    if (longRest) ocvWeight = 0.35f;

    if (mismatchPct >= BATT_OCV_MISMATCH_RECOVER_PCT) {
      ocvWeight = max(ocvWeight, longRest ? 0.50f : 0.25f);
    }

    gSocEstimatePct = (1.0f - ocvWeight) * socFromCount + ocvWeight * socFromOcv;
    gTotalMAh = BATTERY_CAPACITY * (1.0f - gSocEstimatePct / 100.0f);
    gTotalMAh = constrain(gTotalMAh, 0.0f, BATTERY_CAPACITY);
  } else if (moduleCurrent) {
    gSocEstimatePct = 0.0f;
  }

  gSocEstimatePct = constrain(gSocEstimatePct, 0.0f, 100.0f);

  int targetPct = (int)roundf(gSocEstimatePct);
  if (moduleCurrent && gCurrentMA >= 5.0f) {
    if (targetPct < gBattPctShown) gBattPctShown = targetPct;
  } else if (moduleCurrent && gCurrentMA <= -20.0f) {
    if (targetPct > gBattPctShown) gBattPctShown = targetPct;
  } else {
    gBattPctShown = targetPct;
  }
  gBattPctShown = constrain(gBattPctShown, 0, 100);
  gBattPct = (float)gBattPctShown;

  float remaining = BATTERY_CAPACITY * (gSocEstimatePct / 100.0f);
  gBattLife = (moduleCurrent && gBatteryPresent && fabsf(gCurrentMA) > 0.1f)
    ? (remaining / fabsf(gCurrentMA)) * 60.0f
    : -1.0f;

  if (moduleCurrent && !gBatteryPresent) {
    gCurrentMA = 0.0f;
    gPowerMW = 0.0f;
    gBattLife = -1.0f;
  }
}

#endif
