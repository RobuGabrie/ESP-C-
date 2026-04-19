#ifndef HS_MAIN_COMPOSED
void __hs_utils_sensor_polling_stub() {}
#else

static void readOtherSensors() {
  // Time source priority: system time -> RTC -> uptime fallback.
  if (systemTimeValid()) {
    struct tm ti;
    if (getLocalTime(&ti, 50)) {
      snprintf(gTimestamp, sizeof(gTimestamp), "%04d-%02d-%02d %02d:%02d:%02d",
               ti.tm_year + 1900, ti.tm_mon + 1, ti.tm_mday,
               ti.tm_hour, ti.tm_min, ti.tm_sec);
    }
  } else if (hasRTC) {
    int year = 0;
    int month = 0;
    int day = 0;
    int hour = 0;
    int minute = 0;
    int second = 0;
    if (rtcReadDateTime(year, month, day, hour, minute, second)) {
      snprintf(gTimestamp, sizeof(gTimestamp), "%04d-%02d-%02d %02d:%02d:%02d",
               year, month, day, hour, minute, second);
    } else {
      uint32_t s = millis() / 1000;
      uint32_t hh = (s / 3600) % 24;
      uint32_t mm = (s / 60) % 60;
      uint32_t ss = s % 60;
      snprintf(gTimestamp, sizeof(gTimestamp), "1970-01-01 %02lu:%02lu:%02lu",
               (unsigned long)hh, (unsigned long)mm, (unsigned long)ss);
    }
  } else {
    uint32_t s = millis() / 1000;
    uint32_t hh = (s / 3600) % 24;
    uint32_t mm = (s / 60) % 60;
    uint32_t ss = s % 60;
    snprintf(gTimestamp, sizeof(gTimestamp), "1970-01-01 %02lu:%02lu:%02lu",
             (unsigned long)hh, (unsigned long)mm, (unsigned long)ss);
  }

  gUptime = millis() / 1000;

  if (hasINA219 && moduleCurrent) {
    uint16_t raw = 0;
    if (readINA219Register16(0x02, raw)) gInaBusRaw = raw;
    if (readINA219Register16(0x04, raw)) gInaCurrRaw = (int16_t)raw;
    if (readINA219Register16(0x03, raw)) gInaPowRaw = raw;
  }

  if (hasINA219 && moduleCurrent) {
    gVoltage = ina.getBusVoltage_V();
    gCurrentMA = ina.getCurrent_mA();
    gPowerMW = ina.getPower_mW();

    if (isnan(gVoltage) || isinf(gVoltage)) gVoltage = 0.0f;
    if (isnan(gCurrentMA) || isinf(gCurrentMA)) gCurrentMA = 0.0f;
    if (isnan(gPowerMW) || isinf(gPowerMW)) gPowerMW = 0.0f;

    if (fabsf(gCurrentMA) < BATT_CURRENT_DEADBAND_MA) {
      gCurrentMA = 0.0f;
    }
  } else if (!moduleCurrent) {
    gCurrentMA = 0.0f;
    gPowerMW = 0.0f;
    gBattLife = -1.0f;
    gBatteryPresent = false;
  }

  if (moduleTemp) {
    gTemp = readThermistor();
    if (!gThermValid) {
      appendOledLog("TH ERR GPIO%u", THERM_PIN);
    }
  }

  uint32_t nowMs = millis();
  uint32_t dtMs = nowMs - lastEnergy;
  if (lastEnergy == 0) {
    dtMs = 0;
  }
  updateBatteryStateFromIna(nowMs, dtMs);
  lastEnergy = nowMs;

  gRSSI = 0;
  gCpuLoad = measureCpuLoadPct();
  updateOledRawLog();
}

#endif
