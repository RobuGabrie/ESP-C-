#ifndef HS_MAIN_COMPOSED
void __hs_utils_oled_log_stub() {}
#else

static void appendOledLog(const char* fmt, ...) {
  char line[OLED_LOG_LINE_LEN + 1];
  va_list args;
  va_start(args, fmt);
  vsnprintf(line, sizeof(line), fmt, args);
  va_end(args);

  line[OLED_LOG_LINE_LEN] = '\0';
  strncpy(gOledLog[gOledLogHead], line, OLED_LOG_LINE_LEN);
  gOledLog[gOledLogHead][OLED_LOG_LINE_LEN] = '\0';

  gOledLogHead = (gOledLogHead + 1) % OLED_LOG_LINES;
  gOledLogHasData = true;
}

static const char* gpioShortName(uint8_t pin) {
  switch (pin) {
    case 3: return "BTN+";
    case 4: return "BTN-";
    case 8: return "SDA";
    case 9: return "SCL";
    default: return "IO";
  }
}

static void updateOledRawLog() {
  gGpioLogSeq++;

  if (!gGpioLogPrimed) {
    for (size_t i = 0; i < GPIO_PUBLISH_PIN_COUNT; i++) {
      gGpioLastState[i] = digitalRead(GPIO_PUBLISH_PINS[i]);
    }
    appendOledLog("Logger pornit");
    gGpioLogPrimed = true;
    return;
  }

  int changes = 0;
  for (size_t i = 0; i < GPIO_PUBLISH_PIN_COUNT; i++) {
    const uint8_t pin = GPIO_PUBLISH_PINS[i];
    int8_t nowState = digitalRead(pin);
    if (nowState != gGpioLastState[i]) {
      appendOledLog("#%lu G%02u %s %s", (unsigned long)(gGpioLogSeq % 1000), pin,
                    gpioShortName(pin), nowState ? "ON" : "OFF");
      gGpioLastState[i] = nowState;
      changes++;
      if (changes >= 2) break;
    }
  }

  if (changes > 0) return;

  switch (gRawSnapshotPhase % 3) {
    case 0:
      appendOledLog("TH RAW:%4u T:%4.1f", gThermRaw, gTemp);
      break;
    case 1:
      appendOledLog("GYR X:%4.0f Y:%4.0f", gGyroX, gGyroY);
      break;
    default:
      appendOledLog("GYR Z:%4.0f INA:%5u", gGyroZ, gInaBusRaw);
      break;
  }
  gRawSnapshotPhase++;
}

#endif
