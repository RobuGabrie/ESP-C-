#include "sensors/sensor_oled.h"

#ifndef HS_MAIN_COMPOSED
void runOledBlePairingAnimation() {}
#else

// =============================================================================
// Shared drawing helpers
// =============================================================================

static void drawTitleBar(const char* title) {
  oled.fillRect(0, 0, 128, 14, SSD1306_WHITE);
  oled.setTextColor(SSD1306_BLACK);
  oled.setTextSize(1);

  // Flash "LINK LOST" on every page except the timer (page 1) so it
  // does not cover the large countdown digits.
  if (!BleManager::isConnected() && page != 1 && (millis() / 1000) % 2 == 0) {
    int wx = (128 - 13 * 6) / 2;
    oled.setCursor(max(2, wx), 3);
    oled.print("! LINK LOST !");
  } else {
    int x = (128 - (int)strlen(title) * 6) / 2;
    oled.setCursor(max(2, x), 3);
    oled.print(title);
  }

  oled.setCursor(102, 3);
  oled.printf("%d/%d", page + 1, NUM_PAGES);
  if (BleManager::isConnected()) {
    oled.fillCircle(96, 7, 2, SSD1306_BLACK);
  } else {
    oled.drawCircle(96, 7, 2, SSD1306_BLACK);
  }
  oled.setTextColor(SSD1306_WHITE);
  oled.drawLine(0, 14, 127, 14, SSD1306_WHITE);
}

static void drawDynamicBattery(int x, int y, int w, int h, float pct) {
  pct = constrain(pct, 0.0f, 100.0f);
  oled.drawRect(x, y, w - 2, h, SSD1306_WHITE);
  oled.fillRect(x + w - 2, y + h / 4, 2, h / 2, SSD1306_WHITE);
  int fillW = (int)((pct / 100.0f) * (w - 6));
  if (fillW > 0) {
    oled.fillRect(x + 2, y + 2, fillW, h - 4, SSD1306_WHITE);
  }
}

static void drawCentered(const char* text, int y, int sz) {
  oled.setTextSize(sz);
  int x = (128 - (int)strlen(text) * 6 * sz) / 2;
  oled.setCursor(max(0, x), y);
  oled.print(text);
}

static void drawProgressBar(int x, int y, int w, int h, float pct) {
  float cPct = constrain(pct, 0.0f, 100.0f);
  oled.drawRect(x, y, w, h, SSD1306_WHITE);
  const int innerW = w - 4;
  const int innerH = h - 4;
  const int fillW  = (int)(innerW * (cPct / 100.0f));
  if (fillW > 0) {
    oled.fillRect(x + 2, y + 2, fillW, innerH, SSD1306_WHITE);
  }
  oled.drawFastVLine(x + w / 2, y + 1, h - 2, SSD1306_WHITE);
}

// =============================================================================
// Page 0 — OVERVIEW
// Layout (128×64):
//   y=0-13  : title bar
//   y=16    : T:XX.XC            BAT:XXX%
//   y=26    : CPU:XX%            BLE:ON/OFF
//   y=35    : horizontal divider ─────────────
//   y=36-53 : [XXX BPM] | vert | SpO2:XX%
//                                STR: XX%
//   y=57    : hh:mm:ss                  LIVE
// =============================================================================
static void pageDashboard() {
  drawTitleBar("OVERVIEW");

  oled.setTextSize(1);

  // Row 1 — temperature (left) and battery (right)
  oled.setCursor(2, 16);
  oled.printf("T:%.1fC", gTemp);
  char batBuf[10];
  snprintf(batBuf, sizeof(batBuf), "BAT:%3.0f%%", gBattPct);
  oled.setCursor(128 - (int)strlen(batBuf) * 6 - 2, 16);
  oled.print(batBuf);

  // Row 2 — CPU (left) and BLE state (right)
  oled.setCursor(2, 26);
  oled.printf("CPU:%3.0f%%", gCpuLoad);
  oled.setCursor(80, 26);
  oled.printf("BLE:%s", BleManager::isConnected() ? "ON" : "OFF");

  // Horizontal divider
  oled.drawLine(0, 35, 127, 35, SSD1306_WHITE);

  // BPM — large digits left of the vertical divider (x=4..40, y=38..54)
  oled.setTextSize(2);
  oled.setCursor(4, 38);
  if (isfinite(gBpm)) {
    oled.printf("%3.0f", gBpm);
  } else {
    oled.print("---");
  }
  oled.setTextSize(1);
  oled.setCursor(44, 46);
  oled.print("BPM");

  // Vertical divider between BPM and vitals
  oled.drawFastVLine(63, 36, 18, SSD1306_WHITE);

  // SpO2 and stress — right of divider
  oled.setCursor(67, 38);
  if (isfinite(gSpo2)) {
    oled.printf("SpO2:%2.0f%%", gSpo2);
  } else {
    oled.print("SpO2: --");
  }
  oled.setCursor(67, 48);
  if (isfinite(gStressPct)) {
    oled.printf("STR: %2.0f%%", gStressPct);
  } else {
    oled.print("STR:  --");
  }

  // Bottom row — time and connection status
  const char* tm = (strlen(gTimestamp) >= 19) ? (gTimestamp + 11) : "--:--:--";
  oled.setCursor(2, 57);
  oled.print(tm);
  oled.setCursor(92, 57);
  oled.print(BleManager::isConnected() ? "LIVE" : "SRCH");
}

// =============================================================================
// Page 1 — MISSION TIMER  (unchanged)
// =============================================================================
static void pageRTC() {
  drawTitleBar("MISSION TIMER");
  uint32_t currentElapsed = gTimerElapsedMs;
  if (gTimerRunning) {
    currentElapsed = millis() - gTimerStartMs;
  }

  uint32_t totalSec = currentElapsed / 1000;
  uint32_t m   = totalSec / 60;
  uint32_t s   = totalSec % 60;
  uint32_t ms10 = (currentElapsed % 1000) / 100;

  char mainTime[6];
  snprintf(mainTime, sizeof(mainTime), "%02lu:%02lu", m, s);
  char decTime[3];
  snprintf(decTime, sizeof(decTime), ".%lu", ms10);

  oled.setTextSize(3);
  oled.setCursor(7, 22);
  oled.print(mainTime);
  oled.setTextSize(2);
  oled.setCursor(97, 29);
  oled.print(decTime);

  if (gTimerRunning) {
    if ((millis() / 500) % 2 == 0) {
      oled.fillCircle(120, 26, 3, SSD1306_WHITE);
    }
  } else if (currentElapsed > 0) {
    oled.fillRect(117, 24, 2, 6, SSD1306_WHITE);
    oled.fillRect(121, 24, 2, 6, SSD1306_WHITE);
  }

  oled.drawLine(0, 48, 128, 48, SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(2, 54);
  if (gTimerRunning)       oled.print("ACTIVE");
  else if (currentElapsed == 0) oled.print("READY");
  else                     oled.print("PAUSED");

  char timeStr[9] = "--:--:--";
  if (strlen(gTimestamp) >= 19) {
    memcpy(timeStr, gTimestamp + 11, 8);
    timeStr[8] = '\0';
  }
  oled.setCursor(128 - 8 * 6 - 2, 54);
  oled.print(timeStr);
}

// =============================================================================
// Page 2 — TEMPERATURE  (unchanged)
// =============================================================================
static void pageTemp() {
  drawTitleBar("TEMPERATURE");

  char buf[10];
  snprintf(buf, sizeof(buf), "%.1f", gTemp);

  oled.setTextSize(3);
  int numW = (int)strlen(buf) * 18;
  int sx   = max(0, (128 - numW - 18) / 2);
  oled.setCursor(sx, 22);
  oled.print(buf);

  oled.setTextSize(2);
  oled.setCursor(sx + numW + 2, 26);
  oled.print("\xF7""C");

  float pct = constrain((gTemp + 20.0f) / 60.0f * 100.0f, 0.0f, 100.0f);
  drawProgressBar(14, 50, 100, 8, pct);
  oled.setTextSize(1);
  oled.setCursor(2, 51);
  oled.print("-20");
  oled.setCursor(56, 58);
  oled.print("10");
  oled.setCursor(112, 51);
  oled.print("40");
}

// =============================================================================
// Page 3 — PULSE OX
// Layout:
//   y=0-13  : title bar
//   y=15-44 : [BPM box left] [SpO2 box right]  (each h=30)
//   y=47-55 : STR: [progress bar] XX%
//             "Place finger" when no signal
// =============================================================================
static void pagePulseOx() {
  drawTitleBar("PULSE OX");

  if (!modulePulse || !hasMax3010x) {
    drawCentered("MAX3010x NOT FOUND", 28, 1);
    drawCentered("Check 0x57 wiring", 40, 1);
    return;
  }

  const bool hasBpm  = isfinite(gBpm);
  const bool hasSpo2 = isfinite(gSpo2);
  const bool hasStr  = isfinite(gStressPct);

  // BPM box (left: x=0..61, y=15..44)
  oled.drawRect(0, 15, 62, 30, SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(4, 17);
  oled.print("BPM");
  oled.setTextSize(2);
  oled.setCursor(4, 26);
  if (hasBpm) {
    char b[5]; snprintf(b, sizeof(b), "%3.0f", gBpm);
    oled.print(b);
  } else {
    oled.print("---");
  }

  // SpO2 box (right: x=65..127, y=15..44)
  oled.drawRect(65, 15, 63, 30, SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(69, 17);
  oled.print("SpO2");
  oled.setTextSize(2);
  oled.setCursor(69, 26);
  if (hasSpo2) {
    char s[5]; snprintf(s, sizeof(s), "%2.0f", gSpo2);
    oled.print(s);
    oled.setTextSize(1);
    oled.setCursor(105, 31);
    oled.print("%");
  } else {
    oled.print("--");
    oled.setTextSize(1);
    oled.setCursor(105, 31);
    oled.print("%");
  }

  // Stress row — "STR" label, bar, and percentage (y=47..55)
  // When no finger: single centred prompt instead.
  oled.setTextSize(1);
  if (!hasBpm) {
    drawCentered("Place finger", 50, 1);
  } else {
    oled.setCursor(2, 48);
    oled.print("STR");
    drawProgressBar(22, 47, 84, 9, hasStr ? gStressPct : 0.0f);
    oled.setCursor(109, 48);
    if (hasStr) {
      char sv[5]; snprintf(sv, sizeof(sv), "%2.0f%%", gStressPct);
      oled.print(sv);
    } else {
      oled.print("--");
    }
  }
}

// =============================================================================
// Page 4 — BATTERY %
// =============================================================================
static void pageBatteryPct() {
  drawTitleBar("BATTERY");

  // Battery icon centred horizontally (w=60, h=22, y=15..37)
  drawDynamicBattery(34, 15, 60, 22, gBattPct);

  // Large percentage below the icon (y=40..56)
  char pctBuf[7];
  snprintf(pctBuf, sizeof(pctBuf), "%.0f%%", gBattPct);
  drawCentered(pctBuf, 40, 2);

  // Voltage and current on the bottom line
  char line[22];
  snprintf(line, sizeof(line), "%.2fV  %.0fmA", gVoltage, gCurrentMA);
  drawCentered(line, 57, 1);
}

// =============================================================================
// Page 5 — POWER (voltage, current, wattage)
// =============================================================================
static void pagePower() {
  drawTitleBar("POWER");

  // Voltage (left) and current (right) on the top row
  oled.setTextSize(1);
  oled.setCursor(2, 16);
  oled.printf("V:%.2fV", gVoltage);
  char iBuf[12];
  snprintf(iBuf, sizeof(iBuf), "I:%.0fmA", gCurrentMA);
  oled.setCursor(128 - (int)strlen(iBuf) * 6 - 2, 16);
  oled.print(iBuf);

  oled.drawLine(0, 26, 127, 26, SSD1306_WHITE);

  // Power draw — large centred value
  drawCentered("POWER", 29, 1);
  char pBuf[14];
  snprintf(pBuf, sizeof(pBuf), "%.0f mW", gPowerMW);
  drawCentered(pBuf, 37, 2);

  // Charge state
  const char* state = (gCurrentMA > 10.0f) ? "DISCHARGING"
                    : (gCurrentMA < -10.0f) ? "CHARGING"
                    : "STABLE";
  drawCentered(state, 56, 1);
}

// =============================================================================
// Page 6 — SYSTEM STATUS  (unchanged)
// =============================================================================
static void pageCPU() {
  drawTitleBar("SYSTEM STATUS");
  char buf[10];
  snprintf(buf, sizeof(buf), "%d%%", (int)roundf(gCpuLoad));

  oled.setTextSize(3);
  int offset = (128 - (int)strlen(buf) * 18) / 2;
  oled.setCursor(max(0, offset), 18);
  oled.print(buf);

  float pct = constrain(gCpuLoad, 0.0f, 100.0f);
  oled.drawRect(14, 44, 100, 6, SSD1306_WHITE);
  oled.fillRect(16, 46, (int)(pct / 100.0f * 96), 2, SSD1306_WHITE);

  oled.setTextSize(1);
  oled.setCursor(0, 54);
  oled.printf("Up: %lus", (unsigned long)gUptime);
  oled.setCursor(68, 54);
  oled.printf("BLE: %s", BleManager::isConnected() ? "ON" : "OFF");
}

// =============================================================================
// Page dispatcher
// 0=Overview  1=Timer  2=Temp  3=PulseOx  4=Battery%  5=Power  6=CPU
// =============================================================================
static void drawPage() {
  if (!hasOLED) return;

  oled.clearDisplay();
  switch (page) {
    case 0: pageDashboard();  break;
    case 1: pageRTC();        break;
    case 2: pageTemp();       break;
    case 3: pagePulseOx();    break;
    case 4: pageBatteryPct(); break;
    case 5: pagePower();      break;
    case 6: pageCPU();        break;
  }
  oled.display();
}

// =============================================================================
// BLE pairing splash  (unchanged)
// =============================================================================
void runOledBlePairingAnimation() {
  if (!hasOLED) return;

  int radius = 0;
  while (!BleManager::isConnected()) {
    oled.clearDisplay();

    oled.fillRect(0, 0, 128, 14, SSD1306_WHITE);
    oled.setTextColor(SSD1306_BLACK);
    oled.setTextSize(1);
    int tx = (128 - 11 * 6) / 2;
    oled.setCursor(max(0, tx), 3);
    oled.print("BLE PAIRING");
    oled.setTextColor(SSD1306_WHITE);

    oled.drawCircle(64, 32, 6 + radius, SSD1306_WHITE);
    if (radius > 8) oled.drawCircle(64, 32, radius - 8, SSD1306_WHITE);

    oled.drawLine(64, 26, 64, 38, SSD1306_WHITE);
    oled.drawLine(64, 26, 68, 29, SSD1306_WHITE);
    oled.drawLine(68, 29, 60, 35, SSD1306_WHITE);
    oled.drawLine(64, 38, 68, 35, SSD1306_WHITE);
    oled.drawLine(68, 35, 60, 29, SSD1306_WHITE);

    if ((millis() / 500) % 2 == 0) {
      int w = 18 * 6;
      oled.setCursor((128 - w) / 2, 54);
      oled.print("Waiting for App...");
    }

    oled.display();
    radius++;
    if (radius > 20) radius = 0;
    delay(40);
  }

  oled.clearDisplay();
  oled.fillRect(0, 0, 128, 64, SSD1306_WHITE);
  oled.setTextColor(SSD1306_BLACK);

  oled.setTextSize(2);
  int lx = (128 - 4 * 12) / 2;
  oled.setCursor(lx, 16);
  oled.print("LINK");

  oled.setTextSize(1);
  int ex = (128 - 11 * 6) / 2;
  oled.setCursor(ex, 38);
  oled.print("ESTABLISHED");

  oled.display();
  oled.setTextColor(SSD1306_WHITE);
  delay(1500);
}

#endif
