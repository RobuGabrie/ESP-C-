#include "sensors/sensor_oled.h"

#ifndef HS_MAIN_COMPOSED
void runOledBlePairingAnimation() {}
#else

static void drawTitleBar(const char* title) {
  oled.fillRect(0, 0, 128, 14, SSD1306_WHITE);
  oled.setTextColor(SSD1306_BLACK);
  oled.setTextSize(1);

  if (!BleManager::isConnected() && page != 3 && (millis() / 1000) % 2 == 0) {
    int wx = (128 - 13 * 6) / 2;
    oled.setCursor(max(2, wx), 3);
    oled.print("! LINK LOST !");
  } else {
    int x = (128 - (int)strlen(title) * 6) / 2;
    oled.setCursor(max(2, x), 3);
    oled.print(title);
  }

  oled.setCursor(104, 3);
  oled.printf("%d/%d", page + 1, NUM_PAGES);
  oled.setTextColor(SSD1306_WHITE);
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

static void drawBluetoothLogo(int x, int y) {
  oled.drawLine(x + 4, y, x + 4, y + 12, SSD1306_WHITE);
  oled.drawLine(x + 4, y, x + 8, y + 3, SSD1306_WHITE);
  oled.drawLine(x + 8, y + 3, x, y + 9, SSD1306_WHITE);
  oled.drawLine(x + 4, y + 12, x + 8, y + 9, SSD1306_WHITE);
  oled.drawLine(x + 8, y + 9, x, y + 3, SSD1306_WHITE);
}

static void drawArtificialHorizon(int cx, int cy, int r, float pitch, float roll) {
  oled.drawCircle(cx, cy, r, SSD1306_WHITE);

  float rad = roll * 0.0174533f;
  float s = sin(rad);
  float c = cos(rad);
  int pOff = constrain((int)(pitch * 0.5f), -(r - 2), (r - 2));

  int x1 = cx - (r - 2) * c + pOff * s;
  int y1 = cy - (r - 2) * s - pOff * c;
  int x2 = cx + (r - 2) * c + pOff * s;
  int y2 = cy + (r - 2) * s - pOff * c;

  oled.drawLine(x1, y1, x2, y2, SSD1306_WHITE);
  oled.drawPixel(cx, cy, SSD1306_WHITE);
}

static void drawCentered(const char* text, int y, int sz) {
  oled.setTextSize(sz);
  int x = (128 - (int)strlen(text) * 6 * sz) / 2;
  oled.setCursor(max(0, x), y);
  oled.print(text);
}

static void pageDashboard() {
  drawTitleBar("DASHBOARD HUD");
  oled.drawLine(60, 14, 60, 64, SSD1306_WHITE);
  oled.drawLine(60, 39, 128, 39, SSD1306_WHITE);

  oled.setTextSize(1);
  oled.setCursor(4, 18);
  oled.print("TEMP C");

  oled.setTextSize(2);
  char tBuf[8];
  snprintf(tBuf, sizeof(tBuf), "%.1f", gTemp);
  int tx = (60 - strlen(tBuf) * 12) / 2;
  oled.setCursor(max(2, tx), 36);
  oled.print(tBuf);

  drawDynamicBattery(65, 18, 20, 8, gBattPct);
  oled.setTextSize(1);
  oled.setCursor(89, 18);
  oled.printf("%.0f%%", gBattPct);

  oled.setCursor(65, 29);
  if (gCurrentMA > 10.0f) oled.print("DISCHRG");
  else if (gCurrentMA < -10.0f) oled.print("CHARGING");
  else oled.print("STABLE");

  drawBluetoothLogo(65, 46);
  oled.setCursor(76, 44);
  oled.print(BleManager::isConnected() ? "SYNCED" : "SEARCH");

  oled.setCursor(76, 54);
  oled.printf("CPU:%d%%", (int)gCpuLoad);
}

static void pageRTC() {
  drawTitleBar("MISSION TIMER");
  uint32_t currentElapsed = gTimerElapsedMs;
  if (gTimerRunning) {
    currentElapsed = millis() - gTimerStartMs;
  }

  uint32_t totalSec = currentElapsed / 1000;
  uint32_t m = totalSec / 60;
  uint32_t s = totalSec % 60;
  uint32_t ms10 = (currentElapsed % 1000) / 100;

  char mainTime[6];
  snprintf(mainTime, sizeof(mainTime), "%02lu:%02lu", m, s);

  char decTime[3];
  snprintf(decTime, sizeof(decTime), ".%lu", ms10);

  int tx = 7;
  int ty = 22;

  oled.setTextSize(3);
  oled.setCursor(tx, ty);
  oled.print(mainTime);

  oled.setTextSize(2);
  oled.setCursor(tx + 90, ty + 7);
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
  if (gTimerRunning) oled.print("ACTIVE");
  else if (currentElapsed == 0) oled.print("READY");
  else oled.print("PAUSED");

  char timeStr[9] = "--:--:--";
  if (strlen(gTimestamp) >= 19) {
    memcpy(timeStr, gTimestamp + 11, 8);
    timeStr[8] = '\0';
  }

  int lx = 128 - (8 * 6) - 2;
  oled.setCursor(lx, 54);
  oled.print(timeStr);
}

static void pageCPU() {
  drawTitleBar("SYSTEM CORE");
  char buf[10];
  snprintf(buf, sizeof(buf), "%d%%", (int)roundf(gCpuLoad));

  oled.setTextSize(3);
  int offset = (128 - strlen(buf) * 18) / 2;
  oled.setCursor(max(0, offset), 18);
  oled.print(buf);

  float pct = constrain(gCpuLoad, 0.0f, 100.0f);
  oled.drawRect(14, 44, 100, 6, SSD1306_WHITE);
  oled.fillRect(16, 46, (int)(pct / 100.0f * 96), 2, SSD1306_WHITE);

  oled.setTextSize(1);
  oled.setCursor(0, 54);
  oled.printf("Up: %lus", (unsigned long)gUptime);
  oled.setCursor(72, 54);
  oled.printf("Str: %s", moduleCpuStress ? "ON" : "OFF");
}

static void pageRawLog() {
  drawTitleBar("TELEMETRY");
  oled.setTextSize(1);
  oled.setCursor(8, 20);
  oled.printf("Therm ADC : %4u", gThermRaw);
  oled.setCursor(8, 32);
  oled.printf("Gyro X Raw: %6d", gGyroRawX);
  oled.setCursor(8, 44);
  oled.printf("Gyro Y Raw: %6d", gGyroRawY);
  oled.setCursor(8, 56);
  oled.printf("Gyro Z Raw: %6d", gGyroRawZ);
}

static void pageTemp() {
  drawTitleBar("TEMPERATURE");

  float visualHeat = constrain((gTemp + 20.0f) / 60.0f * 100.0f, 0, 100);
  int tx = 10;
  int ty = 20;
  oled.drawRoundRect(tx, ty, 10, 32, 4, SSD1306_WHITE);
  oled.fillCircle(tx + 4, ty + 34, 7, SSD1306_WHITE);
  int fillH = (int)(visualHeat / 100.0f * 26.0f);
  if (fillH > 0) {
    oled.fillRect(tx + 2, ty + 30 - fillH, 6, fillH, SSD1306_WHITE);
  }

  int wholeC = (int)gTemp;
  int decC = (int)(abs(gTemp) * 10) % 10;

  oled.setTextSize(3);
  char wBuf[6];
  snprintf(wBuf, sizeof(wBuf), "%d", wholeC);
  int wLen = strlen(wBuf) * 18;
  oled.setCursor(38, 28);
  oled.print(wBuf);

  oled.setTextSize(2);
  oled.setCursor(38 + wLen, 35);
  oled.printf(".%d", decC);

  oled.drawCircle(38 + wLen + 26, 28, 3, SSD1306_WHITE);
  oled.setCursor(38 + wLen + 32, 28);
  oled.print("C");
}

static void pageGyro() {
  drawTitleBar("ATTITUDE HUD");
  drawArtificialHorizon(64, 42, 20, gPitch, gRoll);

  char yBuf[16];
  snprintf(yBuf, sizeof(yBuf), "HDG: %03.0f", gYaw);
  drawCentered(yBuf, 16, 1);

  oled.drawRect(0, 22, 5, 40, SSD1306_WHITE);
  int pY = 42 - (int)(constrain(gPitch, -90.0f, 90.0f) / 90.0f * 18.0f);
  oled.fillRect(0, pY - 2, 8, 5, SSD1306_WHITE);

  oled.setTextSize(1);
  oled.setCursor(10, 32);
  oled.print("P");
  oled.setCursor(10, 42);
  oled.printf("%.0f", gPitch);

  oled.drawRect(123, 22, 5, 40, SSD1306_WHITE);
  int rY = 42 - (int)(constrain(gRoll, -180.0f, 180.0f) / 180.0f * 18.0f);
  oled.fillRect(120, rY - 2, 8, 5, SSD1306_WHITE);

  int rightX = 94;
  if (gRoll <= -100 || gRoll >= 100) rightX = 88;
  oled.setCursor(114, 32);
  oled.print("R");
  oled.setCursor(rightX, 42);
  oled.printf("%.0f", gRoll);
}

static void pageWiFi() {
  drawTitleBar("BLUETOOTH LINK");
  oled.setTextSize(1);

  if (BleManager::isConnected()) {
    int bx = 58;
    int by = 22;
    oled.drawLine(bx + 6, by, bx + 6, by + 18, SSD1306_WHITE);
    oled.drawLine(bx + 6, by, bx + 12, by + 4, SSD1306_WHITE);
    oled.drawLine(bx + 12, by + 4, bx, by + 13, SSD1306_WHITE);
    oled.drawLine(bx + 6, by + 18, bx + 12, by + 13, SSD1306_WHITE);
    oled.drawLine(bx + 12, by + 13, bx, by + 4, SSD1306_WHITE);

    drawCentered("CONNECTED", 48, 1);
    drawCentered("Streaming Telemetry", 56, 1);
  } else {
    oled.drawLine(44, 20, 84, 40, SSD1306_WHITE);
    oled.drawLine(44, 21, 84, 41, SSD1306_WHITE);
    oled.drawLine(84, 20, 44, 40, SSD1306_WHITE);
    oled.drawLine(84, 21, 44, 41, SSD1306_WHITE);

    drawCentered("DISCONNECTED", 48, 1);
    drawCentered("Check Phone App", 56, 1);
  }
}

static void pageVoltage() {
  drawTitleBar("POWER (VOLTS)");

  char buf[10];
  snprintf(buf, sizeof(buf), "%.2f", gVoltage);
  int numW = strlen(buf) * 18;
  int sx = max(0, (128 - numW - 14) / 2);

  oled.setTextSize(3);
  oled.setCursor(sx, 22);
  oled.print(buf);
  oled.setTextSize(2);
  oled.setCursor(sx + numW + 4, 28);
  oled.print("V");

  float pct = constrain((gVoltage - 3.0f) / 1.2f * 100.0f, 0, 100);
  oled.drawRect(14, 54, 100, 6, SSD1306_WHITE);
  oled.fillRect(16, 56, (int)(pct / 100.0f * 96), 2, SSD1306_WHITE);
}

static void pageCurrent() {
  drawTitleBar("POWER (AMPS)");

  char buf[16];
  snprintf(buf, sizeof(buf), "%.0f", gCurrentMA);
  int numW = strlen(buf) * 18;
  int sx = max(0, (128 - numW - 24) / 2);

  oled.setTextSize(3);
  oled.setCursor(sx, 22);
  oled.print(buf);
  oled.setTextSize(2);
  oled.setCursor(sx + numW + 4, 28);
  oled.print("mA");

  oled.setTextSize(1);
  char pow[24];
  snprintf(pow, sizeof(pow), "Draw: %.0f mW", gPowerMW);
  drawCentered(pow, 54, 1);
}

static void pageBattery() {
  drawTitleBar("BATTERY CELL");
  drawDynamicBattery(34, 18, 60, 24, gBattPct);

  char buf[10];
  snprintf(buf, sizeof(buf), "%.0f%%", gBattPct);
  drawCentered(buf, 48, 2);
}

static void pageI2CScanner() {
  drawTitleBar("I2C DIAGNOSTICS");
  oled.setTextSize(1);

  if (gI2cFoundCount == 0) {
    drawCentered("BUS DEAD / MISO", 32, 1);
    return;
  }

  const uint8_t maxRows = 4;
  const uint8_t totalPages = (gI2cFoundCount + maxRows - 1) / maxRows;
  const uint8_t pageIdx = (totalPages > 1) ? (uint8_t)((millis() / 2000UL) % totalPages) : 0;
  const uint8_t firstIndex = pageIdx * maxRows;

  for (uint8_t i = 0; i < maxRows; i++) {
    const uint8_t idx = firstIndex + i;
    if (idx >= gI2cFoundCount) break;
    const uint8_t addr = gI2cFound[idx];
    oled.setCursor(12, 18 + i * 11);
    oled.printf("[-] 0x%02X %s", addr, i2cDeviceName(addr));
  }
}

static void drawPage() {
  if (!hasOLED) return;

  oled.clearDisplay();
  switch (page) {
    case 0: pageDashboard(); break;
    case 1: pageTemp(); break;
    case 2: pageGyro(); break;
    case 3: pageWiFi(); break;
    case 4: pageCPU(); break;
    case 5: pageVoltage(); break;
    case 6: pageCurrent(); break;
    case 7: pageBattery(); break;
    case 8: pageRawLog(); break;
    case 9: pageI2CScanner(); break;
    case 10: pageRTC(); break;
  }
  oled.display();
}

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
