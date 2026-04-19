static uint8_t bcdToBin(uint8_t v) {
  return (uint8_t)((v & 0x0F) + ((v >> 4) * 10));
}

static uint8_t binToBcd(uint8_t v) {
  return (uint8_t)(((v / 10) << 4) | (v % 10));
}

static bool rtcWriteDateTime(int year, int month, int day, int hour, int minute, int second) {
  if (!hasRTC || gRtcAddr == 0) return false;

  uint8_t yy = (uint8_t)constrain(year - 2000, 0, 99);
  uint8_t mo = (uint8_t)constrain(month, 1, 12);
  uint8_t dd = (uint8_t)constrain(day, 1, 31);
  uint8_t hh = (uint8_t)constrain(hour, 0, 23);
  uint8_t mm = (uint8_t)constrain(minute, 0, 59);
  uint8_t ss = (uint8_t)constrain(second, 0, 59);

  uint8_t regs[7];
  regs[0] = binToBcd(ss);
  regs[1] = binToBcd(mm);
  regs[2] = binToBcd(hh);
  regs[3] = 1;
  regs[4] = binToBcd(dd);
  regs[5] = binToBcd(mo);
  regs[6] = binToBcd(yy);

  Wire.beginTransmission(gRtcAddr);
  Wire.write((uint8_t)0x00);
  for (uint8_t i = 0; i < sizeof(regs); i++) {
    Wire.write(regs[i]);
  }
  return Wire.endTransmission() == 0;
}

static bool rtcReadDateTime(int& year, int& month, int& day, int& hour, int& minute, int& second) {
  if (!hasRTC || gRtcAddr == 0) return false;

  uint8_t regs[7] = {0};
  if (!i2cReadRegs(gRtcAddr, 0x00, regs, sizeof(regs))) return false;

  second = bcdToBin(regs[0] & 0x7F);
  minute = bcdToBin(regs[1] & 0x7F);
  hour = bcdToBin(regs[2] & 0x3F);
  day = bcdToBin(regs[4] & 0x3F);
  month = bcdToBin(regs[5] & 0x1F);
  year = 2000 + bcdToBin(regs[6]);

  if (year < 2020 || year > 2099) return false;
  if (month < 1 || month > 12) return false;
  if (day < 1 || day > 31) return false;
  if (hour > 23 || minute > 59 || second > 59) return false;
  return true;
}
