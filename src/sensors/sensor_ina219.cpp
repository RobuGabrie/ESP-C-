static bool readINA219Register16(uint8_t reg, uint16_t& out) {
  Wire.beginTransmission(0x40);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;

  Wire.requestFrom((uint8_t)0x40, (uint8_t)2);
  if (Wire.available() < 2) return false;

  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  out = ((uint16_t)msb << 8) | (uint16_t)lsb;
  return true;
}
