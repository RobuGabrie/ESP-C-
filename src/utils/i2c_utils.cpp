#ifndef HS_MAIN_COMPOSED
void __hs_utils_i2c_stub() {}
#else

static const char* i2cDeviceName(uint8_t addr) {
	switch (addr) {
		case 0x3C: return "SSD1306";
		case 0x0C: return "AK8963";
		case 0x40: return "INA219";
		case 0x50: return "AT24C32";
		case 0x60: return "RTC";
		case 0x68: return "DS1307";
		case 0x69: return "MPU6500";
		case 0x6A: return "LSM6DS3";
		default: return "Unknown";
	}
}

static void scanI2cDevices() {
	gI2cFoundCount = 0;
	gI2cScanOverflow = false;

	for (uint8_t addr = 1; addr < 127; addr++) {
		Wire.beginTransmission(addr);
		if (Wire.endTransmission() == 0) {
			if (gI2cFoundCount < I2C_SCAN_MAX_DEVICES) {
				gI2cFound[gI2cFoundCount++] = addr;
			} else {
				gI2cScanOverflow = true;
			}
		}
	}

	gLastI2cScanMs = millis();
}

#endif
