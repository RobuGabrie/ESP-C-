#pragma once

bool i2cWriteReg(uint8_t addr, uint8_t reg, uint8_t val);
bool i2cReadRegs(uint8_t addr, uint8_t reg, uint8_t* data, size_t len);
bool initGyro();
bool readMagnetometer();
bool readGyro();
