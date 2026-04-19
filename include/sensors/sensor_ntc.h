#pragma once

float readThermistor();
bool thermistorTempFromResistance(float rNtc, float r25, float& tempC);
