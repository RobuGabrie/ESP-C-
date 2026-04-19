#pragma once

bool rtcWriteDateTime(int year, int month, int day, int hour, int minute, int second);
bool rtcReadDateTime(int& year, int& month, int& day, int& hour, int& minute, int& second);
