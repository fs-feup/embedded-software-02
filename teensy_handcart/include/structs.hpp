#pragma once

#include <Arduino.h>
struct PARAMETERS {
    uint32_t setVoltage = 0;
    uint32_t currVoltage = 0;
    uint32_t allowedCurrent = 0;
    uint32_t setCurrent = 0;
    uint32_t currCurrent = 0;
    uint32_t ccl = 0;
    int16_t temp[60];
};

enum class Status {  // state machine status
  IDLE,
  CHARGING,
  SHUTDOWN
};