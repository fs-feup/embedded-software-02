#pragma once

#include <Arduino.h>
struct PARAMETERS {
    uint32_t set_voltage = 0;
    uint32_t current_voltage = 0;
    uint32_t allowed_current = 0;
    uint32_t set_current = 0;
    uint32_t current_current = 0;
    uint32_t ccl = 0;
    int16_t temperature[60];
};

enum class Status {  // state machine status
  IDLE,
  CHARGING,
  SHUTDOWN
};