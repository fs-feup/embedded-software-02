#pragma once

#include <Arduino.h>
#include "constants.hpp"
struct BoardTemperatureData {
  int8_t min_temp = 0;
  int8_t max_temp = 0;
  int8_t avg_temp = 0;
  bool   has_data = false;
  unsigned long last_update_ms = 0;
};

struct PARAMETERS {
  bool ch_safety = false;
  uint32_t set_voltage = 0;
  uint32_t current_voltage = 0;
  uint32_t allowed_current = 0;
  uint32_t set_current = 0;
  uint32_t current_current = 0;
  uint32_t ccl = 0;
  BoardTemperatureData cell_board_temps[TOTAL_BOARDS]; // For teensy_cells
};

enum class Status {  // state machine status
  IDLE,
  CHARGING,
  SHUTDOWN
};