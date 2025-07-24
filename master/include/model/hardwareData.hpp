#pragma once

#include "embedded/hardwareSettings.hpp"
#include "metro.h"

struct HardwareData {
  bool pneumatic_line_pressure_ = true;
  bool pneumatic_line_pressure_1_ = false;
  bool pneumatic_line_pressure_2_ = false;
  bool asms_on_ = false;
  bool asats_pressed_ = false;
  bool ats_pressed_ = false;
  bool tsms_sdc_closed_{false};
  bool master_sdc_closed_ = false;
  bool wd_ready_ = false;
  int hydraulic_line_front_pressure = 0;
  int _hydraulic_line_pressure = 0;
  uint8_t soc_ = 0;
  double _right_wheel_rpm = 0;
  double _left_wheel_rpm = 0;
};
