#pragma once

#include "embedded/hardwareSettings.hpp"
#include "metro.h"

struct HardwareData {
  bool pneumatic_line_pressure_ = true;
  bool pneumatic_line_pressure_1_ = false;
  bool pneumatic_line_pressure_2_ = false;
  bool asms_on_ = false;
  bool asats_pressed_ = false;
  bool sdc_open_{true}; /*< Detects AATS >*/
  double _right_wheel_rpm = 0;
  double _left_wheel_rpm = 0;
  int _hydraulic_line_pressure = 0;
};
