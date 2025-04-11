#pragma once
#include <Arduino.h>
#include <elapsedMillis.h>

#include "data_struct.hpp"
#include "utils.hpp"

class LogicHandler {
public:
  LogicHandler(SystemData& system_data, SystemVolatileData& current_updated_data);
  bool should_start_manual_driving() const;
  bool should_start_as_driving() const;
  bool should_go_idle() const;
  bool just_entered_emergency();
  uint16_t scale_apps_lower_to_apps_higher(const uint16_t apps_lower) const;
  uint16_t calculate_torque();

private:
  bool plausibility(int apps_higher, int apps_lower) const;
  uint16_t apps_to_bamocar_value(const uint16_t apps_higher, const uint16_t apps_lower);
  elapsedMillis brake_implausibility_timer = 0;
  elapsedMillis apps_implausibility_timer = 0;
  bool apps_timeout = false;
  bool entered_emergency = false;
  SystemData& data;
  SystemVolatileData& updated_data;
  bool check_brake_plausibility(const uint16_t bamocar_value);
  bool check_apps_plausibility(const uint16_t apps_higher_avg, const uint16_t apps_lower_avg);
};
