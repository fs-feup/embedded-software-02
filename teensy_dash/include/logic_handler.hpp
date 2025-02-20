#pragma once
#include <Arduino.h>
#include <elapsedMillis.h>

#include "data_struct.hpp"
#include "io_settings.hpp"
#include "utils.hpp"
class LogicHandler {
public:
  LogicHandler(SystemData& system_data, SystemVolatileData& current_updated_data);
  bool should_start_manual_driving() const;
  bool should_start_as_driving() const;
  bool should_go_idle() const;
  int scale_apps2_to_apps1(int apps2) const;
  int calculate_torque();

private:
  bool plausibility(int apps1, int apps2) const;
  int apps_to_bamocar_value(int apps1, int apps2);
  elapsedMillis implausibility_timer = 0;
  bool apps_timeout = false;
  SystemData& data;
  SystemVolatileData& updated_data;
};

LogicHandler::LogicHandler(SystemData& system_data, SystemVolatileData& current_updated_data) : data(system_data), updated_data(current_updated_data) {}

bool LogicHandler::should_start_manual_driving() const{
  return (data.r2d_pressed && updated_data.TSOn && data.R2DTimer < R2D_TIMEOUT);
}

bool LogicHandler::should_start_as_driving() const { return (updated_data.TSOn && updated_data.as_ready); }

bool LogicHandler::should_go_idle() const { return (!updated_data.TSOn); }

int LogicHandler::scale_apps2_to_apps1(int apps2) const { return apps2 + APPS_LINEAR_OFFSET; }

bool LogicHandler::plausibility(int apps1, int apps2) const {
  if (apps1 < apps2) return false;

  if (apps1 > APPS_1_UPPER_BOUND || apps1 < APPS_1_LOWER_BOUND) return false;

  if (apps2 > APPS_2_UPPER_BOUND || apps2 < APPS_2_LOWER_BOUND) return false;

  if (apps1 >= APPS1_DEAD_THRESHOLD)
    return apps2 >= APPS1_DEADZONE_EQUIVALENT - APPS_MAX_ERROR_ABS && apps2 <= APPS_2_UPPER_BOUND;

  if (apps2 <= APPS2_DEAD_THRESHOLD)
    return apps1 >= APPS_1_LOWER_BOUND && apps1 <= APPS2_DEADZONE_EQUIVALENT + APPS_MAX_ERROR_ABS;

  // TODO(PedroRomao3): check if 2 conditions above are needed

  int apps2_updated = scale_apps2_to_apps1(apps2);
  int plausibility_value = abs(apps2_updated - apps1) * 100 / apps1;

  return (plausibility_value < APPS_MAX_ERROR_PERCENT);
}

int LogicHandler::apps_to_bamocar_value(int apps1, int apps2) {
  int torque_value = 0;

  if (apps2 <= APPS2_DEAD_THRESHOLD)
    torque_value = apps1;
  else
    torque_value = scale_apps2_to_apps1(apps2);

  int bamo_max = BAMOCAR_MAX;
  int bamo_min = BAMOCAR_MIN;
  int apps_max = APPS_MAX;  // TODO(PedroRomao3): vars needed ?
  int apps_min = APPS_MIN;

  if (torque_value > apps_max) torque_value = apps_max;
  if (torque_value < apps_min) torque_value = apps_min;

  // maps sensor value to bamocar range
  torque_value = map(torque_value, apps_min, apps_max, bamo_min, bamo_max);

  return torque_value >= BAMOCAR_MAX ? BAMOCAR_MAX : torque_value;
}

int LogicHandler::calculate_torque() {
  int apps1_average = average_queue(data.apps1_readings);
  int apps2_average = average_queue(data.apps2_readings);

  bool plausible = plausibility(apps1_average, apps2_average);

  if (!plausible and implausibility_timer > APPS_IMPLAUSIBLE_TIMEOUT_MS) {
    return 0;
  }

  if (plausible) implausibility_timer = 0;

  int bamocar_value = apps_to_bamocar_value(apps1_average, apps2_average);

  if (apps_timeout) {
    if (bamocar_value == 0) {
      apps_timeout = false;
    } else
      return 0;
  }

  float pedal_travel_percentage = ((float)bamocar_value / BAMOCAR_MAX) * 100.0;

  if (updated_data.brake_pressure >= BRAKE_BLOCK_THRESHOLD && pedal_travel_percentage >= 25.0) {
    if (implausibility_timer > APPS_BRAKE_PLAUSIBILITY_TIMEOUT_MS) {
      apps_timeout = true;
      return 0;
    }
  } else
    implausibility_timer = 0;

  return bamocar_value;
}
