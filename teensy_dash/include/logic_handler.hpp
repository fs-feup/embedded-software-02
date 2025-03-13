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

LogicHandler::LogicHandler(SystemData& system_data, SystemVolatileData& current_updated_data)
    : data(system_data), updated_data(current_updated_data) {}

bool LogicHandler::should_start_manual_driving() const {
  return (data.r2d_pressed && updated_data.TSOn && data.r2d_brake_timer < config::r2d::TIMEOUT_MS);
}

bool LogicHandler::should_start_as_driving() const {
  return (updated_data.TSOn && updated_data.as_ready);
}

bool LogicHandler::should_go_idle() const { return (!updated_data.TSOn); }

int LogicHandler::scale_apps2_to_apps1(const int apps2) const {
  return apps2 + config::apps::LINEAR_OFFSET;
}

bool LogicHandler::plausibility(const int apps1, const int apps2) const {//unit test, todo
  bool valid_input = (apps1 >= apps2) && (apps1 <= config::apps::UPPER_BOUND_APPS1) &&
                     (apps1 >= config::apps::LOWER_BOUND_APPS1) &&
                     (apps2 <= config::apps::UPPER_BOUND_APPS2) &&
                     (apps2 >= config::apps::LOWER_BOUND_APPS2);

  if (!valid_input) {
    return false;
  }

  bool is_plausible = false;

  if (apps1 >= config::apps::DEAD_THRESHOLD_APPS1) {
    is_plausible =
        (apps2 >= config::apps::DEADZONE_EQUIVALENT_APPS1 - config::apps::MAX_ERROR_ABS &&
         apps2 <= config::apps::UPPER_BOUND_APPS2);
  } else if (apps2 <= config::apps::DEAD_THRESHOLD_APPS2) {
    is_plausible = (apps1 >= config::apps::LOWER_BOUND_APPS1 &&
                    apps1 <= config::apps::DEADZONE_EQUIVALENT_APPS2 + config::apps::MAX_ERROR_ABS);
  } else {
    const int apps2_updated = scale_apps2_to_apps1(apps2);
    const int plausibility_value = abs(apps2_updated - apps1) * 100 / apps1;//rmv 2x 100 ? todo
    is_plausible = (plausibility_value < config::apps::MAX_ERROR_PERCENT * 100);
  }

  return is_plausible;
}

int LogicHandler::apps_to_bamocar_value(const int apps1, const int apps2) {
  int torque_value = 0;

  if (apps2 <= config::apps::DEAD_THRESHOLD_APPS2) {
    torque_value = apps1;
  } else {
    torque_value = scale_apps2_to_apps1(apps2);
  }

  if (torque_value > config::apps::MAX) {
    torque_value = config::apps::MAX;
  }
  if (torque_value < config::apps::MIN) {
    torque_value = config::apps::MIN;
  }

  torque_value = map(torque_value, config::apps::MIN, config::apps::MAX, config::bamocar::MIN,
                     config::bamocar::MAX);

  if (torque_value >= config::bamocar::MAX) {
    return config::bamocar::MAX;
  }
  return torque_value;
}

int LogicHandler::calculate_torque() { //TODO: Refactor this function
  int apps1_average = average_queue(data.apps1_readings);
  int apps2_average = average_queue(data.apps2_readings);

  bool plausible = plausibility(apps1_average, apps2_average);

  if (!plausible && implausibility_timer > config::apps::IMPLAUSIBLE_TIMEOUT_MS) {
    return 0;
  }

  if (plausible) {
    implausibility_timer = 0;
  }

  int bamocar_value = apps_to_bamocar_value(apps1_average, apps2_average);

  if (apps_timeout) {
    if (bamocar_value == 0) {
      apps_timeout = false;
    } else {
      return 0;
    }
  }

  float pedal_travel_percentage = ((float)bamocar_value / config::bamocar::MAX) * 100.0;

  if (updated_data.brake_pressure >= config::apps::BRAKE_BLOCK_THRESHOLD &&
      pedal_travel_percentage >= 25.0) {
    if (implausibility_timer > config::apps::BRAKE_PLAUSIBILITY_TIMEOUT_MS) {
      apps_timeout = true;
      return 0;
    }
  } else {
    implausibility_timer = 0;
  }

  return bamocar_value;
}
