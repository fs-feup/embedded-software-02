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
  uint16_t scale_apps_lower_to_apps_higher(const uint16_t apps_lower) const;
  uint16_t calculate_torque();

private:
  bool plausibility(int apps_higher, int apps_lower) const;
  uint16_t apps_to_bamocar_value(const uint16_t apps_higher, const uint16_t apps_lower);
  elapsedMillis brake_implausibility_timer = 0;
  elapsedMillis apps_implausibility_timer = 0;
  bool apps_timeout = false;
  SystemData& data;
  SystemVolatileData& updated_data;
  bool check_brake_plausibility(const uint16_t bamocar_value);
  bool check_apps_plausibility(const uint16_t apps_higher_avg, const uint16_t apps_lower_avg);
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

uint16_t LogicHandler::scale_apps_lower_to_apps_higher(const uint16_t apps_lower) const {
  return apps_lower + config::apps::LINEAR_OFFSET;
}

bool LogicHandler::plausibility(const int apps_higher, const int apps_lower) const {  // unit test, todo
  bool valid_input = (apps_higher >= apps_lower) && (apps_higher <= config::apps::UPPER_BOUND_APPS_HIGHER) &&
                     (apps_higher >= config::apps::LOWER_BOUND_APPS_HIGHER) &&
                     (apps_lower <= config::apps::UPPER_BOUND_APPS_LOWER) &&
                     (apps_lower >= config::apps::LOWER_BOUND_APPS_LOWER);

  if (!valid_input) {
    return false;
  }

  bool is_plausible = false;

  if (apps_higher >= config::apps::DEAD_THRESHOLD_APPS_HIGHER) {
    is_plausible =
        (apps_lower >= config::apps::APPS_HIGHER_DEADZONE_IN_APPS_LOWER_SCALE - config::apps::MAX_ERROR_ABS &&
         apps_lower <= config::apps::UPPER_BOUND_APPS_LOWER);
  } else if (apps_lower <= config::apps::DEAD_THRESHOLD_APPS_LOWER) {
    is_plausible =
        (apps_higher >= config::apps::LOWER_BOUND_APPS_HIGHER &&
         apps_higher <= config::apps::APPS_LOWER_DEADZONE_IN_APPS_HIGHER_SCALE + config::apps::MAX_ERROR_ABS);
  } else {
    const int apps_lower_updated = scale_apps_lower_to_apps_higher(apps_lower);
    const int plausibility_value = abs(apps_lower_updated - apps_higher) * 100 / apps_higher;
    is_plausible = (plausibility_value < config::apps::MAX_ERROR_PERCENT);
  }

  return is_plausible;
}

uint16_t LogicHandler::apps_to_bamocar_value(const uint16_t apps_higher, const uint16_t apps_lower) {
  uint16_t torque_value = 0;

  if (apps_lower <= config::apps::DEAD_THRESHOLD_APPS_LOWER) {
    torque_value = apps_higher;
  } else {
    torque_value = scale_apps_lower_to_apps_higher(apps_lower);
  }

  torque_value = constrain(torque_value, config::apps::MIN, config::apps::MAX);

  torque_value = map(torque_value, config::apps::MIN, config::apps::MAX, config::bamocar::MIN,
                     config::bamocar::MAX);

  return min(torque_value, config::bamocar::MAX);
}

bool LogicHandler::check_brake_plausibility(uint16_t bamocar_value) {
  float pedal_travel_percentage = ((float)bamocar_value / config::bamocar::MAX) * 100.0;

  if (updated_data.brake_pressure >= config::apps::BRAKE_BLOCK_THRESHOLD &&
      pedal_travel_percentage >= 25.0) {
    if (brake_implausibility_timer > config::apps::BRAKE_PLAUSIBILITY_TIMEOUT_MS) {
      apps_timeout = true;
      return false;
    }
  } else {
    brake_implausibility_timer = 0;
  }

  return true;
}

bool LogicHandler::check_apps_plausibility(uint16_t apps_higher_avg, uint16_t apps_lower_avg) {
  bool plausible = plausibility(apps_higher_avg, apps_lower_avg);

  if (!plausible) {
    if (apps_implausibility_timer > config::apps::IMPLAUSIBLE_TIMEOUT_MS) {
      return false;
    }
  } else {
    apps_implausibility_timer = 0;
  }

  return true;
}

uint16_t LogicHandler::calculate_torque() {
  uint16_t apps_higher_average = average_queue(data.apps_higher_readings);
  uint16_t apps_lower_average = average_queue(data.apps_lower_readings);

  if (!check_apps_plausibility(apps_higher_average, apps_lower_average)) {
    return config::apps::ERROR_PLAUSIBILITY;  // shutdown ?
  }

  uint16_t bamocar_value = apps_to_bamocar_value(apps_higher_average, apps_lower_average);

  if (apps_timeout) {
    if (bamocar_value == 0) {  // Pedal released
      apps_timeout = false;
    } else {
      return 0;
    }
  }

  if (!check_brake_plausibility(bamocar_value)) {
    return 0;
  }

  return bamocar_value;
}
