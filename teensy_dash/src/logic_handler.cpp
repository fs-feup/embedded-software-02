#include "logic_handler.hpp"

#include <io_settings.hpp>

#include "../../CAN_IDs.h"

LogicHandler::LogicHandler(SystemData& system_data, SystemVolatileData& current_updated_data)
    : data(system_data), updated_data(current_updated_data) {}

bool LogicHandler::should_start_manual_driving() const {
  // DEBUG_PRINTLN("Checking if should start manual driving v8");
  // print var
  // DEBUG_PRINTLN("R2D pressed: " + String(data.r2d_pressed));
  // DEBUG_PRINTLN("TSOn: " + String(updated_data.TSOn));
  return (data.r2d_pressed &&
          updated_data.TSOn /* && data.r2d_brake_timer < config::r2d::TIMEOUT_MS */);
}

bool LogicHandler::should_start_as_driving() const {
  return (updated_data.TSOn &&
          (updated_data.as_state == AS_DRIVING || updated_data.as_state == AS_READY));
}

bool LogicHandler::should_go_idle() const { return (!updated_data.TSOn); }

uint16_t LogicHandler::scale_apps_lower_to_apps_higher(const uint16_t apps_lower) {
  return apps_lower + config::apps::LINEAR_OFFSET;
}

bool LogicHandler::plausibility(const int apps_higher,
                                const int apps_lower) {  // unit test, TODO(Yves)
  const bool valid_input = (apps_higher >= apps_lower) &&
                           (apps_higher >= config::apps::LOWER_BOUND_APPS_HIGHER &&
                            apps_higher <= config::apps::UPPER_BOUND_APPS_HIGHER) &&
                           (apps_lower >= config::apps::LOWER_BOUND_APPS_LOWER &&
                            apps_lower <= config::apps::UPPER_BOUND_APPS_LOWER);

  if (!valid_input) {
    return false;
    DEBUG_PRINTLN("Apps implausible: invalid input");
  }

  DEBUG_PRINTLN("Apps Higher: " + String(apps_higher));
  DEBUG_PRINTLN("Apps Lower: " + String(apps_lower));
  const int scaled_apps_lower = scale_apps_lower_to_apps_higher(apps_lower);
  DEBUG_PRINTLN("Scaled Apps Lower: " + String(scaled_apps_lower));

  const int difference = abs(scaled_apps_lower - apps_higher);
  DEBUG_PRINTLN("Difference: " + String(difference));

  const int percentage_difference = (difference * 100) / apps_higher;
  DEBUG_PRINTLN("Percentage Difference: " + String(percentage_difference));

  if (apps_lower < config::apps::APPS_LOWER_ZEROED) {
    if (apps_higher < config::apps::APPS_HIGHER_WHEN_LOWER_ZEROES) {
      DEBUG_PRINTLN("Apps implausible: apps lower is zeroed, so we can ignore the implausibility");
      return true;  // apps lower is zeroed, so we can ignore the implausibility
      
    } else {
      DEBUG_PRINTLN(
        "Apps implausible: apps lower is zeroed, but apps higher is not in the expected range");
      return false;
      
    }
  }
  // print values
  DEBUG_PRINTLN("dbg print lune");
  return (percentage_difference < config::apps::MAX_ERROR_PERCENT);
}

uint16_t LogicHandler::apps_to_bamocar_value(const uint16_t apps_higher,
                                             const uint16_t apps_lower) {
  uint16_t torque_value = apps_higher;

  torque_value = constrain(torque_value, config::apps::MIN, config::apps::MAX);

  torque_value =
      config::apps::MAX - torque_value;  // Invert the value to match Bamocar's expected input
  // DEBUG_PRINTLN("Torque value before deadband: " + String(torque_value));
  if (torque_value <= config::apps::DEADBAND) {
    return 0;
  }

  float normalized_input = (float)(torque_value - config::apps::DEADBAND) /
                           (float)(config::apps::MAX_FOR_TORQUE - config::apps::DEADBAND);

  // Apply sigmoid mapping for smooth transition
  // Sigmoid: output = 1 / (1 + e^(-k * (x - 0.5)))
  const float k = 15.0;  // Steepness of the sigmoid curve (adjust for responsiveness)
  float normalized_output =
      1.0 / (1.0 + exp(-k * (normalized_input -
                             0.5)));  // Use desmos graphing calculator for visualization

  uint16_t mapped_value = (uint16_t)(normalized_input * (config::bamocar::MAX));

  return min(mapped_value, config::bamocar::MAX);
}

bool LogicHandler::just_entered_emergency() {
  const bool is_emergency = (updated_data.as_state == AS_EMERGENCY);

  if (!entered_emergency && is_emergency) {
    entered_emergency = true;
    return true;
  }

  if (entered_emergency && !is_emergency) {
    entered_emergency = false;  // reset if I left emergency so if i enter again buzzer plays
  }

  return false;
}

bool LogicHandler::check_brake_plausibility(const uint16_t bamocar_value) {
  if (const float pedal_travel_percentage =
          (static_cast<float>(bamocar_value) / static_cast<float>(config::bamocar::MAX)) * 100.0f;
      updated_data.brake_pressure >= config::apps::BRAKE_BLOCK_THRESHOLD &&
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

bool LogicHandler::check_apps_plausibility(const uint16_t apps_higher_avg,
                                           const uint16_t apps_lower_avg) {
  if (plausibility(apps_higher_avg, apps_lower_avg)) {
    apps_implausibility_timer = 0;
    return true;
  }
  if (apps_implausibility_timer > config::apps::IMPLAUSIBLE_TIMEOUT_MS) {
    apps_timeout = true;
    return false;
  }
  return true;
}

uint16_t LogicHandler::calculate_torque() {
  const uint16_t apps_higher_average = average_queue(data.apps_higher_readings);
  const uint16_t apps_lower_average = average_queue(data.apps_lower_readings);
  // DEBUG_PRINTLN("Apps Higher Average v2: " + String(apps_higher_average));
  // DEBUG_PRINTLN("Apps Lower Average v2: " + String(apps_lower_average));
  if (!check_apps_plausibility(apps_higher_average, apps_lower_average)) {
    DEBUG_PRINTLN("Apps implausible, going idle");
    DEBUG_PRINTLN("Apps implausible, going idle");
    DEBUG_PRINTLN("Apps implausible, going idle");
    DEBUG_PRINTLN("Apps implausible, going idle");

    return config::apps::ERROR_PLAUSIBILITY;  // shutdown ?
  }

  const uint16_t bamocar_value = apps_to_bamocar_value(apps_higher_average, apps_lower_average);

  // DEBUG_PRINTLN("Bamocar value: " + String(bamocar_value));

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