#include "utils.hpp"

#include <cmath>
#include <io_settings.hpp>
#include <numeric>

void insert_value_queue(const uint16_t value, std::deque<uint16_t>& queue) {
  queue.push_front(value);

  if (queue.size() > config::apps::SAMPLES) {
    queue.pop_back();
  }
}

uint16_t average_queue(const std::deque<uint16_t>& queue) {
  uint16_t avg = 0;
  if (!queue.empty()) {
    const double sum = std::accumulate(queue.begin(), queue.end(), 0);
    avg = static_cast<uint16_t>(sum / queue.size());
  }
  return avg;
}

bool check_sequence(const uint8_t* data, const std::array<uint8_t, 3>& expected) {
  return (data[1] == expected[0] && data[2] == expected[1] && data[3] == expected[2]);
}

std::array<uint8_t, 4> rpm_to_bytes(const float rpm) {
  // Convert to integer with 2 decimal places precision
  const auto scaled_value = static_cast<int32_t>(roundf(rpm * 100.0f));

  // Return array with bytes in little-endian order
  return {static_cast<uint8_t>(scaled_value & 0xFF),
          static_cast<uint8_t>((scaled_value >> 8) & 0xFF),
          static_cast<uint8_t>((scaled_value >> 16) & 0xFF),
          static_cast<uint8_t>((scaled_value >> 24) & 0xFF)};
}

InverterModeParams get_inverter_mode_config(const SwitchMode switch_mode) {
  InverterModeParams params{};
  switch (switch_mode) {
    case SwitchMode::INVERTER_MODE_0:
      params = {.i_max_pk_percent = 0,
                .speed_limit_percent = 0,
                .i_cont_percent = 0,
                .speed_ramp_acc = 100,
                .moment_ramp_acc = 10,
                .speed_ramp_brake = 100,
                .moment_ramp_decc = 10};
      break;
    case SwitchMode::INVERTER_MODE_CAVALETES:
      params = {.i_max_pk_percent = 3,
                .speed_limit_percent = 5,
                .i_cont_percent = 3,
                .speed_ramp_acc = 1000,
                .moment_ramp_acc = 100,
                .speed_ramp_brake = 1000,
                .moment_ramp_decc = 100};
      break;
    case SwitchMode::INVERTER_MODE_LIMITER:
      params = {.i_max_pk_percent = 10,
                .speed_limit_percent = 10,
                .i_cont_percent = 10,
                .speed_ramp_acc = 1000,
                .moment_ramp_acc = 100,
                .speed_ramp_brake = 1000,
                .moment_ramp_decc = 100};
      break;
    case SwitchMode::INVERTER_MODE_BRAKE_TEST:
      params = {.i_max_pk_percent = 70,
                .speed_limit_percent = 84,
                .i_cont_percent = 70,
                .speed_ramp_acc = 200,
                .moment_ramp_acc = 20,
                .speed_ramp_brake = 0,
                .moment_ramp_decc = 0};
      break;
    case SwitchMode::INVERTER_MODE_SKIDPAD:
      params = {.i_max_pk_percent = 70,
                .speed_limit_percent = 70,
                .i_cont_percent = 70,
                .speed_ramp_acc = 500,
                .moment_ramp_acc = 50,
                .speed_ramp_brake = 500,
                .moment_ramp_decc = 50};
      break;
    case SwitchMode::INVERTER_MODE_ENDURANCE:
      params = {.i_max_pk_percent = 70,
                .speed_limit_percent = 84,
                .i_cont_percent = 70,
                .speed_ramp_acc = 200,
                .moment_ramp_acc = 20,
                .speed_ramp_brake = 200,
                .moment_ramp_decc = 20};
      break;
    case SwitchMode::INVERTER_MODE_MAX_ATTACK:
      params = {.i_max_pk_percent = 100,
                .speed_limit_percent = 84,
                .i_cont_percent = 100,
                .speed_ramp_acc = 100,
                .moment_ramp_acc = 10,
                .speed_ramp_brake = 100,
                .moment_ramp_decc = 10};
      break;
    case SwitchMode::INVERTER_MODE_NULL:
      params = {.i_max_pk_percent = 40,
                .speed_limit_percent = 40,
                .i_cont_percent = 40,
                .speed_ramp_acc = 400,
                .moment_ramp_acc = 40,
                .speed_ramp_brake = 400,
                .moment_ramp_decc = 40};
      break;
    case SwitchMode::INVERTER_MODE_NULL2:
      params = {.i_max_pk_percent = 45,
                .speed_limit_percent = 45,
                .i_cont_percent = 45,
                .speed_ramp_acc = 450,
                .moment_ramp_acc = 45,
                .speed_ramp_brake = 450,
                .moment_ramp_decc = 45};
      break;
    case SwitchMode::INVERTER_MODE_NULL3:
      params = {.i_max_pk_percent = 50,
                .speed_limit_percent = 50,
                .i_cont_percent = 50,
                .speed_ramp_acc = 500,
                .moment_ramp_acc = 50,
                .speed_ramp_brake = 500,
                .moment_ramp_decc = 50};
      break;
    case SwitchMode::INVERTER_MODE_NULL4:
      params = {.i_max_pk_percent = 55,
                .speed_limit_percent = 55,
                .i_cont_percent = 55,
                .speed_ramp_acc = 550,
                .moment_ramp_acc = 55,
                .speed_ramp_brake = 550,
                .moment_ramp_decc = 55};
      break;
    case SwitchMode::INVERTER_MODE_NULL5:
      params = {.i_max_pk_percent = 60,
                .speed_limit_percent = 60,
                .i_cont_percent = 60,
                .speed_ramp_acc = 600,
                .moment_ramp_acc = 60,
                .speed_ramp_brake = 600,
                .moment_ramp_decc = 60};
      break;
    default:
      break;
  }
  return params;
}