#include "utils.hpp"

#include <cmath>
#include <io_settings.hpp>
#include <numeric>

void insert_value_queue(const uint16_t value, std::deque<uint16_t> &queue) {
  queue.push_front(value);

  if (queue.size() > config::apps::SAMPLES) {
    queue.pop_back();
  }
}

uint16_t average_queue(const std::deque<uint16_t> &queue) {
  uint16_t avg = 0;
  if (!queue.empty()) {
    const double sum = std::accumulate(queue.begin(), queue.end(), 0);
    avg = static_cast<uint16_t>(sum / queue.size());
  }
  return avg;
}

bool check_sequence(const uint8_t *data, const std::array<uint8_t, 3> &expected) {
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
      params = {.i_max_pk_percent = 16,
                .speed_limit_percent = 13,
                .i_cont_percent = 11,
                .speed_ramp_acc = 100,
                .moment_ramp_acc = 10,
                .speed_ramp_brake = 100,
                .moment_ramp_decc = 10};
      break;
    case SwitchMode::INVERTER_MODE_CAVALETES:
      params = {.i_max_pk_percent = 32,
                .speed_limit_percent = 26,
                .i_cont_percent = 22,
                .speed_ramp_acc = 100,
                .moment_ramp_acc = 10,
                .speed_ramp_brake = 100,
                .moment_ramp_decc = 10};
      break;
    case SwitchMode::INVERTER_MODE_LIMITER:
      params = {.i_max_pk_percent = 48,
                .speed_limit_percent = 39,
                .i_cont_percent = 33,
                .speed_ramp_acc = 1000,
                .moment_ramp_acc = 500,
                .speed_ramp_brake = 1000,
                .moment_ramp_decc = 500};
      break;
    case SwitchMode::INVERTER_MODE_BRAKE_TEST:
      params = {.i_max_pk_percent = 66,
                .speed_limit_percent = 53,
                .i_cont_percent = 44,
                .speed_ramp_acc = 1500,
                .moment_ramp_acc = 750,
                .speed_ramp_brake = 1500,
                .moment_ramp_decc = 750};
      break;
    case SwitchMode::INVERTER_MODE_SKIDPAD:
      params = {.i_max_pk_percent = 66,
                .speed_limit_percent = 53,
                .i_cont_percent = 44,
                .speed_ramp_acc = 100,
                .moment_ramp_acc = 10,
                .speed_ramp_brake = 100,
                .moment_ramp_decc = 10};
      break;
    case SwitchMode::INVERTER_MODE_ENDURANCE:
      params = {.i_max_pk_percent = 66,
                .speed_limit_percent = 53,
                .i_cont_percent = 44,
                .speed_ramp_acc = 100,
                .moment_ramp_acc = 10,
                .speed_ramp_brake = 100,
                .moment_ramp_decc = 10};
      break;
    case SwitchMode::INVERTER_MODE_MAX_ATTACK:
      params = {.i_max_pk_percent = 85,
                .speed_limit_percent = 53,
                .i_cont_percent = 44,
                .speed_ramp_acc = 3000,
                .moment_ramp_acc = 1500,
                .speed_ramp_brake = 3000,
                .moment_ramp_decc = 1500};
      break;
    case SwitchMode::INVERTER_MODE_NULL:
      params = {.i_max_pk_percent = 85,
                .speed_limit_percent = 53,
                .i_cont_percent = 44,
                .speed_ramp_acc = 5000,
                .moment_ramp_acc = 2500,
                .speed_ramp_brake = 5000,
                .moment_ramp_decc = 2500};
      break;
    default:
      break;
  }
  return params;
}
