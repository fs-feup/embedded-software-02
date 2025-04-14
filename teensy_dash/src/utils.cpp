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
  return {
    static_cast<uint8_t>(scaled_value & 0xFF),
    static_cast<uint8_t>((scaled_value >> 8) & 0xFF),
    static_cast<uint8_t>((scaled_value >> 16) & 0xFF),
    static_cast<uint8_t>((scaled_value >> 24) & 0xFF)
  };
}