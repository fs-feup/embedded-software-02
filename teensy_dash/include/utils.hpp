#pragma once
#include <deque>
#include <numeric>

#include "io_settings.hpp"

void insert_value_queue(const int value, std::deque<int>& queue) {
  queue.push_front(value);

  if (queue.size() > config::apps::SAMPLES) {
    queue.pop_back();
  }
}

int average_queue(const std::deque<int>& queue) {
  int avg = 0;
  if (!queue.empty()) {
    const double sum = std::accumulate(queue.begin(), queue.end(), 0);
    avg = static_cast<int>(sum / queue.size());
  }
  return avg;
}

bool check_sequence(const uint8_t* data, const std::array<uint8_t, 3>& expected) {
  return (data[1] == expected[0] && data[2] == expected[1] && data[3] == expected[2]);
}

union float2bytes {
  int input;
  char output[4];
};

void rpm_2_byte(const float rr_rpm,const char* rr_rpm_byte) {
  float2bytes data;
  /*
  1st we multiply rpm by 100 to get a 2 decimal place value.
  The roundf() function rounds rpm to the nearest integer value.
  */
  data.input = roundf(rr_rpm * 100);
  /*
  The order of the bytes in the output array depends on the endianness the system.
  -> little-endian system, the least significant byte will be at output[0],
  and the most significant byte will be at output[3].
  -> big-endian system, it's the other way around.
  */
  std::copy(std::begin(data.output), std::end(data.output), rr_rpm_byte);

  return;
}