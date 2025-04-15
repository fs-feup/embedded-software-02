#pragma once
#include <array>
#include <cstdint>
#include <deque>

#include "data_struct.hpp"

// Add values to the front of the queue and remove from back if necessary
void insert_value_queue(uint16_t value, std::deque<uint16_t>& queue);

// Calculate the average of values in a queue
uint16_t average_queue(const std::deque<uint16_t>& queue);

// Check if data sequence matches expected pattern
bool check_sequence(const uint8_t* data, const std::array<uint8_t, 3>& expected);

/**
 * Converts RPM float value to a 4-byte representation
 * @param rpm The RPM value to convert
 */
std::array<uint8_t, 4> rpm_to_bytes(float rpm);

InverterModeParams get_inverter_mode_config(SwitchMode switch_mode);