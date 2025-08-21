#pragma once
#include <Arduino.h>

#include "../../CAN_IDs.h"
#include "enum_utils.hpp"
#include "model/systemData.hpp"

/**
 * @brief Function to create left wheel msg
 */
void create_left_wheel_msg(std::array<uint8_t, 5>& msg, double value) {
  value /= WHEEL_PRECISION;  // take precision off to send integer value
  if (value < 0) value = 0;

  msg[0] = LEFT_WHEEL_MSG;
  // Copy the bytes of the double value to msg[1] to msg[4]
  for (int i = 0; i < 4; i++)
    msg[i + 1] = static_cast<int>(value) >> (8 * i);  // shift 8(byte) to msb each time
}

inline std::array<uint8_t, 8> create_signals_msg_1(const SystemData& system_data,
                                                   const uint8_t state,
                                                   const uint8_t state_checkup) {
  // Ensure boolean values are normalized to 0 or 1
  auto normalize_bool = [](bool value) -> uint8_t { return value ? 1 : 0; };
  // Pack byte 0 with explicit normalization
  uint8_t byte0 = (normalize_bool(true) << 7) |  // placeholder
                  (normalize_bool(system_data.hardware_data_.asms_on_) << 6) |
                  (normalize_bool(system_data.hardware_data_.asats_pressed_) << 5) |
                  (normalize_bool(system_data.hardware_data_.ats_pressed_) << 4) |
                  (normalize_bool(system_data.hardware_data_.tsms_sdc_closed_) << 3) |
                  (normalize_bool(system_data.hardware_data_.master_sdc_closed_) << 2) |
                  (normalize_bool(system_data.failure_detection_.ts_on_) << 1) |
                  normalize_bool(system_data.hardware_data_.wd_ready_);

  // Pack byte 5 flags with explicit normalization
  uint8_t byte1 =
      (normalize_bool(system_data.failure_detection_.emergency_signal_) << 7) |
      (normalize_bool(system_data.failure_detection_.bms_dead_) << 6) |
      (normalize_bool(system_data.r2d_logics_.engageEbsTimestamp.checkWithoutReset()) << 5) |
      (normalize_bool(system_data.r2d_logics_.releaseEbsTimestamp.checkWithoutReset()) << 4) |
      (normalize_bool(system_data.failure_detection_.steer_dead_) << 3) |
      (normalize_bool(system_data.failure_detection_.pc_dead_) << 2) |
      (normalize_bool(system_data.failure_detection_.inversor_dead_) << 1) |
      normalize_bool(system_data.failure_detection_.res_dead_);

  uint8_t byte2 = (state_checkup & 0x0F) |
                  ((system_data.hardware_data_.pneumatic_line_pressure_1_ & 0x01) << 7) |
                  ((system_data.hardware_data_.pneumatic_line_pressure_2_ & 0x01) << 6) |
                  ((system_data.hardware_data_.pneumatic_line_pressure_ & 0x01) << 5);

  bool pneumatic1 = system_data.hardware_data_.pneumatic_line_pressure_1_;
  bool pneumatic2 = system_data.hardware_data_.pneumatic_line_pressure_2_;
  DEBUG_PRINT_VAR(pneumatic1);
  DEBUG_PRINT_VAR(pneumatic2);

  return {
      byte0,
      byte1,
      byte2,
      static_cast<uint8_t>((system_data.failure_detection_.dc_voltage_ >> 24) & 0xFF),
      static_cast<uint8_t>((system_data.failure_detection_.dc_voltage_ >> 16) & 0xFF),
      static_cast<uint8_t>((system_data.failure_detection_.dc_voltage_ >> 8) & 0xFF),
      static_cast<uint8_t>(system_data.failure_detection_.dc_voltage_ & 0xFF),
      static_cast<uint8_t>((to_underlying(system_data.mission_) & 0x0F) | ((state & 0x0F) << 4))};
}

inline std::array<uint8_t, 8> create_hydraulic_presures_msg(const SystemData& system_data) {
  int adc_front = system_data.hardware_data_.hydraulic_line_front_pressure;
  float hydraulic_front_bar =
      (adc_front - HYDRAULIC_PRESSURE_ADC_MIN) *
      (static_cast<float>(HYDRAULIC_PRESSURE_MAX_BAR) / HYDRAULIC_PRESSURE_SPAN_ADC);

  int adc_rear = system_data.hardware_data_._hydraulic_line_pressure;
  float hydraulic_rear_bar =
      (adc_rear - HYDRAULIC_PRESSURE_ADC_MIN) *
      (static_cast<float>(HYDRAULIC_PRESSURE_MAX_BAR) / HYDRAULIC_PRESSURE_SPAN_ADC);

  DEBUG_PRINT_VAR(hydraulic_front_bar);
  DEBUG_PRINT_VAR(hydraulic_rear_bar);

  union FloatToBytes {
    float f;
    uint8_t bytes[4];
  };

  FloatToBytes front_converter{hydraulic_front_bar};
  FloatToBytes rear_converter{hydraulic_rear_bar};

  std::array<uint8_t, 8> data;
  data[0] = front_converter.bytes[0];
  data[1] = front_converter.bytes[1];
  data[2] = front_converter.bytes[2];
  data[3] = front_converter.bytes[3];

  data[4] = rear_converter.bytes[0];
  data[5] = rear_converter.bytes[1];
  data[6] = rear_converter.bytes[2];
  data[7] = rear_converter.bytes[3];

  return data;
}