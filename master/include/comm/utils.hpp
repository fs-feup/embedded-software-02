#pragma once
#include <Arduino.h>

#include "../../CAN_IDs.h"
#include "model/systemData.hpp"
#include "enum_utils.hpp"

/**
 * @brief Function to create left wheel msg
 */
void create_left_wheel_msg(std::array<uint8_t, 5> &msg, double value) {
  value /= WHEEL_PRECISION;  // take precision off to send integer value
  if (value < 0) value = 0;

  msg[0] = LEFT_WHEEL_MSG;
  // Copy the bytes of the double value to msg[1] to msg[4]
  for (int i = 0; i < 4; i++)
    msg[i + 1] = static_cast<int>(value) >> (8 * i);  // shift 8(byte) to msb each time
}

inline std::array<uint8_t, 8> create_debug_message_1(const SystemData& system_data, const uint8_t state, const uint8_t state_checkup) {
    // Ensure boolean values are normalized to 0 or 1
    auto normalize_bool = [](bool value) -> uint8_t { return value ? 1 : 0; };
    
    // Pack byte 5 flags with explicit normalization
    uint8_t byte5 = (normalize_bool(system_data.failure_detection_.emergency_signal_) << 7) | 
                    (normalize_bool(system_data.hardware_data_.pneumatic_line_pressure_) << 6) |
                    (normalize_bool(system_data.r2d_logics_.engageEbsTimestamp.checkWithoutReset()) << 5) | 
                    (normalize_bool(system_data.r2d_logics_.releaseEbsTimestamp.checkWithoutReset()) << 4) |
                    (normalize_bool(system_data.failure_detection_.steer_dead_) << 3) | 
                    (normalize_bool(system_data.failure_detection_.pc_dead_) << 2) |
                    (normalize_bool(system_data.failure_detection_.inversor_dead_) << 1) | 
                    normalize_bool(system_data.failure_detection_.res_dead_);
    
    // Pack byte 6 flags with explicit normalization
    uint8_t byte6 = (normalize_bool(system_data.hardware_data_.asms_on_) << 7) | 
                    (normalize_bool(system_data.failure_detection_.ts_on_) << 6) | 
                    (normalize_bool(system_data.hardware_data_.tsms_sdc_closed_) << 5) |
                    (state_checkup & 0x0F);
    
    return {
        DBG_LOG_MSG,
        static_cast<uint8_t>((system_data.hardware_data_._hydraulic_line_pressure >> 24) & 0xFF),
        static_cast<uint8_t>((system_data.hardware_data_._hydraulic_line_pressure >> 16) & 0xFF),
        static_cast<uint8_t>((system_data.hardware_data_._hydraulic_line_pressure >> 8) & 0xFF),
        static_cast<uint8_t>(system_data.hardware_data_._hydraulic_line_pressure & 0xFF),
        byte5,
        byte6,
        static_cast<uint8_t>((to_underlying(system_data.mission_) & 0x0F) | ((state & 0x0F) << 4))
    };
}

inline std::array<uint8_t, 8> create_debug_message_2(const SystemData& system_data) {
    // Ensure boolean values are normalized to 0 or 1
    return {
        DBG_LOG_MSG_2,
        static_cast<uint8_t>((system_data.failure_detection_.dc_voltage_ >> 24) & 0xFF),
        static_cast<uint8_t>((system_data.failure_detection_.dc_voltage_ >> 16) & 0xFF),
        static_cast<uint8_t>((system_data.failure_detection_.dc_voltage_ >> 8) & 0xFF),
        static_cast<uint8_t>(system_data.failure_detection_.dc_voltage_ & 0xFF),
        static_cast<uint8_t>(system_data.hardware_data_.pneumatic_line_pressure_1_ & 0x01),
        static_cast<uint8_t>(system_data.hardware_data_.pneumatic_line_pressure_2_ & 0x01),
        static_cast<uint8_t>(system_data.hardware_data_.master_sdc_closed_ & 0x01)
    };
}