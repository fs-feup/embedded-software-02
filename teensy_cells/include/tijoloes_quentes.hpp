#pragma once

#include <FlexCAN_T4.h>
#include "Arduino.h"


// System Configuration
constexpr uint8_t TOTAL_BOARDS = 6;
constexpr uint16_t DELAY_INTERVAL = 100;
constexpr uint8_t N_NTC = 18;
constexpr uint16_t ANALOG_MAX = 1023;
constexpr uint16_t ANALOG_MIN = 0;
constexpr int ERROR_SIGNAL = 35;
constexpr uint8_t MAX_RETRIES = 3;

// Voltage and Resistor Configuration
constexpr float VDD = 5.0;
constexpr float V_REF = 3.3f;
constexpr float RESISTOR_PULLUP = 10'000.0;
constexpr float RESISTOR_NTC_REFERNCE = 10'000.0;  // NTC resistance at 25°C

// Temperature Constants
constexpr float TEMPERATURE_DEFAULT_C = 25.0;
constexpr float TEMPERATURE_MIN_C = -100.0;
constexpr float TEMPERATURE_MAX_C = 100.0;
constexpr float KELVIN_OFFSET = 273.15f;
constexpr float TEMPERATURE_DEFAULT_K = 298.15f;
constexpr float NTC_BETA = 3971.0;
constexpr float MAXIMUM_TEMPERATURE = 60.0;
constexpr int8_t MAX_INT8_T = 127;
constexpr int8_t MIN_INT8_T = -128;
constexpr uint16_t MAX_TEMP_DELAY_MS = 500;

// CAN Communication
constexpr uint32_t MASTER_CELL_ID = 0x109;
constexpr uint8_t CELLS_PER_MESSAGE = 6;
constexpr uint32_t CELL_TEMPS_BASE_ID = 0x110;
constexpr uint32_t CAN_BAUD_RATE = 500'000;

// BMS Protocol
constexpr uint32_t BMS_THERMISTOR_ID = 0x1839'F380;
constexpr uint8_t THERMISTOR_MODULE_NUMBER = 0x00;
constexpr uint8_t NUMBER_OF_THERMISTORS = 0x01;
constexpr uint8_t HIGHEST_THERMISTOR_ID = 0x01;
constexpr uint8_t LOWEST_THERMISTOR_ID = 0x00;
constexpr uint8_t CHECKSUM_CONSTANT = 0x39;
constexpr uint8_t MSG_LENGTH = 0x08;
struct TemperatureData {
    int8_t min_temp = ::MAX_INT8_T;
    int8_t max_temp = ::MIN_INT8_T;
    int8_t avg_temp = 0;
};
struct BoardData {
    TemperatureData temp_data;
    bool valid = false;
    unsigned long last_update_ms;
};

float read_ntc_temperature(int analog_value);
void read_check_temperatures();
int8_t safe_temperature_cast(float temp);
void send_can_max_min_avg_temperatures();
void send_to_bms(const TemperatureData& global_data);
void send_can_all_cell_temperatures();
void show_temperatures();
void code_reset();
void can_snifflas(const CAN_message_t& msg);
void calculate_global_stats(TemperatureData& global_data);
bool check_temperture_timeout();