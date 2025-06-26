#pragma once

#include <FlexCAN_T4.h>

#include "Arduino.h"
#include "../../CAN_IDs.h"
// System Configuration
constexpr uint8_t TOTAL_BOARDS = 6;
constexpr uint16_t TEMP_SENSOR_READ_INTERVAL = 100;
constexpr int8_t NO_ERROR_RESET_THRESHOLD = 50;
constexpr uint8_t NTC_SENSOR_COUNT = 18;
constexpr uint16_t ANALOG_MAX = 1023;
constexpr uint16_t ANALOG_MIN = 0;
constexpr int ERROR_SIGNAL = 35;
constexpr uint8_t MAX_RETRIES = 3;
constexpr uint8_t MAX_NUM_ERRORS = 4;
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
constexpr uint16_t MAX_TEMP_DELAY_MS = 2000;

// CAN Communication
constexpr uint8_t CELLS_PER_MESSAGE = 6;
constexpr uint32_t CAN_DRIVING_BAUD_RATE = 1'000'000;
constexpr uint32_t CAN_CHARGING_BAUD_RATE = 125'000;

// BMS Protocol
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
  bool has_communicated = false;
  unsigned long last_update_ms;
};

float read_ntc_temperature(int analog_value);
void read_check_temperatures();
int8_t safe_temperature_cast(float temp);
void send_can_max_min_avg_temperatures();
void show_temperatures();
void code_reset();
bool send_can_message(CAN_message_t& msg);

// Functions specific to master board
#if THIS_IS_MASTER
void send_master_heartbeat();
void can_snifflas(const CAN_message_t& msg);
void calculate_global_stats(TemperatureData& global_data);
bool check_temperature_timeouts();
void send_to_bms(const TemperatureData& global_data);
#endif

// Functions specific to non-master boards
#if !THIS_IS_MASTER
bool check_master_timeout();
void can_receive_from_master(const CAN_message_t& msg);
#endif

#if DEBUG_ENABLED
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_WRITE(x) Serial.write(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_WRITE(x)
#endif