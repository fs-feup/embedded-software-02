#pragma once

#include <FlexCAN_T4.h>
#include "Arduino.h"

#define TOTAL_BOARDS 6
#define DELAY_INTERVAL 500
#define N_NTC 18
#define VDD 5.0
#define RESISTOR_PULLUP 10000.0
#define RESISTOR_NTC_REFERNCE 10000.0  // Resistência a 25°C do NTC
#define TEMPERATURE_DEFAULT_C 25.0
#define TEMPERATURE_DEFAULT_K 298.15
#define NTC_BETA 3971.0
#define MAXIMUM_TEMPERATURE 60.0
#define MASTER_CELL_ID 0x109
#define CELLS_PER_MESSAGE 6
#define CELL_TEMPS_BASE_ID 0x110
#define CAN_BAUD_RATE 500000
#define THERMISTOR_MODULE_NUMBER 0x00
#define NUMBER_OF_THERMISTORS 0x01
#define HIGHEST_THERMISTOR_ID 0x01
#define LOWEST_THERMISTOR_ID 0x00
#define CHECKSUM_CONSTANT 0x39
#define MSG_LENGTH 0x08
#define BMS_THERMISTOR_ID 0x1839F380
#define MAX_INT8_T 127
#define MIN_INT8_T -128
#define ANALOG_MAX 1023
struct BoardData {
    int8_t min_temp = MAX_INT8_T;
    int8_t max_temp = MIN_INT8_T;
    int8_t avg_temp = 0;
    bool valid = false;
};

float read_ntc_temperature(int analog_value);
void read_Temperatures();
void check_Temperatures();
int8_t safeTemperatureCast(float temp);
void send_CAN_max_min_avg_Temperatures();
void send_to_BMS(int8_t global_min, int8_t global_max, int8_t global_avg);
void send_CAN_all_cell_temperatures();
void show_Temperatures();
void code_reset();
void can_sniffer(const CAN_message_t& msg);
void calculate_global_stats(int8_t& global_min, int8_t& global_max, int8_t& global_avg);