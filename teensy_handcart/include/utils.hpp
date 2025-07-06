#pragma once
#include <Arduino.h>


void extract_value(volatile uint32_t &param_value, const uint8_t *buf) {
    param_value = 0;
    param_value |= buf[4] << 24;
    param_value |= buf[5] << 16;
    param_value |= buf[6] << 8;
    param_value |= buf[7];
}

void print_value(const char *label, const uint32_t value) {
    Serial.print(label);
    Serial.println(value);
}