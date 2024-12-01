#pragma once
#include <Arduino.h>
#include <deque>
#include "io_settings.hpp"
#include "utils.hpp"
#include "data_struct.hpp"
#include <cstdint>

class IOManager {
public:
    IOManager();

    void setup();
    void manage();
    void read_apps();
    void play_buzzer(uint8_t duration_seconds);
    RPMValues calculate_rpm();
    

private:
    SystemData& data;
    void update_buzzer();
};

IOManager::IOManager(SystemData& system_data) : data(system_data) {}

void IOManager::manage() {
    read_apps();
    update_buzzer();
    calculate_rpm();
}
void IOManager::setup() {
    pinMode(APPS_1_PIN, INPUT);
    pinMode(APPS_2_PIN, INPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(FRONT_RIGHT_WHEEL_ENCODER_PIN), []()
        { 
            IOManager::data.second_to_last_wheel_pulse_fr = IOManager::data.last_wheel_pulse_fr;
            IOManager::data.last_wheel_pulse_fr = micros(); 
        }, RISING);
    attachInterrupt(digitalPinToInterrupt(FRONT_LEFT_WHEEL_ENCODER_PIN), []()
        { 
            IOManager::data.second_to_last_wheel_pulse_fl = IOManager::data.last_wheel_pulse_fl;
            IOManager::data.last_wheel_pulse_fl = micros(); 
        }, RISING);
}

void IOManager::read_apps() {
    insert_value_queue(data.apps1_readings, analogRead(APPS_1_PIN));
    insert_value_queue(data.apps2_readings, analogRead(APPS_2_PIN));
}

void IOManager::play_buzzer(uint8_t duration_seconds) {
    data.buzzer_active = true;
    data.buzzer_start_time = millis();
    data.buzzer_duration_ms = duration_seconds * 1000; 
    digitalWrite(BUZZER_PIN, HIGH);
}

void IOManager::update_buzzer() {
    if (data.buzzer_active && (millis() - data.buzzer_start_time >= data.buzzer_duration_ms)) {
        digitalWrite(BUZZER_PIN, LOW);
        data.buzzer_active = false;
    }
}

RPMValues IOManager::calculate_rpm() {
    unsigned long time_interval_fr = data.last_wheel_pulse_fr - data.second_to_last_wheel_pulse_fr;
    unsigned long time_interval_fl = data.last_wheel_pulse_fl - data.second_to_last_wheel_pulse_fl;

    float fr_rpm = (micros() - data.last_wheel_pulse_fr > LIMIT_RPM_INTERVAL) ? 0.0 : 1 / (time_interval_fr * 1e-6 * WPS_PULSES_PER_ROTATION) * 60;
    float fl_rpm = (micros() - data.last_wheel_pulse_fl > LIMIT_RPM_INTERVAL) ? 0.0 : 1 / (time_interval_fl * 1e-6 * WPS_PULSES_PER_ROTATION) * 60;

    return { fr_rpm, fl_rpm };
}