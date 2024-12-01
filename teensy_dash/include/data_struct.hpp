#pragma once
#include <deque>

struct SystemData {
    std::deque<int> apps1_readings;
    std::deque<int> apps2_readings;
    bool buzzer_active = false;
    unsigned long buzzer_start_time;
    unsigned long buzzer_duration_ms;

    unsigned long last_wheel_pulse_fr = 0;
    unsigned long second_to_last_wheel_pulse_fr = 0;
    unsigned long last_wheel_pulse_fl = 0;
    unsigned long second_to_last_wheel_pulse_fl = 0;
};