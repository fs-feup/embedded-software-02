#pragma once

constexpr int COMPONENT_TIMESTAMP_TIMEOUT = 500;
constexpr int RES_TIMESTAMP_TIMEOUT = 200;
constexpr int DC_VOLTAGE_TIMEOUT = 150;
constexpr int DC_VOLTAGE_HOLD = 1000;
constexpr int EBS_BUZZER_TIMEOUT = 8000;
constexpr int LED_BLINK_INTERVAL = 500;
constexpr int PROCESS_INTERVAL = 100;
constexpr int INITIAL_CHECKUP_STEP_TIMEOUT = 500;
constexpr unsigned long READY_TIMEOUT_MS = 5000;
constexpr unsigned long RELEASE_EBS_TIMEOUT_MS = 1000;
constexpr unsigned long ENGAGE_EBS_TIMEOUT_MS = 5000;
constexpr int WD_TIMEOUT_MS = 500;
constexpr int BRAKE_PRESSURE_LOWER_THRESHOLD = 160;
constexpr int BRAKE_PRESSURE_UPPER_THRESHOLD = 510;
constexpr int LIMIT_RPM_INTERVAL = 500000;

constexpr int ADC_MAX_VALUE = 1023;
constexpr int SOC_PERCENT_MAX = 100;
constexpr int MAX_MISSION = 7;
constexpr int PULSES_PER_ROTATION = 48;  // TODO: adjust
constexpr int SECONDS_IN_MINUTE = 60;
constexpr float MICRO_TO_SECONDS = 1e-6f;

// Number of consecutive different values of a digital input to consider change
// (to avoid noise)
constexpr int CHANGE_COUNTER_LIMIT = 5;
constexpr int WD_PULSE_INTERVAL_MS = 50;
constexpr int WATCHDOG_TOGGLE_DURATION = 1000;
constexpr int WATCHDOG_TEST_DURATION = 1000;

/*
 * ===========
 * OUTPUT PINS
 * ===========
 */

constexpr int ASSI_BLUE_PIN = 25;
constexpr int ASSI_YELLOW_PIN = 12;

constexpr int EBS_VALVE_1_PIN = 17;
constexpr int EBS_VALVE_2_PIN = 13;

constexpr int CLOSE_SDC = 21;
// #define SDC_LOGIC_WATCHDOG_OUT_PIN 10
constexpr int SDC_BSPD_OUT = 14;

constexpr int BRAKE_LIGHT = 2;
constexpr int WD_SDC_CLOSE = 40;  // high if bspd is high (sdc closed)?
constexpr int WD_ALIVE = 15;

/*
 * ==========
 * INPUT PINS
 * ==========
 */

constexpr int ASMS_IN_PIN = 18;
constexpr int AMI = 23;

// When pressure on the line passes the threshold defined, pin is set to HIGH
constexpr int EBS_SENSOR1 = 41;
constexpr int EBS_SENSOR2 = 39;
constexpr int SDC_BSPD_STATE_PIN = 22;

constexpr int RL_WSS = 4;
constexpr int RR_WSS = 5;
constexpr int BRAKE_SENSOR = 38;
constexpr int SOC = 24;
constexpr int ATS = 16;
constexpr int ASATS = 20;
constexpr int WD_READY = 37;
constexpr int WD_SDC_RELAY = 33;
