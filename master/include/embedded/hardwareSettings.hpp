#pragma once

constexpr int COMPONENT_TIMESTAMP_TIMEOUT = 500;
constexpr int RES_TIMESTAMP_TIMEOUT = 200;
constexpr int DC_VOLTAGE_TIMEOUT = 150;
constexpr int DC_VOLTAGE_HOLD = 1000;
constexpr int EBS_BUZZER_TIMEOUT = 8000;
constexpr int LED_BLINK_INTERVAL = 500;
constexpr int INITIAL_CHECKUP_STEP_TIMEOUT = 500;
constexpr unsigned long READY_TIMEOUT_MS = 5000;
constexpr unsigned long RELEASE_EBS_TIMEOUT_MS = 1000;
constexpr unsigned long ENGAGE_EBS_TIMEOUT_MS = 5000;
constexpr int WD_TIMEOUT_MS = 500;
constexpr int BRAKE_PRESSURE_LOWER_THRESHOLD = 120;
constexpr int BRAKE_PRESSURE_UPPER_THRESHOLD = 510;

constexpr int WHEEL_MEASUREMENT_INTERVAL_MS = 30;  // 30ms // TODO: change to adequate value
constexpr int WHEEL_MEASUREMENT_INTERVAL_MIN = (WHEEL_MEASUREMENT_INTERVAL_MS / 60'000.0);
constexpr int PULSES_PER_ROTATION = 48;

// Number of consecutive different values of a digital input to consider change
// (to avoid noise)
constexpr int CHANGE_COUNTER_LIMIT = 5;
constexpr int WD_PULSE_INTERVAL_MS = 50;
// TODO: confirm wd timer values

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
constexpr int SDC_BSPD_OUT = 14;//TODO

constexpr int BRAKE_LIGHT = 2;
constexpr int WD_SDC_CLOSE = 40;
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
constexpr int RR_WSS = 5;//TODO
constexpr int BRAKE_SENSOR = 38;
constexpr int SOC = 24;    // TODO: send to dash via can ,include in logging class
constexpr int ATS = 16;    // TODO: id asms on close sdc
constexpr int ASATS = 20;  // TODO: if asms on close sdc
constexpr int WD_READY = 37;
constexpr int WD_SDC_RELAY = 33;
