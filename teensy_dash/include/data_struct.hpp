#pragma once
#include <deque>

enum class SwitchMode {
  INVERTER_MODE0,
  INVERTER_MODE1,
  INVERTER_MODE2,
  INVERTER_MODE3,
  INVERTER_MODE4,
  INVERTER_MODE5,
  INVERTER_MODE6,
  INVERTER_MODE7,
};

struct RPMValues {
  float fr_rpm;
  float fl_rpm;
};

struct SystemData {
  bool r2d_pressed = false;
  bool ats_pressed = false;
  bool display_pressed = false;
  SwitchMode switch_mode = SwitchMode::INVERTER_MODE0;
  bool buzzer_active = false;
  unsigned long buzzer_start_time;
  unsigned long buzzer_duration_ms;
  std::deque<int> apps1_readings;
  std::deque<int> apps2_readings;
  float fr_rpm = 0;
  float fl_rpm = 0;
  std::deque<int> brake_readings;

  elapsedMillis r2d_brake_timer = 0;
};

struct SystemVolatileData {
  bool TSOn = false;
  bool as_ready = false;
  bool asms_on = false;
  int brake_pressure = 0;
  int speed = 0;
  uint8_t soc = 0;

  unsigned long last_wheel_pulse_fr = 0;
  unsigned long second_to_last_wheel_pulse_fr = 0;
  unsigned long last_wheel_pulse_fl = 0;
  unsigned long second_to_last_wheel_pulse_fl = 0;
};

void copy_volatile_data(SystemVolatileData& dest, volatile SystemVolatileData const& src) {
  noInterrupts();
  dest.TSOn = src.TSOn;
  dest.as_ready = src.as_ready;
  dest.asms_on = src.asms_on;
  dest.brake_pressure = src.brake_pressure;
  dest.last_wheel_pulse_fr = src.last_wheel_pulse_fr;
  dest.second_to_last_wheel_pulse_fr = src.second_to_last_wheel_pulse_fr;
  dest.last_wheel_pulse_fl = src.last_wheel_pulse_fl;
  dest.second_to_last_wheel_pulse_fl = src.second_to_last_wheel_pulse_fl;
  interrupts();
}