#pragma once
#include <Arduino.h>
#include <Bounce2.h>

#include <cstdint>
#include <deque>

#include "data_struct.hpp"
#include "io_settings.hpp"
#include "utils.hpp"

class IOManager {
public:
  IOManager(SystemData& system_data, volatile SystemVolatileData& volatile_updatable_data,
            SystemVolatileData& volatile_updated_data);

  void setup();
  void manage();
  void read_apps();
  void play_r2d_sound();
  void play_buzzer(uint8_t duration_seconds);
  void calculate_rpm();
  void manage_ats();
  void read_rotative_switch();

private:
  SystemData& data;
  volatile SystemVolatileData& updatable_data;
  SystemVolatileData& updated_data;
  inline static IOManager* instance = nullptr;
  void update_buzzer();
  void read_pins_handle_leds();
  SwitchMode last_mode = SwitchMode::MODE_0;
  Bounce r2d_button = Bounce();
  Bounce ats_button = Bounce();
  Bounce display_button = Bounce();
};

IOManager::IOManager(SystemData& system_data, volatile SystemVolatileData& volatile_updatable_data,
                     SystemVolatileData& volatile_updated_data)
    : data(system_data),
      updatable_data(volatile_updatable_data),
      updated_data(volatile_updated_data) {
  instance = this;
}

void IOManager::manage() {
  r2d_button.update();
  ats_button.update();
  display_button.update();
  data.r2d_pressed = r2d_button.fell();
  data.ats_pressed = ats_button.fell();
  data.display_pressed = display_button.fell();
  read_rotative_switch();
  read_pins_handle_leds();
  read_apps();
  update_buzzer();
  calculate_rpm();
  manage_ats();
}

void IOManager::read_rotative_switch() {
  const int raw_value = analogRead(pins::analog::ROTARY_SWITCH);
  const int mode_index = raw_value / config::adc::SEGMENT_SIZE;
  const int clamped_index = std::min(mode_index, config::adc::NUM_MODES - 1);

  int boundary_low = clamped_index * config::adc::SEGMENT_SIZE - config::adc::HYSTERESIS;
  int boundary_high = (clamped_index + 1) * config::adc::SEGMENT_SIZE + config::adc::HYSTERESIS;
  if (raw_value >= boundary_low && raw_value < boundary_high) {
    data.switch_mode = static_cast<SwitchMode>(clamped_index);
    last_mode = data.switch_mode;
  } else {
    data.switch_mode = last_mode;
  }
}

void IOManager::manage_ats() {
  if (data.ats_pressed && !updated_data.asms_on) {
    data.ats_pressed = false;
    digitalWrite(pins::digital::ATS_OUT, HIGH);
  }
}
void IOManager::setup() {
  // TODO: botão rotativo
  pinMode(pins::digital::INERTIA, INPUT);
  pinMode(pins::analog::APPS_1, INPUT);
  pinMode(pins::analog::APPS_2, INPUT);
  pinMode(pins::output::BUZZER, OUTPUT);
  pinMode(pins::output::BSPD_LED, OUTPUT);
  pinMode(pins::output::INERTIA_LED, OUTPUT);
  pinMode(pins::digital::ATS_OUT, OUTPUT);
  attachInterrupt(
      digitalPinToInterrupt(pins::encoder::FRONT_RIGHT_WHEEL),
      []() {
        IOManager::instance->updatable_data.second_to_last_wheel_pulse_fr =
            IOManager::instance->updatable_data.last_wheel_pulse_fr;
        IOManager::instance->updatable_data.last_wheel_pulse_fr = micros();
      },
      RISING);

  attachInterrupt(
      digitalPinToInterrupt(pins::encoder::FRONT_LEFT_WHEEL),
      []() {
        IOManager::instance->updatable_data.second_to_last_wheel_pulse_fl =
            IOManager::instance->updatable_data.last_wheel_pulse_fl;
        IOManager::instance->updatable_data.last_wheel_pulse_fl = micros();
      },
      RISING);
  r2d_button.attach(pins::digital::R2D, INPUT);
  r2d_button.interval(0.1);
  ats_button.attach(pins::digital::ATS, INPUT);
  ats_button.interval(0.1);
  display_button.attach(pins::output::DISPLAY_MODE, INPUT);
  display_button.interval(0.1);
}

void IOManager::read_apps() {
  insert_value_queue(analogRead(pins::analog::APPS_1), data.apps1_readings);
  insert_value_queue(analogRead(pins::analog::APPS_2), data.apps2_readings);
}

void IOManager::play_r2d_sound() { play_buzzer(1); }

void IOManager::play_buzzer(uint8_t duration_seconds) {
  data.buzzer_active = true;  // TODO: this is PWM now
  data.buzzer_start_time = millis();
  data.buzzer_duration_ms = duration_seconds * 1000;
  digitalWrite(pins::output::BUZZER, HIGH);
}

void IOManager::update_buzzer() {
  if (data.buzzer_active && (millis() - data.buzzer_start_time >= data.buzzer_duration_ms)) {
    digitalWrite(pins::output::BUZZER, LOW);
    data.buzzer_active = false;
  }
}

inline void IOManager::read_pins_handle_leds() {
  digitalWrite(pins::output::BSPD_LED, digitalRead(pins::digital::BSPD));
  digitalWrite(pins::output::INERTIA_LED, digitalRead(pins::digital::INERTIA));
}

void IOManager::calculate_rpm() {
  unsigned long time_interval_fr =
      updated_data.last_wheel_pulse_fr - updated_data.second_to_last_wheel_pulse_fr;
  unsigned long time_interval_fl =
      updated_data.last_wheel_pulse_fl - updated_data.second_to_last_wheel_pulse_fl;

  data.fr_rpm = (micros() - updated_data.last_wheel_pulse_fr > config::wheel::LIMIT_RPM_INTERVAL)
                    ? 0.0
                    : 1 / (time_interval_fr * 1e-6 * config::wheel::PULSES_PER_ROTATION) * 60;
  data.fl_rpm = (micros() - updated_data.last_wheel_pulse_fl > config::wheel::LIMIT_RPM_INTERVAL)
                    ? 0.0
                    : 1 / (time_interval_fl * 1e-6 * config::wheel::PULSES_PER_ROTATION) * 60;
}