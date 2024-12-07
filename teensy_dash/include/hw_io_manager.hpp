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
  IOManager(SystemData& system_data, volatile SystemVolatileData& volatile_updatable_data, SystemVolatileData& volatile_updated_data);

  void setup();
  void manage();
  void read_apps();
  void play_r2d_sound();
  void play_buzzer(uint8_t duration_seconds);
  void calculate_rpm();

private:
  SystemData& data;
  volatile SystemVolatileData& updatable_data;
  SystemVolatileData& updated_data;
  inline static IOManager* instance = nullptr;
  void update_buzzer();
  void read_pins_handle_leds();
  Bounce r2d_button = Bounce();
  Bounce ats_button = Bounce();
  Bounce display_button = Bounce();
};

IOManager::IOManager(SystemData& system_data, volatile SystemVolatileData& volatile_updatable_data, SystemVolatileData& volatile_updated_data)
    : data(system_data), updatable_data(volatile_updatable_data), updated_data(volatile_updated_data) {
      instance = this;
    }

void IOManager::manage() {
  r2d_button.update();
  ats_button.update();
  display_button.update();
  data.r2d_pressed = r2d_button.fell();
  data.ats_pressed = ats_button.fell();//TODO: check wtf this is for 
  data.display_pressed = display_button.fell();//TODO: display stuff
  read_pins_handle_leds();
  read_apps();
  update_buzzer();
  calculate_rpm();
}
void IOManager::setup() {
  //TODO: botão rotativo
  pinMode(BMS_PIN, INPUT);//TODO: make this a loop
  pinMode(IMD_PIN, INPUT);
  pinMode(TS_OFF_PIN, INPUT);
  pinMode(SDC_PIN, INPUT);
  pinMode(BSPD_PIN, INPUT);
  pinMode(INERTIA_PIN, INPUT);
  pinMode(APPS_1_PIN, INPUT);
  pinMode(APPS_2_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BMS_LED_PIN, OUTPUT);
  pinMode(IMD_LED_PIN, OUTPUT);
  pinMode(TS_OFF_LED_PIN, OUTPUT);
  pinMode(SDC_LED_PIN, OUTPUT);
  pinMode(BSPD_LED_PIN, OUTPUT);
  pinMode(INERTIA_LED_PIN, OUTPUT);
  attachInterrupt(
      digitalPinToInterrupt(FRONT_RIGHT_WHEEL_ENCODER_PIN),
      []() {
        IOManager::instance->updatable_data.second_to_last_wheel_pulse_fr =
            IOManager::instance->updatable_data.last_wheel_pulse_fr;
        IOManager::instance->updatable_data.last_wheel_pulse_fr = micros();
      },
      RISING);

  attachInterrupt(
      digitalPinToInterrupt(FRONT_LEFT_WHEEL_ENCODER_PIN),
      []() {
        IOManager::instance->updatable_data.second_to_last_wheel_pulse_fl =
            IOManager::instance->updatable_data.last_wheel_pulse_fl;
        IOManager::instance->updatable_data.last_wheel_pulse_fl = micros();
      },
      RISING);
  r2d_button.attach(R2D_PIN, INPUT);
  r2d_button.interval(0.1);
  ats_button.attach(ATS_PIN, INPUT);
  ats_button.interval(0.1);
  display_button.attach(DISPLAY_PIN, INPUT);
  display_button.interval(0.1);
}

void IOManager::read_apps() {
  insert_value_queue(analogRead(APPS_1_PIN), data.apps1_readings);
  insert_value_queue(analogRead(APPS_2_PIN), data.apps2_readings);
}

void IOManager::play_r2d_sound() { play_buzzer(1); }

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

inline void IOManager::read_pins_handle_leds() {
  digitalWrite(BMS_LED_PIN, digitalRead(BMS_PIN));
  digitalWrite(IMD_LED_PIN, digitalRead(IMD_PIN));
  digitalWrite(TS_OFF_LED_PIN, digitalRead(TS_OFF_PIN));
  digitalWrite(SDC_LED_PIN, digitalRead(SDC_PIN));
  digitalWrite(BSPD_LED_PIN, digitalRead(BSPD_PIN));
  digitalWrite(INERTIA_LED_PIN, digitalRead(INERTIA_PIN));
}

void IOManager::calculate_rpm() {
  unsigned long time_interval_fr =
      updated_data.last_wheel_pulse_fr - updated_data.second_to_last_wheel_pulse_fr;
  unsigned long time_interval_fl =
      updated_data.last_wheel_pulse_fl - updated_data.second_to_last_wheel_pulse_fl;

  data.fr_rpm = (micros() - updated_data.last_wheel_pulse_fr > LIMIT_RPM_INTERVAL)
                    ? 0.0
                    : 1 / (time_interval_fr * 1e-6 * WPS_PULSES_PER_ROTATION) * 60;
  data.fl_rpm = (micros() - updated_data.last_wheel_pulse_fl > LIMIT_RPM_INTERVAL)
                    ? 0.0
                    : 1 / (time_interval_fl * 1e-6 * WPS_PULSES_PER_ROTATION) * 60;
}