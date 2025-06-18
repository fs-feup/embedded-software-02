#include "hw_io_manager.hpp"

#include <io_settings.hpp>
#include <utils.hpp>

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
  read_hydraulic_pressure();
  read_rotative_switch();
  read_pins_handle_leds();
  read_apps();
  update_buzzer();
  calculate_rpm();
  manage_ats();
  update_R2D_timer();
}

void IOManager::read_rotative_switch() const {  // maybe map
  // const int raw_value = analogRead(pins::analog::ROTARY_SWITCH);
  // data.switch_mode = static_cast<SwitchMode>((raw_value + config::adc::HALF_JUMP) *
  //                                            config::adc::NEW_SCALE_MAX / config::adc::MAX_VALUE); // TODO: USE ENUM LAST VALUE +1 INSTEAD OF NEW_SCALE_MAX
  // TODO CONFIRM BEST WAY
  int pos = map(analogRead(pins::analog::ROTARY_SWITCH), 0, config::adc::MAX_VALUE, 0, 7);
  data.switch_mode = static_cast<SwitchMode>(pos);


}

void IOManager::read_hydraulic_pressure() const {
  insert_value_queue(analogRead(pins::analog::BRAKE_PRESSURE), data.brake_readings);
}

void IOManager::update_R2D_timer() const {
  if (average_queue(data.brake_readings) > config::apps::BRAKE_BLOCK_THRESHOLD) {
    data.r2d_brake_timer = 0;
  }
}

void IOManager::manage_ats() const {
  static bool ats_active = false;
  static unsigned long ats_activated_time = 0;

  if (data.ats_pressed && !updated_data.asms_on && !ats_active) {
    DEBUG_PRINT("ATS pressed, setting HIGH");
    data.ats_pressed = false;
    digitalWrite(pins::digital::ATS_OUT, HIGH);
    ats_active = true;
    ats_activated_time = millis();
  }
  if (ats_active && (millis() - ats_activated_time >= 1000)) {
    DEBUG_PRINT("ATS HIGH duration elapsed, setting LOW");
    digitalWrite(pins::digital::ATS_OUT, LOW);
    ats_active = false;
  }
}

void IOManager::setup() {
  pinMode(pins::digital::INERTIA, INPUT);
  pinMode(pins::analog::APPS_HIGHER, INPUT);
  pinMode(pins::analog::APPS_LOWER, INPUT);
  pinMode(pins::analog::ROTARY_SWITCH, INPUT);
  pinMode(pins::analog::BRAKE_PRESSURE, INPUT);
  pinMode(pins::output::BUZZER, OUTPUT);
  pinMode(pins::output::BSPD_LED, OUTPUT);
  pinMode(pins::output::INERTIA_LED, OUTPUT);
  pinMode(pins::digital::ATS_OUT, OUTPUT);
  pinMode(pins::output::TS_LED, OUTPUT);
  pinMode(pins::digital::TS, OUTPUT);

  attachInterrupt(
      digitalPinToInterrupt(pins::encoder::FRONT_RIGHT_WHEEL),
      []() {
        instance->updatable_data.second_to_last_wheel_pulse_fr =
            instance->updatable_data.last_wheel_pulse_fr;
        instance->updatable_data.last_wheel_pulse_fr = micros();
      },
      RISING);

  attachInterrupt(
      digitalPinToInterrupt(pins::encoder::FRONT_LEFT_WHEEL),
      []() {
        instance->updatable_data.second_to_last_wheel_pulse_fl =
            instance->updatable_data.last_wheel_pulse_fl;
        instance->updatable_data.last_wheel_pulse_fl = micros();
      },
      RISING);
  r2d_button.attach(pins::digital::R2D, INPUT);
  r2d_button.interval(100);
  ats_button.attach(pins::digital::ATS, INPUT);
  ats_button.interval(100);
  display_button.attach(pins::output::DISPLAY_MODE, INPUT);
  display_button.interval(100);
}

void IOManager::read_apps() const {
  insert_value_queue(analogRead(pins::analog::APPS_HIGHER), data.apps_higher_readings);
  insert_value_queue(analogRead(pins::analog::APPS_LOWER), data.apps_lower_readings);
}

void IOManager::play_r2d_sound() const { play_buzzer(1); }

void IOManager::play_buzzer(const uint8_t duration_seconds) const {
  data.buzzer_active = true;
  data.buzzer_start_time = millis();
  data.buzzer_duration_ms = duration_seconds * 1000;
  DEBUG_PRINTLN("Playing buzzer for " + String(data.buzzer_duration_ms) + " ms");
  // tone(pins::output::BUZZER, config::buzzer::BUZZER_FREQUENCY);  // TODO(romain): tone has time
  //                                                                // limite maybe timer not needed
  digitalWrite(pins::output::BUZZER, HIGH);  // Use digitalWrite for buzzer
}

void IOManager::update_buzzer() const {
  if (data.buzzer_active && (millis() - data.buzzer_start_time >= data.buzzer_duration_ms)) {
    // noTone(pins::output::BUZZER);
    digitalWrite(pins::output::BUZZER, LOW);  // Stop the buzzer
    data.buzzer_active = false;
  }
}

void IOManager::read_pins_handle_leds() {
  digitalWrite(pins::output::BSPD_LED, digitalRead(pins::digital::BSPD));
  digitalWrite(pins::output::INERTIA_LED, digitalRead(pins::digital::INERTIA));
  digitalWrite(pins::output::TS_LED, digitalRead(pins::digital::TS));
}

void IOManager::calculate_rpm() const {
  constexpr float MICROSEC_TO_MIN = 60.0f / 1'000'000.0f;  // Convert μs to minutes
  const unsigned long current_time = micros();

  // Front right wheelLIMIT_RPM_INTERVAL
  if (const unsigned long time_since_last_pulse_fr =
          current_time - updated_data.last_wheel_pulse_fr;
      time_since_last_pulse_fr > config::wheel::LIMIT_RPM_INTERVAL) {
    // No recent pulses, wheel stopped
    data.fr_rpm = 0.0f;
  } else {
    // Check if time_interval calculation would overflow
    const unsigned long time_interval_fr =
        (updated_data.last_wheel_pulse_fr >= updated_data.second_to_last_wheel_pulse_fr)
            ? updated_data.last_wheel_pulse_fr - updated_data.second_to_last_wheel_pulse_fr
            : 0;  // Handle overflow case

    // Avoid division by zero
    if (time_interval_fr > 0) {
      data.fr_rpm = (MICROSEC_TO_MIN /
                     static_cast<float>(time_interval_fr * config::wheel::PULSES_PER_ROTATION));
    } else {
      data.fr_rpm = 0.0f;  // Invalid interval
    }
  }

  // Front left wheel (same logic)
  if (const unsigned long time_since_last_pulse_fl =
          current_time - updated_data.last_wheel_pulse_fl;
      time_since_last_pulse_fl > config::wheel::LIMIT_RPM_INTERVAL) {
    data.fl_rpm = 0.0f;
  } else {
    const unsigned long time_interval_fl =
        (updated_data.last_wheel_pulse_fl >= updated_data.second_to_last_wheel_pulse_fl)
            ? updated_data.last_wheel_pulse_fl - updated_data.second_to_last_wheel_pulse_fl
            : 0;

    if (time_interval_fl > 0) {
      data.fl_rpm = (MICROSEC_TO_MIN /
                     static_cast<float>(time_interval_fl * config::wheel::PULSES_PER_ROTATION));
    } else {
      data.fl_rpm = 0.0f;
    }
  }
}