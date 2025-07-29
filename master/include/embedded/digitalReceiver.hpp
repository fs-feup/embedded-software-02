// ReSharper disable CppMemberFunctionMayBeConst
#pragma once

#include <Bounce2.h>

#include <model/hardwareData.hpp>
#include <model/structure.hpp>

#include "debugUtils.hpp"
#include "hardwareSettings.hpp"
#include "metro.h"
#include "utils.hpp"

struct SimulateTSMSActivate {
private:
  unsigned long timer_ = 0;
  int state = 0;

public:
  bool activate_shit(bool tsms_activated) {
    switch (state) {
      case 0: {
        if (millis() - timer_ > 10000) {
          state = 1;
          timer_ = millis();
        }
        return true & tsms_activated;
        break;
      }
      case 1: {
        if (millis() - timer_ > 50) {
          timer_ = millis();
          state = 0;
        }
        return false;
      }
    }
  }
};

/**
 * @brief Class responsible for the reading of the digital
 * inputs into the Master teensy
 */
class DigitalReceiver {
  SimulateTSMSActivate sim;
  unsigned long tsms_timer_ = 0;
  Bounce asats_button = Bounce();

public:
  inline static uint32_t last_wheel_pulse_rl =
      0;  // Timestamp of the last pulse for left wheel RPM calculation
  inline static uint32_t second_to_last_wheel_pulse_rl = 0;  // Timestamp of the second to last
                                                             // pulse for left wheel RPM calculation
  inline static uint32_t last_wheel_pulse_rr =
      0;  // Timestamp of the last pulse for right wheel RPM calculation
  inline static uint32_t second_to_last_wheel_pulse_rr =
      0;  // Timestamp of the second to last pulse for right
          // wheel RPM calculation

  /**
   * @brief read all digital inputs
   */
  void digital_reads();

  /**
   * @brief Constructor for the class, sets pintmodes and buttons
   */
  DigitalReceiver(SystemData* system_data) : system_data_(system_data) {
    pinMode(SDC_TSMS_STATE_PIN, INPUT);
    pinMode(AMI, INPUT);
    pinMode(ASMS_IN_PIN, INPUT);
    pinMode(EBS_SENSOR2, INPUT);
    pinMode(EBS_SENSOR1, INPUT);

    pinMode(RL_WSS, INPUT);
    pinMode(RR_WSS, INPUT);
    pinMode(BRAKE_SENSOR, INPUT);
    pinMode(SOC, INPUT);
    pinMode(ATS, INPUT);
    pinMode(WD_READY, INPUT);
    pinMode(WD_SDC_RELAY, INPUT);

    asats_button.attach(ASATS, INPUT);
    asats_button.interval(100);

    attachInterrupt(
        digitalPinToInterrupt(RR_WSS),
        []() {
          second_to_last_wheel_pulse_rr = last_wheel_pulse_rr;
          last_wheel_pulse_rr = micros();
        },
        RISING);
    attachInterrupt(
        digitalPinToInterrupt(RL_WSS),
        []() {
          second_to_last_wheel_pulse_rl = last_wheel_pulse_rl;
          last_wheel_pulse_rl = micros();
        },
        RISING);
  }

private:
  SystemData* system_data_;  ///< Pointer to the system updatable data storage

  std::deque<int> brake_readings;                 ///< Buffer for brake sensor readings
  unsigned int asms_change_counter_ = 0;          ///< counter to avoid noise on asms
  unsigned int aats_change_counter_ = 0;          ///< counter to avoid noise on aats
  unsigned int sdc_change_counter_ = 0;           ///< counter to avoid noise on sdc
  unsigned int pneumatic_change_counter_ = 0;     ///< counter to avoid noise on pneumatic line
  unsigned int mission_change_counter_ = 0;       ///< counter to avoid noise on mission change
  unsigned int sdc_bspd_change_counter_ = 0;      ///< counter to avoid noise on sdc bspd
  unsigned int ats_change_counter_ = 0;           ///< counter to avoid noise on ats
  unsigned int wd_ready_change_counter_ = 0;      ///< counter to avoid noise on wd ready
  Mission last_tried_mission_ = Mission::MANUAL;  ///< Last attempted mission state

  /**
   * @brief Reads the pneumatic line pressure states and updates the HardwareData object.
   * Debounces input changes to avoid spurious transitions.
   */
  void read_pneumatic_line();

  /**
   * @brief Reads the current mission state based on input pins and updates the mission object.
   * Debounces input changes to avoid spurious transitions.
   */
  void read_mission();

  /**
   * @brief Reads the ASMS switch state and updates the HardwareData object.
   * Debounces input changes to avoid spurious transitions.
   */
  void read_asms_switch();
  /**
   * @brief Reads the AATS state and updates the HardwareData object.
   * Debounces input changes to avoid spurious transitions.
   */
  void read_asats_state();

  /**
   * @brief Reads the wheel speed sensors and updates the HardwareData object.
   * Debounces input changes to avoid spurious transitions.
   */
  void read_wheel_speed_sensors();

  /**
   * @brief Reads the bspd pin and updates the HardwareData object.
   * Debounces input changes to avoid spurious transitions.
   */
  void read_bspd_sdc();

  /**
   * @brief Reads the brake sensor and updates the HardwareData object.
   * Debounces input changes to avoid spurious transitions.
   */
  void read_brake_sensor();

  /**
   * @brief Reads the state of charge and updates the HardwareData object.
   */
  void read_soc();

  /**
   * @brief Reads the ats and updates the HardwareData object.
   * Debounces input changes to avoid spurious transitions.
   */
  void read_ats();

  /**
   * @brief Reads the watchdog ready pin and updates the HardwareData object.
   */
  void read_watchdog_ready();

  /**
   * @brief Reads the rpm of the wheels and updates the HardwareData object.
   */
  void read_rpm();
};

inline void DigitalReceiver::digital_reads() {
  asats_button.update();
  system_data_->hardware_data_.asats_pressed_ = asats_button.fell();
  read_pneumatic_line();
  read_mission();
  read_asms_switch();
  // read_asats_state();
  read_soc();
  read_brake_sensor();
  read_ats();
  read_bspd_sdc();
  read_watchdog_ready();
  read_rpm();
}

inline void DigitalReceiver::read_soc() {
  const int raw_value = analogRead(SOC);
  int mapped_value = map(raw_value, 0, ADC_MAX_VALUE, 0, SOC_PERCENT_MAX);
  mapped_value = constrain(mapped_value, 0, SOC_PERCENT_MAX);  // constrain to 0-100, just in case
  system_data_->hardware_data_.soc_ = static_cast<uint8_t>(mapped_value);
}

inline void DigitalReceiver::read_bspd_sdc() {
  bool is_sdc_closed = digitalRead(SDC_TSMS_STATE_PIN);  // low when sdc/bspd open
  // is_sdc_closed = this->sim.activate_shit(is_sdc_closed);
  debounce(is_sdc_closed, system_data_->hardware_data_.tsms_sdc_closed_,
           this->sdc_bspd_change_counter_, 25);
  // DEBUG_PRINT_VAR(is_sdc_closed);
  // DEBUG_PRINT_VAR(system_data_->hardware_data_.tsms_sdc_closed_);
}
inline void DigitalReceiver::read_brake_sensor() {
  int hydraulic_pressure = analogRead(BRAKE_SENSOR);
  insert_value_queue(hydraulic_pressure, brake_readings, 10);
  system_data_->hardware_data_._hydraulic_line_pressure = average_queue(brake_readings);
}
inline void DigitalReceiver::read_pneumatic_line() {
  bool pneumatic1 = digitalRead(EBS_SENSOR2);
  bool pneumatic2 = digitalRead(EBS_SENSOR1);

  system_data_->hardware_data_.pneumatic_line_pressure_1_ = pneumatic1;
  system_data_->hardware_data_.pneumatic_line_pressure_2_ = pneumatic2;
  bool latest_pneumatic_pressure = pneumatic1 && pneumatic2;

  debounce(latest_pneumatic_pressure, system_data_->hardware_data_.pneumatic_line_pressure_,
           pneumatic_change_counter_);
}

inline void DigitalReceiver::read_mission() {
  int raw_value = analogRead(AMI);
  int mapped_value = map(constrain(raw_value, 0, ADC_MAX_VALUE), 0, ADC_MAX_VALUE, 0,
                         MAX_MISSION);  // constrain just in case
  Mission latest_mission = static_cast<Mission>(mapped_value);

  if ((latest_mission == system_data_->mission_) && (latest_mission == last_tried_mission_)) {
    mission_change_counter_ = 0;
  } else {
    mission_change_counter_++;
  }
  this->last_tried_mission_ = latest_mission;
  if (mission_change_counter_ >= CHANGE_COUNTER_LIMIT) {
    system_data_->mission_ = latest_mission;
    mission_change_counter_ = 0;
  }
}

inline void DigitalReceiver::read_asms_switch() {
  bool latest_asms_status = digitalRead(ASMS_IN_PIN);
  debounce(latest_asms_status, system_data_->hardware_data_.asms_on_, asms_change_counter_);
}

// inline void DigitalReceiver::read_asats_state() {
//   bool asats_pressed = !digitalRead(ASATS);
//   debounce(asats_pressed, system_data_->hardware_data_.asats_pressed_, aats_change_counter_);
// }

inline void DigitalReceiver::read_ats() {
  bool ats_pressed = digitalRead(ATS);
  // DEBUG_PRINT_VAR(ats_pressed);
  debounce(ats_pressed, system_data_->hardware_data_.ats_pressed_, ats_change_counter_);
}

inline void DigitalReceiver::read_watchdog_ready() {
  bool wd_ready = digitalRead(WD_READY);
  debounce(wd_ready, system_data_->hardware_data_.wd_ready_, wd_ready_change_counter_);
}

inline void DigitalReceiver::read_rpm() {
  unsigned long time_interval_rr = (last_wheel_pulse_rr - second_to_last_wheel_pulse_rr);
  unsigned long time_interval_rl = (last_wheel_pulse_rl - second_to_last_wheel_pulse_rl);
  if (micros() - last_wheel_pulse_rr > LIMIT_RPM_INTERVAL) {
    system_data_->hardware_data_._right_wheel_rpm = 0.0;
  } else {
    system_data_->hardware_data_._right_wheel_rpm =
        1 / (time_interval_rr * MICRO_TO_SECONDS * PULSES_PER_ROTATION) * SECONDS_IN_MINUTE;
  }
  if (micros() - last_wheel_pulse_rl > LIMIT_RPM_INTERVAL) {
    system_data_->hardware_data_._left_wheel_rpm = 0.0;
  } else {
    system_data_->hardware_data_._left_wheel_rpm =
        1 / (time_interval_rl * MICRO_TO_SECONDS * PULSES_PER_ROTATION) * SECONDS_IN_MINUTE;
  }
}