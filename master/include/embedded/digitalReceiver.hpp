// ReSharper disable CppMemberFunctionMayBeConst
#pragma once

#include <Bounce2.h>

#include <model/hardwareData.hpp>
#include <model/structure.hpp>

#include "debugUtils.hpp"
#include "hardwareSettings.hpp"
#include "utils.hpp"

/**
 * @brief Class responsible for the reading of the digital
 * inputs into the Master teensy
 */
class DigitalReceiver {
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
  DigitalReceiver(HardwareData *digital_data, Mission *mission)
      : hardware_data_(digital_data), mission_(mission) {
    pinMode(SDC_BSPD_STATE_PIN, INPUT);
    pinMode(AMI, INPUT);
    pinMode(ASMS_IN_PIN, INPUT);
    pinMode(EBS_SENSOR2, INPUT);
    pinMode(EBS_SENSOR1, INPUT);

    pinMode(RL_WSS, INPUT);
    pinMode(RR_WSS, INPUT);
    pinMode(BRAKE_SENSOR, INPUT);
    pinMode(SOC, INPUT);
    pinMode(ATS, INPUT);
    pinMode(ASATS, INPUT);
    pinMode(WD_READY, INPUT);
    pinMode(WD_SDC_RELAY, INPUT);

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
  HardwareData *hardware_data_;  ///< Pointer to the digital data storage
  Mission *mission_;             ///< Pointer to the current mission status

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
  read_pneumatic_line();
  read_mission();
  read_asms_switch();
  read_asats_state();
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
  hardware_data_->soc_ = static_cast<uint8_t>(mapped_value);
}

inline void DigitalReceiver::read_bspd_sdc() {
  bool is_sdc_open = (0 == digitalRead(SDC_BSPD_STATE_PIN));  // low when sdc/bspd open
  debounce(is_sdc_open, hardware_data_->bspd_sdc_open_, sdc_bspd_change_counter_);
}
inline void DigitalReceiver::read_brake_sensor() {
  int hydraulic_pressure = analogRead(BRAKE_SENSOR);
  insert_value_queue(hydraulic_pressure, brake_readings);
  hardware_data_->_hydraulic_line_pressure = average_queue(brake_readings);
}
inline void DigitalReceiver::read_pneumatic_line() {
  bool pneumatic1 = digitalRead(EBS_SENSOR2);
  bool pneumatic2 = digitalRead(EBS_SENSOR1);

  hardware_data_->pneumatic_line_pressure_1_ = pneumatic1;
  hardware_data_->pneumatic_line_pressure_2_ = pneumatic2;
  bool latest_pneumatic_pressure = pneumatic1 && pneumatic2;

  debounce(latest_pneumatic_pressure, hardware_data_->pneumatic_line_pressure_,
           pneumatic_change_counter_);
}

inline void DigitalReceiver::read_mission() {
  const int raw_value = analogRead(AMI);
  int mapped_value = map(constrain(raw_value, 0, ADC_MAX_VALUE), 0, ADC_MAX_VALUE, 0, MAX_MISSION);  // constrain just in case
  Mission latest_mission = static_cast<Mission>(mapped_value);

  if ((latest_mission == *mission_) && (latest_mission == last_tried_mission_)) {
    mission_change_counter_ = 0;
  } else {
    mission_change_counter_++;
  }
  this->last_tried_mission_ = latest_mission;
  if (mission_change_counter_ >= CHANGE_COUNTER_LIMIT) {
    *mission_ = latest_mission;
    mission_change_counter_ = 0;
  }
  *mission_ = Mission::MANUAL;
}

inline void DigitalReceiver::read_asms_switch() {
  bool latest_asms_status = digitalRead(ASMS_IN_PIN);
  debounce(latest_asms_status, hardware_data_->asms_on_, asms_change_counter_);
}

inline void DigitalReceiver::read_asats_state() {
  bool asats_pressed = !digitalRead(ASATS);
  debounce(asats_pressed, hardware_data_->asats_pressed_, aats_change_counter_);
}

inline void DigitalReceiver::read_ats() {
  bool ats_pressed = digitalRead(ATS);
  // DEBUG_PRINT_VAR(ats_pressed);
  debounce(ats_pressed, hardware_data_->ats_pressed_, ats_change_counter_);
}

inline void DigitalReceiver::read_watchdog_ready() {
  bool wd_ready = digitalRead(WD_READY);

  debounce(wd_ready, hardware_data_->wd_ready_, wd_ready_change_counter_);
}

inline void DigitalReceiver::read_rpm() {
  unsigned long time_interval_rr = (last_wheel_pulse_rr - second_to_last_wheel_pulse_rr);
  unsigned long time_interval_rl = (last_wheel_pulse_rl - second_to_last_wheel_pulse_rl);
  if (micros() - last_wheel_pulse_rr > LIMIT_RPM_INTERVAL) {
    hardware_data_->rr_wheel_rpm = 0.0;
  } else {
    hardware_data_->rr_wheel_rpm =
        1 / (time_interval_rr * MICRO_TO_SECONDS * PULSES_PER_ROTATION) * SECONDS_IN_MINUTE;
  }
  if (micros() - last_wheel_pulse_rl > LIMIT_RPM_INTERVAL) {
    hardware_data_->rl_wheel_rpm = 0.0;
  } else {
    hardware_data_->rl_wheel_rpm =
        1 / (time_interval_rl * MICRO_TO_SECONDS * PULSES_PER_ROTATION) * SECONDS_IN_MINUTE;
  }
}