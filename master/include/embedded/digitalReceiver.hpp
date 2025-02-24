// ReSharper disable CppMemberFunctionMayBeConst
#pragma once

#include <Bounce2.h>

#include <model/hardwareData.hpp>
#include <model/structure.hpp>

#include "debugUtils.hpp"
#include "hardwareSettings.hpp"

/**
 * @brief Class responsible for the reading of the digital
 * inputs into the Master teensy
 */
class DigitalReceiver {
public:
  static double _current_left_wheel_rpm;  // Class variable to store the left wheel RPM (non-static)
  static unsigned long last_wheel_pulse_ts;  // Timestamp of the last pulse for RPM calculation

  /**
   * @brief read all digital inputs
   */
  void digital_reads();

  /**
   * @brief callback to update rl wheel rpm
   */
  static void updateLeftWheelRpm();

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

  }

private:
  HardwareData *hardware_data_;  ///< Pointer to the digital data storage
  Mission *mission_;             ///< Pointer to the current mission status

  unsigned int asms_change_counter_ = 0;          ///< counter to avoid noise on asms
  unsigned int aats_change_counter_ = 0;          ///< counter to avoid noise on aats
  unsigned int pneumatic_change_counter_ = 0;     ///< counter to avoid noise on pneumatic line
  unsigned int mission_change_counter_ = 0;       ///< counter to avoid noise on mission change
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
  void read_aats_state();
};

inline void DigitalReceiver::digital_reads() {
  read_pneumatic_line();
  read_mission();
  read_asms_switch();
  read_aats_state();
}

inline void DigitalReceiver::read_pneumatic_line() {
  // bool pneumatic1 = digitalRead(EBS_SENSOR2);
  bool pneumatic2 = digitalRead(EBS_SENSOR1);  // TODO: maybe poorly read

  // hardware_data_->pneumatic_line_pressure_1_ = pneumatic1;
  hardware_data_->pneumatic_line_pressure_2_ = pneumatic2;
  bool latest_pneumatic_pressure = pneumatic2;

  // Only change the value if it has been different 5 times in a row
  pneumatic_change_counter_ = latest_pneumatic_pressure == hardware_data_->pneumatic_line_pressure_
                                  ? 0
                                  : pneumatic_change_counter_ + 1;
  if (pneumatic_change_counter_ >= DIGITAL_INPUT_COUNTER_LIMIT) {
    hardware_data_->pneumatic_line_pressure_ = latest_pneumatic_pressure;  // both need to be True
    pneumatic_change_counter_ = 0;
  }
}

inline void DigitalReceiver::read_mission() {
  Mission latest_mission = static_cast<Mission>(5);

  mission_change_counter_ = (latest_mission == *mission_) && (latest_mission == last_tried_mission_)
                                ? 0
                                : mission_change_counter_ + 1;
  this->last_tried_mission_ = latest_mission;
  if (mission_change_counter_ >= DIGITAL_INPUT_COUNTER_LIMIT) {
    *mission_ = latest_mission;
    mission_change_counter_ = 0;
  }
}

inline void DigitalReceiver::read_asms_switch() {
  bool latest_asms_status = digitalRead(ASMS_IN_PIN);

  asms_change_counter_ =
      latest_asms_status == hardware_data_->asms_on_ ? 0 : asms_change_counter_ + 1;
  if (asms_change_counter_ >= DIGITAL_INPUT_COUNTER_LIMIT) {
    hardware_data_->asms_on_ = latest_asms_status;
    asms_change_counter_ = 0;
  }
}

inline void DigitalReceiver::read_aats_state() {
  // AATS is on if SDC is closed (SDC STATE PIN AS HIGH)
  bool is_sdc_closed = !digitalRead(SDC_BSPD_STATE_PIN);
  aats_change_counter_ = is_sdc_closed == hardware_data_->sdc_open_ ? 0 : aats_change_counter_ + 1;
  if (aats_change_counter_ >= DIGITAL_INPUT_COUNTER_LIMIT) {
    hardware_data_->sdc_open_ = is_sdc_closed;  // both need to be True
    aats_change_counter_ = 0;
  }
}
