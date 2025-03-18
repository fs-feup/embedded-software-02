#pragma once

#include "TeensyTimerTool.h"
#include "comm/communicator.hpp"
#include "debugUtils.hpp"
#include "embedded/digitalSender.hpp"
#include "enum_utils.hpp"
#include "metro.h"
#include "model/systemData.hpp"
#include "timings.hpp"

class OutputCoordinator {
private:
  TeensyTimerTool::PeriodicTimer watchdog_timer_;
  Metro blink_timer_{LED_BLINK_INTERVAL};  ///< Timer for blinking LED

  SystemData* system_data_;
  Communicator* communicator_;
  DigitalSender* digital_sender_;

  Metro mission_timer_;
  Metro state_timer_;
  Metro process_timer_{PROCESS_INTERVAL};

  uint8_t previous_master_state_;
  uint8_t previous_checkup_state_;
  uint8_t previous_mission_;

public:
  OutputCoordinator(SystemData* system_data, Communicator* communicator,
                    DigitalSender* digital_sender)
      : system_data_(system_data),
        communicator_(communicator),
        digital_sender_(digital_sender),
        mission_timer_(MISSION_PUBLISH_INTERVAL),
        state_timer_(STATE_PUBLISH_INTERVAL),
        previous_master_state_(static_cast<uint8_t>(15)),
        previous_checkup_state_(static_cast<uint8_t>(15)),
        previous_mission_(static_cast<uint8_t>(15)) {}

  void init() {
    mission_timer_.reset();
    state_timer_.reset();
    DEBUG_PRINT("Output coordinator initialized...");
  }

  void process(uint8_t current_master_state, uint8_t current_checkup_state) {
    if (process_timer_.check()) {
      send_soc();
      send_asms();
      send_debug_on_state_change(current_master_state, current_checkup_state);
      send_mission_update();
      send_state_update(current_master_state);
      dash_ats_update(current_master_state);
      update_physical_outputs();
      send_rpm();
      process_timer_.reset();
    }
  }

  void blink_emergency_led() {
    if (blink_timer_.check()) {
      digital_sender_->blink_led(ASSI_BLUE_PIN);
    }
  }

  void blink_driving_led() {
    if (blink_timer_.check()) {
      digital_sender_->blink_led(ASSI_YELLOW_PIN);
    }
  }

  /**
   * @brief ASSI LEDs blue flashing, sdc open and buzzer ringing.
   */
  void enter_emergency_state() {
    digital_sender_->turn_off_assi();
    blink_timer_.reset();
    digital_sender_->activate_ebs();
    digital_sender_->open_sdc();
  }

  /**
   * @brief Everything off, sdc closed.
   */
  void enter_manual_state() {
    digital_sender_->turn_off_assi();
    digital_sender_->deactivate_ebs();
    digital_sender_->close_sdc();
  }

  /**
   * @brief Everything off, sdc open.
   */
  void enter_off_state() {
    digital_sender_->turn_off_assi();
    digital_sender_->deactivate_ebs();
    digital_sender_->open_sdc();
  }

  /**
   * @brief ASSI yellow LED on, ebs valves activated, sdc closed.
   */
  void enter_ready_state() {
    watchdog_timer_.begin([] { DigitalSender::toggle_watchdog(); }, 50'000);

    digital_sender_->turn_off_assi();
    digital_sender_->turn_on_yellow();
    digital_sender_->activate_ebs();
    digital_sender_->close_sdc();
  }

  /**
   * @brief ASSI LEDs yellow flashing, ebs valves deactivated, sdc closed.
   */
  void enter_driving_state() {
    digital_sender_->turn_off_assi();
    blink_timer_.reset();
    digital_sender_->deactivate_ebs();
    digital_sender_->close_sdc();
  }

  /**
   * @brief ASSI blue LED on, ebs valves activated, sdc open.
   */
  void enter_finish_state() {
    digital_sender_->turn_off_assi();
    digital_sender_->turn_on_blue();
    digital_sender_->activate_ebs();
    digital_sender_->open_sdc();
  }

private:
  // Communication functions
  void send_debug_on_state_change(uint8_t current_master_state, uint8_t current_checkup_state) {
    uint8_t current_mission = to_underlying(system_data_->mission_);

    if (previous_master_state_ != current_master_state ||
        previous_checkup_state_ != current_checkup_state || previous_mission_ != current_mission) {
      previous_master_state_ = current_master_state;
      previous_checkup_state_ = current_checkup_state;
      previous_mission_ = current_mission;

      Communicator::publish_debug_morning_log(*system_data_, current_master_state,
                                              current_checkup_state);
    }
  }

  void send_soc() { Communicator::publish_soc(system_data_->hardware_data_.soc_); }

  void send_asms() { Communicator::publish_asms_on(system_data_->hardware_data_.asms_on_); }

  void send_mission_update() {
    if (mission_timer_.check()) {
      Communicator::publish_mission(to_underlying(system_data_->mission_));
      mission_timer_.reset();
    }
  }

  void send_state_update(uint8_t current_master_state) {
    if (state_timer_.check()) {
      Communicator::publish_state(current_master_state);
      state_timer_.reset();
    }
  }

  void update_physical_outputs() {
    brake_light_update();
    bsdp_sdc_update();
  }

  void brake_light_update() {
    int brake_val = system_data_->hardware_data_.hydraulic_pressure_;
    if (brake_val >= BRAKE_PRESSURE_LOWER_THRESHOLD &&
        brake_val <= BRAKE_PRESSURE_UPPER_THRESHOLD) {
      digital_sender_->turn_on_brake_light();
    } else {
      digital_sender_->turn_off_brake_light();
    }
  }
  void bsdp_sdc_update() {
    if (system_data_->hardware_data_.bspd_sdc_open_) {
      digital_sender_->bspd_error();
    } else {
      digital_sender_->no_bspd_error();
    }
  }
  void dash_ats_update(uint8_t current_master_state) {
    if (system_data_->hardware_data_.ats_pressed_ &&
        current_master_state == to_underlying(State::AS_MANUAL)) {
      digital_sender_->close_sdc();
    } else {
      digital_sender_->open_sdc();
    }
  }
  void send_rpm() { Communicator::publish_rpm(); }
};
