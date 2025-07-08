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
  Metro blink_timer_{LED_BLINK_INTERVAL};  ///< Timer for blinking LED

  SystemData* system_data_;
  Communicator* communicator_;
  DigitalSender* digital_sender_;

  Metro mission_timer_;
  Metro state_timer_;
  Metro process_timer_{PROCESS_INTERVAL};
  Metro slower_process_timer_{SLOWER_PROCESS_INTERVAL};

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
    dash_ats_update(current_master_state);
    update_physical_outputs();
    if (process_timer_.check()) {
      send_soc();
      send_asms();
      send_mission_update();
      send_state_update(current_master_state);
    }
    if (slower_process_timer_.check()) {
      send_debug_on_state_change(current_master_state, current_checkup_state);
      send_rpm();
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
    digital_sender_->open_sdc();
    // digital_sender_->close_sdc(); // close sdc only when ats pressed and in manual mode already
    DEBUG_PRINT("Entering manual state...");
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
    Communicator::publish_debug_morning_log(*system_data_, current_master_state,
                                            current_checkup_state);
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
    // digital_sender_->turn_on_blue();
    // digital_sender_->turn_on_yellow();
  }

  void brake_light_update() {
    int brake_val = system_data_->hardware_data_._hydraulic_line_pressure;
    // DEBUG_PRINT("Brake pressure: " + String(brake_val));
    // DEBUG_PRINT("Brake pressure lower threshold: " +
    //            String(BRAKE_PRESSURE_LOWER_THRESHOLD));
    // DEBUG_PRINT("Brake pressure upper threshold: " +
    //            String(BRAKE_PRESSURE_UPPER_THRESHOLD));

    if (brake_val >= BRAKE_PRESSURE_LOWER_THRESHOLD &&
        brake_val <= BRAKE_PRESSURE_UPPER_THRESHOLD) {
      digital_sender_->turn_on_brake_light();
    } else {
      digital_sender_->turn_off_brake_light();
    }
  }
  void bsdp_sdc_update() {
    // TODO: implement bspd logic, update led from this output in Dash
    if (!system_data_->hardware_data_.tsms_sdc_closed_) {
      digital_sender_->bspd_error();
    } else {
      digital_sender_->no_bspd_error();
    }
  }
  void dash_ats_update(uint8_t current_master_state) {
    DEBUG_PRINT("=== ATS Update Debug ===");
    DEBUG_PRINT("ATS Pressed: " + String(system_data_->hardware_data_.ats_pressed_));
    DEBUG_PRINT("Current Master State: " + String(current_master_state) +
                " (AS_MANUAL=" + String(to_underlying(State::AS_MANUAL)) + ")");
    DEBUG_PRINT("TSMS SDC Closed: " + String(system_data_->hardware_data_.tsms_sdc_closed_));

    if (system_data_->hardware_data_.ats_pressed_ &&
        current_master_state == to_underlying(State::AS_MANUAL) &&
        system_data_->hardware_data_.tsms_sdc_closed_) {
      DEBUG_PRINT(">>> CLOSING SDC - All conditions met");
      digital_sender_->close_sdc();
    } else if (!system_data_->hardware_data_.tsms_sdc_closed_) {
      DEBUG_PRINT(">>> OPENING SDC - TSMS SDC not closed");
      digital_sender_->open_sdc();
    } else {
      DEBUG_PRINT(">>> NO SDC ACTION - Conditions not met");
      if (!system_data_->hardware_data_.ats_pressed_) {
        DEBUG_PRINT("    - ATS not pressed");
      }
      if (current_master_state != to_underlying(State::AS_MANUAL)) {
        DEBUG_PRINT("    - Not in AS_MANUAL state");
      }
    }
    DEBUG_PRINT("========================");
  }
  void send_rpm() { Communicator::publish_rpm(); }
};
