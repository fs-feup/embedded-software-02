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

  // unsigned long tsms_open_time_ = 0;  ///< Time when TSMS opened
  // unsigned long counter = 0;
  // bool opened = false;
  // bool tsms_was_closed_ = false;  ///< Track previous TSMS state

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

  void process(uint8_t current_master_state, uint8_t current_checkup_state, uint8_t ebs_state) {
    // dash_ats_update(current_master_state);
    update_physical_outputs();
    if (process_timer_.check()) {
      send_soc();
      send_asms();
      send_mission_update();
      send_state_update(current_master_state);
      send_ebs_state(current_checkup_state, ebs_state);
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
    this->system_data_->hardware_data_.master_sdc_closed_ = false;
    DEBUG_PRINT("Entering emergency state");
  }

  /**
   * @brief Everything off, sdc closed.
   */
  void enter_manual_state() {
    digital_sender_->turn_off_assi();
    digital_sender_->deactivate_ebs();
    digital_sender_->close_sdc();
    this->system_data_->hardware_data_.master_sdc_closed_ = false;
    DEBUG_PRINT("Entering manual driving");
  }

  /**
   * @brief Everything off, sdc open.
   */
  void enter_off_state() {
    digital_sender_->turn_off_assi();
    digital_sender_->deactivate_ebs();
    digital_sender_->open_sdc();
    this->system_data_->hardware_data_.master_sdc_closed_ = false;
    DEBUG_PRINT("Entering off state");
  }

  /**
   * @brief ASSI yellow LED on, ebs valves activated, sdc closed.
   */
  void enter_ready_state() {
    digital_sender_->turn_off_assi();
    digital_sender_->turn_on_yellow();
    digital_sender_->activate_ebs();
    digital_sender_->close_sdc();
    this->system_data_->hardware_data_.master_sdc_closed_ = true;
    DEBUG_PRINT("Entering ready state");
  }

  /**
   * @brief ASSI LEDs yellow flashing, ebs valves deactivated, sdc closed.
   */
  void enter_driving_state() {
    digital_sender_->turn_off_assi();
    blink_timer_.reset();
    digital_sender_->deactivate_ebs();
    digital_sender_->close_sdc();
    this->system_data_->hardware_data_.master_sdc_closed_ = true;
    DEBUG_PRINT("Entering driving state");
  }

  /**
   * @brief ASSI blue LED on, ebs valves activated, sdc open.
   */
  void enter_finish_state() {
    digital_sender_->turn_off_assi();
    digital_sender_->turn_on_blue();
    digital_sender_->activate_ebs();
    digital_sender_->open_sdc();
    this->system_data_->hardware_data_.master_sdc_closed_ = false;
    DEBUG_PRINT("Entering finished state");
  }

  void refresh_r2d_vars() { this->system_data_->r2d_logics_.refresh_r2d_vars(); }
  void refresh_emergency_vars() {
    this->system_data_->failure_detection_.emergency_signal_ = false;
    this->system_data_->failure_detection_.steer_dead_ = false;
    this->system_data_->failure_detection_.pc_dead_ = false;
    this->system_data_->failure_detection_.inversor_dead_ = false;
    this->system_data_->failure_detection_.res_dead_ = false;
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
  void send_ebs_state(uint8_t current_checkup_state, uint8_t ebs_state) {
    static constexpr uint8_t ASB_EBS_STATE_OFF = 1;
    static constexpr uint8_t ASB_EBS_STATE_INITIAL_CHECKUP_PASSED = 2;
    static constexpr uint8_t ASB_EBS_STATE_ACTIVATED = 3;

    static constexpr uint8_t ASB_REDUNDANCY_STATE_DEACTIVATED = 1;
    static constexpr uint8_t ASB_REDUNDANCY_STATE_ENGAGED = 2;
    static constexpr uint8_t ASB_REDUNDANCY_STATE_INITIAL_CHECKUP_PASSED = 3;
    switch (ebs_state) {
      case 0:  // DISABLE_ACTUATOR_1
        Communicator::publish_ebs_states(ASB_EBS_STATE_OFF, false);
        Communicator::publish_ebs_states(ASB_REDUNDANCY_STATE_ENGAGED, true);
        break;

      case 1:  // CHECK_ACTUATOR_2
        Communicator::publish_ebs_states(ASB_EBS_STATE_OFF, false);
        Communicator::publish_ebs_states(ASB_REDUNDANCY_STATE_ENGAGED, true);
        break;

      case 2:  // CHANGE_ACTUATORS
        Communicator::publish_ebs_states(ASB_EBS_STATE_ACTIVATED, false);
        Communicator::publish_ebs_states(ASB_REDUNDANCY_STATE_DEACTIVATED, true);
        break;

      case 3:  // CHECK_ACTUATOR_1
        Communicator::publish_ebs_states(ASB_EBS_STATE_ACTIVATED, false);
        Communicator::publish_ebs_states(ASB_REDUNDANCY_STATE_DEACTIVATED, true);
        break;

      case 4:  // ENABLE_ACTUATOR_2
        Communicator::publish_ebs_states(ASB_EBS_STATE_ACTIVATED, false);
        Communicator::publish_ebs_states(ASB_REDUNDANCY_STATE_ENGAGED, true);
        break;

      case 5:  // CHECK_BOTH_ACTUATORS
        Communicator::publish_ebs_states(ASB_EBS_STATE_ACTIVATED, false);
        Communicator::publish_ebs_states(ASB_REDUNDANCY_STATE_ENGAGED, true);
        break;

      case 6:  // COMPLETE
        Communicator::publish_ebs_states(ASB_EBS_STATE_INITIAL_CHECKUP_PASSED, false);
        Communicator::publish_ebs_states(ASB_REDUNDANCY_STATE_INITIAL_CHECKUP_PASSED, true);
        break;

      default:
        Communicator::publish_ebs_states(ASB_EBS_STATE_OFF, false);
        Communicator::publish_ebs_states(ASB_REDUNDANCY_STATE_DEACTIVATED, true);
        break;
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
    // bsdp_sdc_update();
    //  digital_sender_->turn_on_blue();
    //  digital_sender_->turn_on_yellow();
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
  // void dash_ats_update(uint8_t current_master_state) {
  //   // DEBUG_PRINT("=== ATS Update Debug ===");
  //   // DEBUG_PRINT("ATS Pressed: " + String(system_data_->hardware_data_.ats_pressed_));
  //   // DEBUG_PRINT("Current Master State: " + String(current_master_state) +
  //   //             " (AS_MANUAL=" + String(to_underlying(State::AS_MANUAL)) + ")");
  //   // DEBUG_PRINT("TSMS SDC Closed: " +
  //   // String(system_data_->hardware_data_.tsms_sdc_closed_)); if
  //   // (system_data_->hardware_data_.tsms_sdc_closed_) {
  //   //   opened_again = false;
  //   // }

  //   // if (tsms_was_closed_ && !system_data_->hardware_data_.tsms_sdc_closed_) {
  //   //   DEBUG_PRINT(">>> TSMS opened - recording time for 100ms delay");
  //   //   tsms_open_time_ = millis();
  //   //   opened = true;
  //   // }

  //   // DEBUG_PRINT_VAR(system_data_->hardware_data_.tsms_sdc_closed_);
  //   // DEBUG_PRINT_VAR(tsms_open_time_);
  //   // DEBUG_PRINT_VAR(tsms_was_closed_);
  //   // DEBUG_PRINT_VAR(this->counter);
  //   // DEBUG_PRINT_VAR(this->opened);
  //   // DEBUG_PRINT_VAR(this->system_data_->hardware_data_.master_sdc_closed_);
  //   // if (tsms_open_time_ > 0) {
  //   //   this->counter++;

  //   // }

  //   if (system_data_->hardware_data_.ats_pressed_ &&
  //     current_master_state == to_underlying(State::AS_MANUAL) &&
  //     system_data_->hardware_data_.tsms_sdc_closed_) {
  //       // DEBUG_PRINT(">>> CLOSING SDC - All conditions met");
  //       digital_sender_->close_sdc();
  //       // tsms_was_closed_ = system_data_->hardware_data_.tsms_sdc_closed_;

  //     this->system_data_->hardware_data_.master_sdc_closed_ = true;
  //   } else if (!system_data_->hardware_data_.tsms_sdc_closed_
  //     // && tsms_open_time_ > 0 &&
  //     //          (millis() - tsms_open_time_) >= 100
  //             )
  //              {
  //     // DEBUG_PRINT(">>> OPENING SDC - TSMS SDC not closed (100ms delay elapsed)");
  //     digital_sender_->open_sdc();
  //     this->system_data_->hardware_data_.master_sdc_closed_ = false;
  //   } else {
  //     // DEBUG_PRINT(">>> NO SDC ACTION - Conditions not met");
  //     if (!system_data_->hardware_data_.ats_pressed_) {
  //       // DEBUG_PRINT("    - ATS not pressed");
  //     }
  //     if (current_master_state != to_underlying(State::AS_MANUAL)) {
  //       // DEBUG_PRINT("    - Not in AS_MANUAL state");
  //     }
  //   }
  //   DEBUG_PRINT("========================");
  // }
  void send_rpm() { Communicator::publish_rpm(); }
};
