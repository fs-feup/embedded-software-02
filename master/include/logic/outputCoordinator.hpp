#pragma once

#include "comm/communicator.hpp"
#include "debugUtils.hpp"
#include "embedded/digitalSender.hpp"
#include "enum_utils.hpp"
#include "model/systemData.hpp"
#include "timings.hpp"

class OutputCoordinator {
private:
  SystemData* system_data_;
  Communicator* communicator_;
  DigitalSender* digital_sender_;

  Metro mission_timer_;
  Metro state_timer_;

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
    send_soc();
    send_asms();
    send_debug_on_state_change(current_master_state, current_checkup_state);
    send_mission_update();
    send_state_update(current_master_state);
    update_physical_outputs();
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

  void send_soc() {
    Communicator::publish_soc(system_data_->hardware_data_.soc_);
  }

  void send_asms(){
    Communicator::publish_asms_on(system_data_->hardware_data_.asms_on_);
  }

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
};
