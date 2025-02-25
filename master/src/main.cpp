#include "comm/communicator.hpp"
#include "debugUtils.hpp"
#include "embedded/digitalReceiver.hpp"
#include "embedded/digitalSender.hpp"
#include "enum_utils.hpp"
#include "logic/stateLogic.hpp"
#include "model/systemData.hpp"
#include "timings.hpp"

SystemData system_data;  // Model
SystemData
    system_data_copy;  // Copy of the model for Communicator (where interrupt updates are stored)
Communicator communicator = Communicator(&system_data_copy);  // CAN
DigitalReceiver digital_receiver = DigitalReceiver(&system_data_copy.hardware_data_,
                                                   &system_data_copy.mission_);  // Digital inputs
DigitalSender digital_sender = DigitalSender();                                  // Digital outputs
ASState as_state = ASState(&system_data, &communicator, &digital_sender);

Metro rl_rpm_timer = Metro{LEFT_WHEEL_PUBLISH_INTERVAL};
Metro mission_timer = Metro(MISSION_PUBLISH_INTERVAL);
Metro state_timer = Metro(STATE_PUBLISH_INTERVAL);
// only publih debug log if there is a change in one of the states
uint8_t previous_master_state = static_cast<uint8_t>(15);
uint8_t previous_checkup_state = static_cast<uint8_t>(15);
uint8_t previous_mission = static_cast<uint8_t>(15);

void setup() {
  Serial.begin(9600);
  Communicator::_systemData = &system_data_copy;
  communicator.init();
  rl_rpm_timer.reset();
  mission_timer.reset();
  state_timer.reset();
  DEBUG_PRINT("Starting up...");
}

void loop() {
  digital_receiver.digital_reads();
  noInterrupts();
  system_data = system_data_copy;
  interrupts();
  as_state.calculate_state();

  uint8_t current_master_state = to_underlying(as_state.state_);
  uint8_t current_checkup_state = to_underlying(as_state._checkup_manager_.checkup_state_);
  uint8_t current_mission = to_underlying(system_data.mission_);

  // Publish debug log if there is a change in one of the states or mission
  if (previous_master_state != current_master_state ||
      previous_checkup_state != current_checkup_state || previous_mission != current_mission) {
    previous_master_state = current_master_state;
    previous_checkup_state = current_checkup_state;
    previous_mission = current_mission;

    Communicator::publish_debug_log_not_wood(system_data, current_master_state, current_checkup_state);//TODO: extract logging info, maybe new class
  }

  if (mission_timer.check()) {
    Communicator::publish_mission(current_mission);
    mission_timer.reset();
  }
  if (state_timer.check()) {
    Communicator::publish_state(current_master_state);
    state_timer.reset();
  }
}
