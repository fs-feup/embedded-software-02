#include "comm/communicator.hpp"
#include "debugUtils.hpp"
#include "embedded/digitalReceiver.hpp"
#include "embedded/digitalSender.hpp"
#include "enum_utils.hpp"
#include "logic/outputCoordinator.hpp"  // Updated path
#include "logic/stateLogic.hpp"
#include "model/systemData.hpp"
#include "timings.hpp"

SystemData system_data;
SystemData
    system_data_copy;  // Copy of the model for Communicator (where interrupt updates are stored)
Communicator communicator = Communicator(&system_data_copy);  // CAN
DigitalReceiver digital_receiver =
    DigitalReceiver(&system_data_copy.hardware_data_, &system_data_copy.mission_);
DigitalSender digital_sender = DigitalSender();
OutputCoordinator output_coordinator =
    OutputCoordinator(&system_data, &communicator, &digital_sender);
ASState as_state = ASState(&system_data, &communicator, &output_coordinator);
Metro rl_rpm_timer = Metro{LEFT_WHEEL_PUBLISH_INTERVAL};

void setup() {
  Serial.begin(9600);
  Communicator::_systemData = &system_data_copy;
  communicator.init();
  output_coordinator.init();
  rl_rpm_timer.reset();
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

  output_coordinator.process(current_master_state, current_checkup_state);
}