#include "comm/communicator.hpp"
#include "debugUtils.hpp"
#include "embedded/digitalReceiver.hpp"
#include "embedded/digitalSender.hpp"
#include "enum_utils.hpp"
#include "logic/outputCoordinator.hpp"
#include "logic/stateLogic.hpp"
#include "model/systemData.hpp"
#include "timings.hpp"

SystemData system_data;
SystemData
    system_data_copy;  // Copy of the model for Communicator (where interrupt updates are stored)
Communicator communicator = Communicator(&system_data_copy);  // CAN
DigitalReceiver digital_receiver =
    DigitalReceiver(&system_data_copy);
DigitalSender digital_sender = DigitalSender();
OutputCoordinator output_coordinator =
    OutputCoordinator(&system_data, &communicator, &digital_sender);
ASState as_state = ASState(&system_data, &communicator, &output_coordinator);
TeensyTimerTool::PeriodicTimer watchdog_timer_;
bool is_first_loop = true;
void setup() {
  Serial.begin(9600);
  Communicator::_systemData = &system_data_copy;
  communicator.init();
  output_coordinator.init();
  DEBUG_PRINT("Starting up...");
  delay(100);
  
}

void loop() {
  if (is_first_loop) {
    watchdog_timer_.begin([] { DigitalSender::toggle_watchdog(); }, 10'000);
    is_first_loop = false;
  }
  digitalWrite(WD_SDC_CLOSE, HIGH);
  digital_receiver.digital_reads();
  noInterrupts();
  system_data = system_data_copy;
  interrupts();

  as_state.calculate_state();

  uint8_t current_master_state = to_underlying(as_state.state_);
  uint8_t current_checkup_state = to_underlying(as_state._checkup_manager_.checkup_state_);

  output_coordinator.process(current_master_state, current_checkup_state);

} 