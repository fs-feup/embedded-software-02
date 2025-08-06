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
Communicator communicator = Communicator(&system_data);  // CAN
DigitalReceiver digital_receiver = DigitalReceiver(&system_data);
DigitalSender digital_sender = DigitalSender();
OutputCoordinator output_coordinator =
    OutputCoordinator(&system_data, &communicator, &digital_sender);
ASState as_state = ASState(&system_data, &communicator, &output_coordinator);
TeensyTimerTool::PeriodicTimer watchdog_timer_;
bool is_first_loop = true;
void setup() {
  Serial.begin(9600);
  Communicator::_systemData = &system_data;
  communicator.init();
  output_coordinator.init();
  DEBUG_PRINT("Starting up...");
  delay(100);
}

void print_all_board_temps() {
  Serial.println("\n--- ALL NTC SENSOR DATA ---");

  for (int board = 0; board < 6; board++) {
    Serial.printf("\n=== BOARD %d ===\n", board + 1);

    for (int sensor = 0; sensor < NTC_SENSOR_COUNT; sensor++) {
      int8_t temp = system_data.cell_board_all_temps[board][sensor];
      Serial.printf("Sensor %2d: %3d°C\n", sensor, temp);
    }
  }
}

void loop() {
  digital_receiver.digital_reads();
  noInterrupts();
  system_data.updated_timestamps_ = system_data.updatable_timestamps_;
  interrupts();

  as_state.calculate_state();
  uint8_t current_master_state = to_underlying(as_state.state_);
  uint8_t current_checkup_state = to_underlying(as_state._checkup_manager_.checkup_state_);
  uint8_t ebs_state = to_underlying(as_state._checkup_manager_.pressure_test_phase_);

  output_coordinator.process(current_master_state, current_checkup_state, ebs_state);
  // print_all_board_temps();
  delay(LOOP_DELAY);
}