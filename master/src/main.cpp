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

  delay(LOOP_DELAY);
}


// #include "Arduino.h"

// void setup() {
//   Serial.begin(9600);
//   // Initialize pins and other setup tasks here
//   pinMode(25, INPUT);
//   pinMode(22, INPUT);
//   pinMode(21, INPUT);
//   pinMode(20, INPUT);
//   pinMode(19, INPUT);
//   pinMode(12, INPUT);
//   pinMode(17, INPUT);
//   pinMode(13, INPUT);
//   pinMode(14, INPUT);
//   pinMode(2, INPUT);
//   pinMode(40, INPUT);
//   pinMode(15, INPUT);
//   pinMode(18, INPUT);
//   pinMode(41, INPUT);
//   pinMode(39, INPUT);
//   pinMode(5, INPUT);
//   pinMode(4, INPUT);
//   pinMode(38, INPUT);
//   pinMode(24, INPUT);
//   pinMode(16, INPUT);
//   pinMode(37, INPUT);
//   pinMode(33, INPUT);
//   pinMode(10, INPUT);
//   // Additional setup code as needed
// }

// void loop() {
//   Serial.print("Pin 25 state: ");
//   Serial.println(digitalRead(25));
//   Serial.print("Pin 25 analog state: ");
//   Serial.println(analogRead(25));
//   Serial.print("Pin 12 state: ");
//   Serial.println(digitalRead(12));
//   Serial.print("Pin 17 state: ");
//   Serial.println(digitalRead(17));
//   Serial.print("Pin 17 analog state: ");
//   Serial.println(analogRead(17));
//   Serial.print("Pin 13 state: ");
//   Serial.println(digitalRead(13));
//   Serial.print("Pin 21 state: ");
//   Serial.println(digitalRead(21));
//   Serial.print("Pin 21 analog state: ");
//   Serial.println(analogRead(21));
//   Serial.print("Pin 14 state: ");
//   Serial.println(digitalRead(14));
//   Serial.print("Pin 14 analog state: ");
//   Serial.println(analogRead(14));
//   Serial.print("Pin 2 state: ");
//   Serial.println(digitalRead(2));
//   Serial.print("Pin 40 state: ");
//   Serial.println(digitalRead(40));
//   Serial.print("Pin 40 analog state: ");
//   Serial.println(analogRead(40));
//   Serial.print("Pin 15 state: ");
//   Serial.println(digitalRead(15));
//   Serial.print("Pin 15 analog state: ");
//   Serial.println(analogRead(15));
//   Serial.print("Pin 18 state: ");
//   Serial.println(digitalRead(18));
//   Serial.print("Pin 18 analog state: ");
//   Serial.println(analogRead(18));
//   Serial.print("Pin 19 state: ");
//   Serial.println(digitalRead(19));
//   Serial.print("Pin 19 analog state: ");
//   Serial.println(analogRead(19));
//   Serial.print("Pin 41 state: ");
//   Serial.println(digitalRead(41));
//   Serial.print("Pin 41 analog state: ");
//   Serial.println(analogRead(41));
//   Serial.print("Pin 39 state: ");
//   Serial.println(digitalRead(39));
//   Serial.print("Pin 39 analog state: ");
//   Serial.println(analogRead(39));
//   Serial.print("Pin 22 state: ");
//   Serial.println(digitalRead(22));
//   Serial.print("Pin 22 analog state: ");
//   Serial.println(analogRead(22));
//   Serial.print("Pin 5 state: ");
//   Serial.println(digitalRead(5));
//   Serial.print("Pin 4 state: ");
//   Serial.println(digitalRead(4));
//   Serial.print("Pin 38 state: ");
//   Serial.println(digitalRead(38));
//   Serial.print("Pin 38 analog state: ");
//   Serial.println(analogRead(38));
//   Serial.print("Pin 24 state: ");
//   Serial.println(digitalRead(24));
//   Serial.print("Pin 24 analog state: ");
//   Serial.println(analogRead(24));
//   Serial.print("Pin 16 state: ");
//   Serial.println(digitalRead(16));
//   Serial.print("Pin 16 analog state: ");
//   Serial.println(analogRead(16));
//   Serial.print("Pin 20 state: ");
//   Serial.println(digitalRead(20));
//   Serial.print("Pin 20 analog state: ");
//   Serial.println(analogRead(20));
//   Serial.print("Pin 37 state: ");
//   Serial.println(digitalRead(37));
//   Serial.print("Pin 33 state: ");
//   Serial.println(digitalRead(33));
//   Serial.print("Pin 10 state: ");
//   Serial.println(digitalRead(10));
//   Serial.print("\n\n\n\n");
//   delay(5000);  // Delay for readability in the serial monitor
// }