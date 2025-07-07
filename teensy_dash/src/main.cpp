#include <Arduino.h>

#include "can_comm_handler.hpp"
#include "data_struct.hpp"
#include "hw_io_manager.hpp"
#include "logic_handler.hpp"
#include "spi/SPI_MSTransfer_T4.h"
#include "state_machine.hpp"

SystemData data;
SystemVolatileData updated_data;
volatile SystemVolatileData updatable_data;
elapsedMillis loop_timer;
constexpr uint8_t MAIN_LOOP_INTERVAL = 10;

SPI_MSTransfer_T4<&SPI> display_spi;
IOManager io_manager(data, updatable_data, updated_data);
CanCommHandler can_comm_handler(data, updatable_data, updated_data, display_spi);
LogicHandler logic_handler(data, updated_data);
StateMachine state_machine(can_comm_handler, logic_handler, io_manager);

void setup() {
  Serial.begin(9600);

  display_spi.begin();
  io_manager.setup();
  delay(10);
  io_manager.manage();
  can_comm_handler.setup();
}

void loop() {
  if (loop_timer >= MAIN_LOOP_INTERVAL) {
    io_manager.manage();
    can_comm_handler.write_messages();
    copy_volatile_data(updated_data, updatable_data);
    state_machine.update();
    loop_timer = 0;
  }
}
