#include <Arduino.h>

#include "can_comm_handler.hpp"
#include "data_struct.hpp"
#include "hw_io_manager.hpp"
#include "logic_handler.hpp"
#include "state_machine.hpp"

SystemData data;
SystemVolatileData updated_data;
volatile SystemVolatileData updatable_data;
elapsedMillis loop_timer;
constexpr uint8_t MAIN_LOOP_INTERVAL = 100;

IOManager io_manager(data, updatable_data, updated_data);
CanCommHandler can_comm_handler(data, updatable_data, updated_data);
LogicHandler logic_handler(data, updated_data);
StateMachine state_machine(can_comm_handler, logic_handler, io_manager);

void setup() {
  io_manager.setup();
  can_comm_handler.setup();
}

void loop() {
  if (loop_timer >= 100) {
    io_manager.manage();
    can_comm_handler.write_periodic_messages();
    copy_volatile_data(updated_data, updatable_data);
    state_machine.update();

    loop_timer = 0;
  }
}
