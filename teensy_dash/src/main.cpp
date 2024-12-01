#include <Arduino.h>
#include "hw_io_manager.hpp"
#include "can_comm_handler.hpp"
#include "logic_handler.hpp"
# include "data_struct.hpp"

SystemData data;
volatile SystemData data_copy;

IOManager io_manager(data_copy);
CanCommHandler can_comm_handler(data_copy);
LogicHandler logic_handler(data);


void setup() {
}

void loop() {
    io_manager.manage();
    noInterrupts();
    data = data_copy;
    interrupts();
    //run state machine

}

