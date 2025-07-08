#include <Arduino.h>

#include "../../CAN_IDs.h"
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
constexpr uint8_t MAIN_LOOP_INTERVAL = 50;

SPI_MSTransfer_T4<&SPI> display_spi;
IOManager io_manager(data, updatable_data, updated_data);
CanCommHandler can_comm_handler(data, updatable_data, updated_data/*, display_spi*/);
LogicHandler logic_handler(data, updated_data);
StateMachine state_machine(can_comm_handler, logic_handler, io_manager);

void setup() {
  Serial.begin(115200);

  io_manager.setup();
  // delay(10);
  io_manager.manage();
  // delay(100);

  can_comm_handler.setup();  delay(100);

  Serial.println("Initializing SPI");
  Serial.println("Initializing SPI");
  Serial.println("Initializing SPI");
  Serial.println("Initializing SPI");
  Serial.println("Initializing SPI");
  Serial.println("Initializing SPI");
  delay(1000);
  Serial.println("AAAAAAAAAAAAA");

  display_spi.begin();  delay(100);

  Serial.println("Initializing SPI DONE");
  Serial.println("Initializing SPI DONE");
  Serial.println("Initializing SPI DONE");
  Serial.println("Initializing SPI DONE");
  Serial.println("Initializing SPI DONE");
  Serial.println("Initializing SPI DONE");
  Serial.println("Initializing SPI DONE");
}

void loop() {
  if (loop_timer >= MAIN_LOOP_INTERVAL) {
    Serial.println("chill");
    io_manager.manage();
    can_comm_handler.write_messages();
    copy_volatile_data(updated_data, updatable_data);
    state_machine.update();

    loop_timer = 0;
    Serial.flush();

    static uint16_t current_form;  // Define a static array to hold the values
    static uint16_t random_inverter_errors = 0b1010101000000000;
    if (data.display_pressed) {
      current_form = (current_form == 1) ? 2 : 1;  // toggle 1 and 2
      display_spi.transfer16(&current_form, 1, WIDGET_FORM_CMD, millis() & 0xFFFF);
      display_spi.transfer16(&random_inverter_errors, 1, WIDGET_INVERTER_ERRORS, millis() & 0xFFFF);
      // DBUG_PRINT_VAR(widgetID);
      data.display_pressed = false;
    }
  }
}
