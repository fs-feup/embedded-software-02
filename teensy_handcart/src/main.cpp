#include <Arduino.h>
#include <Bounce2.h>
#include <FlexCAN_T4.h>
#include <elapsedMillis.h>

#include "../../CAN_IDs.h"
#include "SPI_MSTransfer_T4.h"
#include "constants.hpp"
#include "structs.hpp"
#include "utils.hpp"
// #include "../../debugUtils.hpp"

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can1;

SPI_MSTransfer_T4<&SPI> displaySPI;

elapsedMillis step;
elapsedMillis spi_update_timer;
elapsedMillis cell_spi_timer;

PARAMETERS param;

const CAN_message_t HC_msg = {.id = HC_ID, .len = 1, .buf = {0x00}};

bool ch_enable_pin = 1;    // This was CH enable pin status
bool shutdown_status = 0;  // latching status, 1(high) for shutdown
bool sdc_status_pin = 0;
auto display_button = Bounce();
bool display_button_pressed = false;

Status charger_status;  // current state machine status

void handle_set_data_response(const CAN_message_t &message) {
  switch (message.buf[1]) {
    case SET_VOLTAGE_RESPONSE: {
      extract_value(param.set_voltage, message.buf);
      print_value("Voltage Set= ", param.set_voltage);
      break;
    }

    case SET_CURRENT_RESPONSE: {
      extract_value(param.set_current, message.buf);
      print_value("Current Set= ", param.set_current);
      break;
    }

    default: {
      break;
    }
  }
}

void handle_read_data_response(const CAN_message_t &message) {
  switch (message.buf[1]) {
    case CURRENT_VOLTAGE_RESPONSE: {
      extract_value(param.current_voltage, message.buf);

      const uint16_t buf[] = {static_cast<uint16_t>(param.current_voltage)};
      displaySPI.transfer16(buf, 1, WIDGET_VOLTAGE, millis() & 0xFFFF);
      print_value("Current voltage= ", param.current_voltage);
      break;
    }

    case CURRENT_CURRENT_RESPONSE: {
      extract_value(param.current_current, message.buf);

      const uint16_t buf[] = {static_cast<uint16_t>(param.current_current)};
      displaySPI.transfer16(buf, 1, WIDGET_CURRENT, millis() & 0xFFFF);
      print_value("Current Current= ", param.current_current);
      break;
    }

    default: {
      break;
    }
  }
}

void parse_charger_message(const CAN_message_t &message) {
  switch (message.buf[0]) {
    case SET_DATA_RESPONSE: {
      handle_set_data_response(message);
      break;
    }

    case READ_DATA_RESPONSE: {
      handle_read_data_response(message);
      break;
    }

    default: {
      break;
    }
  }
}

void can_snifflas(const CAN_message_t &message) {
  Serial.print("Received CAN message with ID: ");
  Serial.print(message.id, HEX);
  if (message.id == CHARGER_ID) {
    parse_charger_message(message);
  } else if (message.id == BMS_ID_CCL) {
    param.ccl = message.buf[0] * 1000;  // Assuming conversion is correct
    param.ch_safety = message.buf[2] & 0x04;
    // print
    print_value("CCL= ", param.ccl);

    // Serial.print("Discharge relay: "); Serial.println((status & 0x01) ? "ON" : "OFF");
    // Serial.print("Charge relay: "); Serial.println((status & 0x02) ? "ON" : "OFF");
    // Serial.print("Charger safety: "); Serial.println((status & 0x04) ? "ON" : "OFF");
    // Serial.print("Malfunction indicator: "); Serial.println((status & 0x08) ? "ON" : "OFF");
    // Serial.print("Multi-Purpose Input: "); Serial.println((status & 0x10) ? "ON" : "OFF");
    // Serial.print("Always-on signal: "); Serial.println((status & 0x20) ? "ON" : "OFF");
    // Serial.print("Is-Ready signal: "); Serial.println((status & 0x40) ? "ON" : "OFF");
    // Serial.print("Is-Charging signal: "); Serial.println((status & 0x80) ? "ON" : "OFF");

  } else if (message.id == BMS_ID_ERR) {
    // Handle error messages if needed
    Serial.println("Error message received");
  } else if (message.id >= CELL_TEMPS_BASE_ID && message.id < (CELL_TEMPS_BASE_ID + TOTAL_BOARDS)) {
    // Handles new teensy_cells temperature messages
    uint8_t board_id_from_can_id = message.id - CELL_TEMPS_BASE_ID;

    if (message.len == 4) {  // Expected length: BOARD_ID, min, max, avg
      uint8_t board_id_from_payload = message.buf[0];

      if (board_id_from_can_id == board_id_from_payload) {  // Sanity check
        if (board_id_from_can_id < TOTAL_BOARDS) {
          param.cell_board_temps[board_id_from_can_id].min_temp =
              static_cast<int8_t>(message.buf[1]);
          param.cell_board_temps[board_id_from_can_id].max_temp =
              static_cast<int8_t>(message.buf[2]);
          param.cell_board_temps[board_id_from_can_id].avg_temp =
              static_cast<int8_t>(message.buf[3]);
          param.cell_board_temps[board_id_from_can_id].has_data = true;
          param.cell_board_temps[board_id_from_can_id].last_update_ms = millis();
        }
      }
    }
  }
  else if (message.id == BMS_DUMP_ROW_0) {
    uint16_t buf16[8];
    for (int i = 0; i < 8; ++i) {
      buf16[i] = static_cast<uint16_t>(message.buf[i]);
    }
    displaySPI.transfer16(buf16, 8, WIDGET_BMS_DUMP_0, millis() & 0xFFFF);
  }
  else if (message.id == BMS_DUMP_ROW_1) {
    uint16_t buf16[8];
    for (int i = 0; i < 8; ++i) {
      buf16[i] = static_cast<uint16_t>(message.buf[i]);
    }
    displaySPI.transfer16(buf16, 8, WIDGET_BMS_DUMP_1, millis() & 0xFFFF);
  }  else if (message.id == BMS_DUMP_ROW_2) {
    uint16_t buf16[8];
    for (int i = 0; i < 8; ++i) {
      buf16[i] = static_cast<uint16_t>(message.buf[i]);
    }
    displaySPI.transfer16(buf16, 8, WIDGET_BMS_DUMP_2, millis() & 0xFFFF);
  }
  else if (message.id == BMS_THERMISTOR_ID) {
    const auto min_temp = static_cast<uint16_t>(message.buf[1]);
    const auto max_temp = static_cast<uint16_t>(message.buf[2]);

    displaySPI.transfer16(&min_temp, 1, WIDGET_CELLS_MIN, millis() & 0xFFFF);
    displaySPI.transfer16(&max_temp, 1, WIDGET_CELLS_MAX, millis() & 0xFFFF);

    uint16_t buf16[8];
    for (int i = 0; i < 8; ++i) {
      buf16[i] = static_cast<uint16_t>(message.buf[i]);
    }
    displaySPI.transfer16(buf16, 8, WIDGET_BMS_DUMP_3, millis() & 0xFFFF);
  }
}

void charger_machine() {
  switch (charger_status) {
    case Status::IDLE: {
      Serial.println("IDLE!");
      print_value("Shutdown status: ", shutdown_status);
      print_value("CH enable pin: ", ch_enable_pin);
      if (shutdown_status == 0 && ! ch_enable_pin) {
        charger_status = Status::CHARGING;
        constexpr uint16_t buf[] = {0x0001};
        displaySPI.transfer16(buf, 1, WIDGET_CH_STATUS, millis() & 0xFFFF);
      }
      break;
    }
    case Status::CHARGING: {
      Serial.println("CHARGING!");
      if (shutdown_status) {
        charger_status = Status::SHUTDOWN;
        constexpr uint16_t buf[] = {0x0002};
        displaySPI.transfer16(buf, 1, WIDGET_CH_STATUS, millis() & 0xFFFF);
      }
      break;
    }
    case Status::SHUTDOWN:
      Serial.println("SHUTDOWN!");
      break;

    default: {
      Serial.println("invalid charger state");
      break;
    }
  }
}

void read_inputs() {
  static bool last_sdc_status = false;
  shutdown_status = digitalRead(SHUTDOWN_PIN);
  ch_enable_pin = digitalRead(CH_ENABLE_PIN);

  sdc_status_pin = digitalRead(SDC_BUTTON_PIN);
  // Serial.print("SDC: ");
  // Serial.println(sdc_status_pin ? "ON" : "OFF");
  if (sdc_status_pin != last_sdc_status) {
    last_sdc_status = sdc_status_pin;
    const uint16_t buf[] = {sdc_status_pin};
    displaySPI.transfer16(buf, 1, WIDGET_SDC_BUTTON, millis() & 0xFFFF);
  }

  display_button.update();
  display_button_pressed = display_button.rose();
}

void power_on_module(const bool OnOff) {
  CAN_message_t powerMsg;

  powerMsg.id = CHARGER_ID;
  powerMsg.flags.extended = 1;
  powerMsg.len = 8;
  powerMsg.buf[0] = 0x10;
  powerMsg.buf[1] = 0x04;
  powerMsg.buf[2] = 0x00;
  powerMsg.buf[3] = 0x00;
  powerMsg.buf[4] = 0x00;
  powerMsg.buf[5] = 0x00;
  powerMsg.buf[6] = 0x00;
  powerMsg.buf[7] = 0x00;

  if (!OnOff) {
    powerMsg.buf[7] = 0x01;
  };  // turn off command

  can1.write(powerMsg);  // send message
}

void set_voltage(uint32_t voltage) {
  CAN_message_t voltageMsg;

  voltageMsg.id = CHARGER_ID;
  voltageMsg.flags.extended = 1;
  voltageMsg.len = 8;
  voltageMsg.buf[0] = 0x10;
  voltageMsg.buf[1] = 0x02;
  voltageMsg.buf[2] = 0x00;
  voltageMsg.buf[3] = 0x00;
  voltageMsg.buf[4] = voltage >> 24 & 0xff;
  voltageMsg.buf[5] = voltage >> 16 & 0xff;
  voltageMsg.buf[6] = voltage >> 8 & 0xff;
  voltageMsg.buf[7] = voltage & 0xff;

  can1.write(voltageMsg);  // send message
}

void set_current(const uint32_t current) {
  CAN_message_t current_msg;

  current_msg.id = CHARGER_ID;
  current_msg.flags.extended = 1;
  current_msg.len = 8;
  current_msg.buf[0] = 0x10;
  current_msg.buf[1] = 0x03;
  current_msg.buf[2] = 0x00;
  current_msg.buf[3] = 0x00;
  current_msg.buf[4] = current >> 24 & 0xff;
  current_msg.buf[5] = current >> 16 & 0xff;
  current_msg.buf[6] = current >> 8 & 0xff;
  current_msg.buf[7] = current & 0xff;

  can1.write(current_msg);  // send message
}

void set_low() {
  CAN_message_t low_msg;

  low_msg.id = CHARGER_ID;
  low_msg.flags.extended = 1;
  low_msg.len = 8;
  low_msg.buf[0] = 0x10;
  low_msg.buf[1] = 0x5f;
  low_msg.buf[2] = 0x00;
  low_msg.buf[3] = 0x00;
  low_msg.buf[4] = 0x00;
  low_msg.buf[5] = 0x00;
  low_msg.buf[6] = 0x00;
  low_msg.buf[7] = 0x00;

  can1.write(low_msg);  // send message
}

void read_current() {
  CAN_message_t request_msg;
  request_msg.id = REQUEST_CURRENT_ID;
  request_msg.len = 8;
  request_msg.buf[0] = 0x12;  // Byte0: GroupAddress and MessageType
  request_msg.buf[1] = 0x01;  // Byte1: CommandType (Read current)
  request_msg.buf[2] = 0x00;  // Byte2: Reserved
  request_msg.buf[3] = 0x00;  // Byte3: Reserved
  request_msg.buf[4] = 0x00;  // Byte4: Reserved
  request_msg.buf[5] = 0x00;  // Byte5: Reserved
  request_msg.buf[6] = 0x00;  // Byte6: Reserved
  request_msg.buf[7] = 0x00;  // Byte7: Reserved

  can1.write(request_msg);
}
void update_charger(Status charger_status) {
  set_current(param.allowed_current);
  set_voltage(MAX_VOLTAGE);
  power_on_module(charger_status == Status::CHARGING);
}
void print_temps() {
  Serial.println("--- Teensy Cell Temperatures ---");
  for (int i = 0; i < TOTAL_BOARDS; i++) {
    if (param.cell_board_temps[i].has_data) {
      Serial.printf("Board %d: Min=%d C, Max=%d C, Avg=%d C (Last update: %lu ms ago)\n", i,
                    param.cell_board_temps[i].min_temp, param.cell_board_temps[i].max_temp,
                    param.cell_board_temps[i].avg_temp,
                    millis() - param.cell_board_temps[i].last_update_ms);
    } else {
      Serial.printf("Board %d: No data received yet.\n", i);
    }
  }
}

void print_all_board_temps() {
  // Temperature thresholds
  const int TEMP_WARNING = 45;   // Celsius
  const int TEMP_CRITICAL = 55;  // Celsius

  Serial.println("\n--- TEMPERATURE BOARD DATA ---");
  Serial.println("Board | Min  | Max  | Avg  | Updated");
  Serial.println("------+------+------+------+--------");

  bool any_data = false;
  unsigned long current_time = millis();

  for (int i = 0; i < TOTAL_BOARDS; i++) {
    if (param.cell_board_temps[i].has_data) {
      any_data = true;

      // Calculate time since last update
      unsigned long time_since_update = current_time - param.cell_board_temps[i].last_update_ms;

      // Format the output with padding for alignment
      Serial.printf("%4d  | ", i);

      // Print min temp with warning indicator
      if (param.cell_board_temps[i].min_temp >= TEMP_CRITICAL)
        Serial.printf("%3d! | ", param.cell_board_temps[i].min_temp);
      else if (param.cell_board_temps[i].min_temp >= TEMP_WARNING)
        Serial.printf("%3d* | ", param.cell_board_temps[i].min_temp);
      else
        Serial.printf("%3d  | ", param.cell_board_temps[i].min_temp);

      // Print max temp with warning indicator
      if (param.cell_board_temps[i].max_temp >= TEMP_CRITICAL)
        Serial.printf("%3d! | ", param.cell_board_temps[i].max_temp);
      else if (param.cell_board_temps[i].max_temp >= TEMP_WARNING)
        Serial.printf("%3d* | ", param.cell_board_temps[i].max_temp);
      else
        Serial.printf("%3d  | ", param.cell_board_temps[i].max_temp);

      // Print avg temp with warning indicator
      if (param.cell_board_temps[i].avg_temp >= TEMP_CRITICAL)
        Serial.printf("%3d! | ", param.cell_board_temps[i].avg_temp);
      else if (param.cell_board_temps[i].avg_temp >= TEMP_WARNING)
        Serial.printf("%3d* | ", param.cell_board_temps[i].avg_temp);
      else
        Serial.printf("%3d  | ", param.cell_board_temps[i].avg_temp);

      // Print time since last update
      if (time_since_update < 5000) {
        Serial.printf("%lus\n", time_since_update / 1000);
      } else {
        Serial.printf("%lus!\n", time_since_update / 1000);
      }
    }
  }

  if (!any_data) {
    Serial.println("No temperature data available");
  }

  Serial.println("* = Warning temperature");
  Serial.println("! = Critical temperature");
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115'200);
  delay(2000);  // Allow time for Serial Monitor to open
  Serial.println("startup");
  Serial.println("startup");
  Serial.println("startup");
  Serial.println("startup");
  Serial.println("startup");
  Serial.println("startup");
  Serial.println("startup");
  Serial.println("startup");
  Serial.println("startup");

  pinMode(CH_ENABLE_PIN, INPUT);
  pinMode(SHUTDOWN_PIN, INPUT);
  pinMode(DISPLAY_BUTTON_PIN, INPUT);
  pinMode(SDC_BUTTON_PIN, INPUT);  // todo display
  display_button.attach(DISPLAY_BUTTON_PIN, INPUT);
  display_button.interval(10);  // 50ms debounce time

  displaySPI.begin();

  can1.begin();
  can1.setBaudRate(125'000);
  can1.enableFIFO();
  can1.enableFIFOInterrupt();

  can1.setFIFOFilter(REJECT_ALL);

  if (!can1.setFIFOFilter(0, CHARGER_ID, STD)) {
    Serial.println("Failed to set FIFO filter to CHARGER_ID");
  }
  if (!can1.setFIFOFilter(1, BMS_ID_CCL, STD)) {
    Serial.println("Failed to set FIFO filter to BMS_ID_CCL");
  }
  if (can1.setFIFOFilter(2, BMS_ID_ERR, STD)) {
    Serial.println("Failed to set FIFO filter to BMS_ID_ERR");
  }
  if (!can1.setFIFOFilter(3, BMS_THERMISTOR_ID, EXT)) {  // Extended frame
    Serial.println("Failed to set FIFO filter to BMS_THERMISTOR_ID");
  }
  if (!can1.setFIFOFilter(4, BMS_DUMP_ROW_0, EXT)) {
    Serial.println("Failed to set FIFO filter to BMS_DUMP_ROW_0");
  }
  if (!can1.setFIFOFilter(5, BMS_DUMP_ROW_1, EXT)) {
    Serial.println("Failed to set FIFO filter to BMS_DUMP_ROW_1");
  }
  if (!can1.setFIFOFilter(6, BMS_DUMP_ROW_2, EXT)) {
    Serial.println("Failed to set FIFO filter to BMS_DUMP_ROW_2");
  }

  uint8_t filter_idx_start = 7;
  for (int i = 0; i < TOTAL_BOARDS; ++i) {
    if (!can1.setFIFOFilter(filter_idx_start + i, CELL_TEMPS_BASE_ID + i, STD)) {
      Serial.println("Warning: Not enough FIFO filters for all teensy_cell boards.");
    }
  }

  can1.onReceive(can_snifflas);

  for (int i = 0; i < TOTAL_BOARDS; ++i) {
    param.cell_board_temps[i].has_data = false;
    param.cell_board_temps[i].last_update_ms = 0;
  }

  param.set_voltage = MAX_VOLTAGE;

  can1.write(HC_msg);  // send message

  delay(100);
  can1.write(HC_msg);  // send message

  delay(100);
  can1.write(HC_msg);  // send message

  delay(100);
  can1.write(HC_msg);  // send message

  delay(100);
  can1.write(HC_msg);  // send message

  delay(100);
  can1.write(HC_msg);  // send message

  delay(100);
  constexpr uint16_t buf[] = {0x0000};
  displaySPI.transfer16(buf, 1, WIDGET_CH_STATUS, millis() & 0xFFFF);
  // DBUG_PRINT_VAR(widgetID);
}

void loop() {

  if (step < 400) {
    return;
  }
  step = 0;

  can1.write(HC_msg);  // send message

  read_inputs();

  charger_machine();

  param.allowed_current = /* (param.ccl < SET_CURRENT) ? param.ccl :  */ SET_CURRENT;

  update_charger(charger_status);

  static uint16_t data[1];  // Define a static array to hold the values
  static int form_num = 1;

  if (display_button_pressed) {
    Serial.println("button pressed");
    form_num = (form_num == 1) ? 2 : 1;  // toggle 1 and 2
    data[0] = form_num;
    displaySPI.transfer16(data, 1, 0x9999, millis() & 0xFFFF);
    // DBUG_PRINT_VAR(widgetID);
    display_button_pressed = false;
  }
}