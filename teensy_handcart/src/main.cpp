#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <elapsedMillis.h>

#include "constants.hpp"
#include "structs.hpp"
#include "utils.hpp"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

elapsedMillis step;

PARAMETERS param;

bool ch_safety_pin = 1;    // This was CH safety pin status, todo: check it exists
bool shutdown_status = 0;  // Charging shutdown status, 1 for shutdown
bool latching_status = 0;  // Latching error status, 0 means error (open)

Status charger_status;       // current state machine status
Status next_charger_status;  // next state machine status

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
      print_value("Current voltage= ", param.current_voltage);
      break;
    }

    case CURRENT_CURRENT_RESPONSE: {
      extract_value(param.current_current, message.buf);
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
  if (message.id == CHARGER_ID) {
    parse_charger_message(message);
  } else if (message.id == BMS_ID_CCL) {
    param.ccl = message.buf[0] * 1000; // Assuming conversion is correct
  } else if (message.id >= CELL_TEMPS_BASE_ID && message.id < (CELL_TEMPS_BASE_ID + TOTAL_BOARDS)) {
    // Handles new teensy_cells temperature messages
    uint8_t board_id_from_can_id = message.id - CELL_TEMPS_BASE_ID;

    if (message.len == 4) { // Expected length: BOARD_ID, min, max, avg
      uint8_t board_id_from_payload = message.buf[0];

      if (board_id_from_can_id == board_id_from_payload) { // Sanity check
        if (board_id_from_can_id < TOTAL_CELL_BOARDS) {
          param.cell_board_temps[board_id_from_can_id].min_temp = static_cast<int8_t>(message.buf[1]);
          param.cell_board_temps[board_id_from_can_id].max_temp = static_cast<int8_t>(message.buf[2]);
          param.cell_board_temps[board_id_from_can_id].avg_temp = static_cast<int8_t>(message.buf[3]);
          param.cell_board_temps[board_id_from_can_id].has_data = true;
          param.cell_board_temps[board_id_from_can_id].last_update_ms = millis();
        }
      } else {
        // Optional: Log board ID mismatch
        // Serial.printf("Cell Temp Msg: Board ID mismatch! CAN_ID implies %d, Payload says %d\n", board_id_from_can_id, board_id_from_payload);
      }
    } else {
      // Optional: Log incorrect message length
      // Serial.printf("Cell Temp Msg: Incorrect length %d for board %d (ID 0x%X)\n", message.len, board_id_from_can_id, message.id);
    }
  }
}

void charger_machine() {
  next_charger_status = charger_status;
  switch (charger_status) {
    case Status::IDLE: {
      if (shutdown_status) {
        next_charger_status = Status::SHUTDOWN;
      } else if (ch_safety_pin == 1 && latching_status == 1) {
        next_charger_status = Status::CHARGING;
      }
      break;
    }
    case Status::CHARGING: {
      if (shutdown_status) {
        next_charger_status = Status::SHUTDOWN;
      } else if (ch_safety_pin == 0 || latching_status == 0) {
        next_charger_status = Status::IDLE;
      }
      break;
    }
    case Status::SHUTDOWN:
      break;

    default: {
      Serial.println("invalid charger state");
      break;
    }
  }
}

void read_inputs() {
  shutdown_status = digitalRead(SHUTDOWN_PIN);
  latching_status = !shutdown_status;
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
  for (int i = 0; i < TOTAL_CELL_BOARDS; i++) {
    if (param.cell_board_temps[i].has_data) {
      Serial.printf("Board %d: Min=%d C, Max=%d C, Avg=%d C (Last update: %lu ms ago)\n",
                    i,
                    param.cell_board_temps[i].min_temp,
                    param.cell_board_temps[i].max_temp,
                    param.cell_board_temps[i].avg_temp,
                    millis() - param.cell_board_temps[i].last_update_ms);
    } else {
      Serial.printf("Board %d: No data received yet.\n", i);
    }
  }
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115'200);
  Serial.println("startup");

  pinMode(CH_SAFETY_PIN, INPUT);
  pinMode(SHUTDOWN_PIN, INPUT);
  pinMode(LATCHING_ERROR_PIN, INPUT);

  can1.begin();
  can1.setBaudRate(125'000);
  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  can1.setFIFOFilter(REJECT_ALL);
  can1.setFIFOFilter(0, CHARGER_ID, STD);
  can1.setFIFOFilter(1, BMS_ID_CCL, STD);
  can1.setFIFOFilter(2, BMS_ID_ERR, STD);

  uint8_t filter_idx_start = 3; 
  for (int i = 0; i < TOTAL_CELL_BOARDS; ++i) {
    if (filter_idx_start + i < 8) { 
        can1.setFIFOFilter(filter_idx_start + i, CELL_TEMPS_BASE_ID + i, STD);
    } else {
        Serial.println("Warning: Not enough FIFO filters for all teensy_cell boards.");
        break;
    }
  }

  can1.onReceive(can_snifflas);

  for (int i = 0; i < TOTAL_CELL_BOARDS; ++i) {
    param.cell_board_temps[i].has_data = false;
    param.cell_board_temps[i].last_update_ms = 0;
  }

  param.set_voltage = MAX_VOLTAGE;
}

void loop() {
  if (step < 1000) {
    return;
  }

  step = 0;

  read_inputs();
  charger_machine();
  charger_status = next_charger_status;

  param.allowed_current = (param.ccl < MAX_CURRENT) ? param.ccl : MAX_CURRENT;

  print_temps();

  update_charger(charger_status);

  Serial.printf("ccl: %d\n", param.ccl);

  Serial.println("loop iteration");
  Serial.println("====================================");
  Serial.println();
}