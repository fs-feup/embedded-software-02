#include <Arduino.h>
#include <FlexCAN_T4.h>

#include "../../CAN_IDs.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

CAN_message_t clearErrors = {.id = BAMO_COMMAND_ID, .len = 3, .buf = {0x8E, 0x44, 0x4D}};
CAN_message_t enableTransmission = {.id = BAMO_COMMAND_ID, .len = 3, .buf = {0x3D, 0xE8, 0x00}};
CAN_message_t checkBTBStatus = {.id = BAMO_COMMAND_ID, .len = 3, .buf = {0x3D, 0xE2, 0x00}};
CAN_message_t removeDisable = {.id = BAMO_COMMAND_ID, .len = 3, .buf = {0x51, 0x00, 0x00}};

enum InitState {
  STATE_CLEAR_ERRORS,
  STATE_ENABLE_TRANSMISSION,
  STATE_CHECK_BTB,
  STATE_REMOVE_DISABLE,
  STATE_INITIALIZED,
  STATE_ERROR
};

InitState currentState = STATE_CLEAR_ERRORS;
unsigned long stateStartTime = 0;          // Tracks when the current state began
unsigned long lastActionTime = 0;          // Tracks the last command send time
unsigned long lastTorqueTime = 0;          // Tracks the last torque command time
const unsigned long actionInterval = 10;   // 10 ms interval for sending commands
const unsigned long timeout = 300;         // 100 ms timeout for responses
const unsigned long torqueInterval = 100;  // 100 ms interval for torque commands
bool commandSent = false;                  // Tracks if a one-time command has been sent
bool transmissionEnabled = false;          // Tracks if the transmission is enabled
bool BTBReady = false;                     // Tracks if the BTB is ready

bool check_sequence(const uint8_t* data, const std::array<uint8_t, 3>& expected) {
  return (data[1] == expected[0] && data[2] == expected[1] && data[3] == expected[2]);
}

void bamocar_callback(const uint8_t* msg_data) {
  switch (msg_data[0]) {
    case BTB_READY_0:
      BTBReady = check_sequence(msg_data, BTB_READY_SEQUENCE);
      break;

    case ENABLE_0:
      transmissionEnabled = check_sequence(msg_data, ENABLE_SEQUENCE);
      break;
    default:
      break;
  }
}

void can_snifflas(const CAN_message_t& msg) {
  switch (msg.id) {
    case BAMO_RESPONSE_ID:
      bamocar_callback(msg.buf);
      break;
    default:
      break;
  }
}

void canSetup() {
  can1.begin();
  can1.setBaudRate(500'000);
  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  can1.setFIFOFilter(REJECT_ALL);
  can1.setFIFOFilter(0, BAMO_RESPONSE_ID, STD);
  can1.onReceive(can_snifflas);
}

void sendTorque(int torque) {
  CAN_message_t torque_message;
  torque_message.id = BAMO_COMMAND_ID;
  torque_message.len = 3;
  torque_message.buf[0] = 0x90;

  uint8_t torque_byte1 = (torque >> 8) & 0xFF;  // MSB
  uint8_t torque_byte2 = torque & 0xFF;         // LSB

  torque_message.buf[1] = torque_byte2;
  torque_message.buf[2] = torque_byte1;

  can1.write(torque_message);
}


void setup() {
  Serial.begin(115200);
  canSetup();
}

void loop() {
  unsigned long currentTime = millis();  // Get current time

  switch (currentState) {
    case STATE_CLEAR_ERRORS:

      if (!commandSent) {
        can1.write(clearErrors);
        commandSent = true;
        stateStartTime = currentTime;
      }
      if (currentTime - stateStartTime >= 100) {
        currentState = STATE_ENABLE_TRANSMISSION;
        commandSent = false;
        stateStartTime = currentTime;
        lastActionTime = currentTime;
      }
      break;

    case STATE_ENABLE_TRANSMISSION:
      // Periodically send enableTransmission until response or timeout
      if (currentTime - lastActionTime >= actionInterval) {
        can1.write(enableTransmission);
        lastActionTime = currentTime;
      }
      if (transmissionEnabled) {
        currentState = STATE_CHECK_BTB;
        commandSent = false;
        stateStartTime = currentTime;
        lastActionTime = currentTime;
      } else if (currentTime - stateStartTime >= timeout) {
        Serial.println("Timeout enabling transmission");
        currentState = STATE_ERROR;
      }
      break;

    case STATE_CHECK_BTB:
      if (currentTime - lastActionTime >= actionInterval) {
        can1.write(checkBTBStatus);
        lastActionTime = currentTime;
      }
      if (BTBReady) {
        currentState = STATE_REMOVE_DISABLE;
        commandSent = false;
      } else if (currentTime - stateStartTime >= timeout) {
        Serial.println("Timeout checking BTB");
        currentState = STATE_ERROR;
      }
      break;

    case STATE_REMOVE_DISABLE:
      if (!commandSent) {
        can1.write(removeDisable);
        commandSent = true;
        currentState = STATE_INITIALIZED;
      }
      break;

    case STATE_INITIALIZED:
      // torque commands periodically
      if (currentTime - lastTorqueTime >= torqueInterval) {
        if (BTBReady && transmissionEnabled) {
          int torqueValue = 1000;
          sendTorque(torqueValue);
          lastTorqueTime = currentTime;
        }
      }
      break;

    case STATE_ERROR:
      Serial.println("Error during initialization");
      break;
  }
}
