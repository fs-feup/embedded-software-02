#include <Arduino.h>
#include <FlexCAN_T4.h>

#include "../../CAN_IDs.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

CAN_message_t clearErrors = {.id = BAMO_COMMAND_ID, .len = 3, .buf = {0x8E, 0x44, 0x4D}};
CAN_message_t enableTransmission = {.id = BAMO_COMMAND_ID, .len = 3, .buf = {0x3D, 0xE8, 0x00}};
CAN_message_t checkBTBStatus = {.id = BAMO_COMMAND_ID, .len = 3, .buf = {0x3D, 0xE2, 0x00}};
CAN_message_t removeDisable = {.id = BAMO_COMMAND_ID, .len = 3, .buf = {0x51, 0x00, 0x00}};
CAN_message_t rampAccRequest = {.id = BAMO_COMMAND_ID, .len = 3, .buf = {0x35, 0xF4, 0x01}};
CAN_message_t rampDecRequest = {.id = BAMO_COMMAND_ID, .len = 3, .buf = {0xED, 0xE8, 0x03}};
CAN_message_t speedRequestFirst = {.id = BAMO_COMMAND_ID, .len = 3, .buf = {0x31, 0xD4, 0x03}};
CAN_message_t actualSpeedRequest = {.id = BAMO_COMMAND_ID, .len = 3, .buf = {0x3D, 0x30, 0x64}};
CAN_message_t speedRequestZero = {.id = BAMO_COMMAND_ID, .len = 3, .buf = {0x31, 0x00, 0x00}};

CAN_message_t torqueRequestFirst = {.id = BAMO_COMMAND_ID, .len = 3, .buf = {0x90, 0xFC, 0x3F}};
CAN_message_t actualTorqueRequest = {.id = BAMO_COMMAND_ID, .len = 3, .buf = {0x3D, 0xA0, 0x64}};
CAN_message_t torqueRequestZero = {.id = BAMO_COMMAND_ID, .len = 3, .buf = {0x90, 0x00, 0x00}};


CAN_message_t rpmRequest;
CAN_message_t speedRequest;
CAN_message_t currentMOTOR;
CAN_message_t tempMOTOR;
CAN_message_t tempBAMO;
CAN_message_t torque_motor;
CAN_message_t VoltageMotor;
CAN_message_t battery_voltage;

CAN_message_t disable = {.id = BAMO_COMMAND_ID, .len = 3, .buf = {0x51, 0x04, 0x00}};
CAN_message_t statusRequest = {.id = BAMO_COMMAND_ID, .len = 3, .buf = {0x3D, 0x40, 0x00}};
CAN_message_t DCVoltageRequest = {.id = BAMO_COMMAND_ID, .len = 3, .buf = {0x3D, 0xEB, 0x64}};

enum InitState {
  CLEAR_ERRORS,
  ENABLE_TRANSMISSION,
  CHECK_BTB,
  DISABLE,
  REMOVE_DISABLE,
  ACC_RAMP,
  DEC_RAMP,
  CONTROL_VALUE_INIT,
  ACTUAL_CONTROL_REQUEST,
  INITIALIZED,
  END,
  ERROR
};

InitState currentState = CLEAR_ERRORS;
unsigned long stateStartTime = 0;           // Tracks when the current state began
unsigned long lastActionTime = 0;           // Tracks the last command send time
unsigned long lastTorqueTime = 0;           // Tracks the last torque command time
const unsigned long actionInterval = 100;   // 10 ms interval for sending commands
const unsigned long timeout = 2000;         // 300 ms timeout for responses
const unsigned long torqueInterval = 1000;  // 1000 ms (1 second) interval for torque commands
bool commandSent = false;                   // Tracks if a one-time command has been sent
bool transmissionEnabled = false;           // Tracks if the transmission is enabled
bool BTBReady = false;                      // Tracks if the BTB is ready
unsigned long currentTime = 0;              // Tracks the current time
int userControl = 1000;    // Default torque value (non-zero)
String inputBuffer = "";  // Buffer to store incoming characters
bool newInput = false;    // Flag to indicate new input is available

void request_dataLOG_messages() {
  rpmRequest.id = BAMO_COMMAND_ID;
  rpmRequest.len = 3;
  rpmRequest.buf[0] = 0x3D;
  rpmRequest.buf[1] = 0xCE;
  rpmRequest.buf[2] = 0x0A;
  can1.write(rpmRequest);

  speedRequest.id = BAMO_COMMAND_ID;
  speedRequest.len = 3;
  speedRequest.buf[0] = 0x3D;
  speedRequest.buf[1] = 0x30;
  speedRequest.buf[2] = 0x0A;
  can1.write(speedRequest);

  currentMOTOR.id = BAMO_COMMAND_ID;
  currentMOTOR.len = 3;
  currentMOTOR.buf[0] = 0x3D;
  currentMOTOR.buf[1] = 0x5f;
  currentMOTOR.buf[2] = 0x0A;
  can1.write(currentMOTOR);

  tempMOTOR.id = BAMO_COMMAND_ID;
  tempMOTOR.len = 3;
  tempMOTOR.buf[0] = 0x3D;
  tempMOTOR.buf[1] = 0x49;
  tempMOTOR.buf[2] = 0x0A;
  can1.write(tempMOTOR);

  tempBAMO.id = BAMO_COMMAND_ID;
  tempBAMO.len = 3;
  tempBAMO.buf[0] = 0x3D;
  tempBAMO.buf[1] = 0x4A;
  tempBAMO.buf[2] = 0x0A;
  can1.write(tempBAMO);

  torque_motor.id = BAMO_COMMAND_ID;
  torque_motor.len = 3;
  torque_motor.buf[0] = 0x3D;
  torque_motor.buf[1] = 0xA0;
  torque_motor.buf[2] = 0x0A;
  can1.write(torque_motor);

  VoltageMotor.id = BAMO_COMMAND_ID;
  VoltageMotor.len = 3;
  VoltageMotor.buf[0] = 0x3D;
  VoltageMotor.buf[1] = 0x8A;
  VoltageMotor.buf[2] = 0x0A;
  can1.write(VoltageMotor);

  battery_voltage.id = BAMO_COMMAND_ID;
  battery_voltage.len = 3;
  battery_voltage.buf[0] = 0x3D;
  battery_voltage.buf[1] = 0xeb;
  battery_voltage.buf[2] = 0x0A;
  can1.write(battery_voltage);
}

void bamocar_callback(const uint8_t* msg_data) {
  // Serial.println("Received BAMOCAR response");

  Serial.print("Data: ");
  for (int i = 0; i < 8; i++) {                 // CAN messages can be up to 8 bytes
    if (msg_data[i] < 0x10) Serial.print("0");  // Add leading zero for values < 0x10
    Serial.print(msg_data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  switch (msg_data[0]) {
    case BTB_READY_0:
      if (msg_data[1] == BTB_READY_1 && msg_data[2] == BTB_READY_2 && msg_data[3] == BTB_READY_3) {
        BTBReady = true;
        Serial.println("BTB ready");
      }
      break;
    case ENABLE_0:
      if (msg_data[1] == ENABLE_1 && msg_data[2] == ENABLE_2 && msg_data[3] == ENABLE_3) {
        transmissionEnabled = true;
        Serial.println("Transmission enabled");
      }
      break;
    case 0x30:
      Serial.print("Motor speed: ");
      Serial.println((msg_data[1]) | msg_data[2] << 8 | msg_data[3] << 16);
      break;
    case 0xA0:
      Serial.print("Motor torque: ");
      Serial.println((msg_data[1]) | msg_data[2] << 8 | msg_data[3] << 16);
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
  can1.setBaudRate(500'000);  // Matches Unitek default 500 kBaud
  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  can1.setFIFOFilter(REJECT_ALL);
  can1.setFIFOFilter(0, BAMO_RESPONSE_ID, STD);  // Filter for BAMO_RESPONSE_ID (0x181)
  can1.onReceive(can_snifflas);
}

void sendTorque(int torqueValue) {
  CAN_message_t torque_message;
  torque_message.id = BAMO_COMMAND_ID;                // 0x201 for sending to BAMOCAR
  torque_message.len = 3;                             // REGID + 2 bytes for 16-bit torque value
  torque_message.buf[0] = 0x90;                       // REGID for TORQUE_SETPOINT
  torque_message.buf[1] = torqueValue & 0xFF;         // Lower byte
  torque_message.buf[2] = (torqueValue >> 8) & 0xFF;  // Upper byte

  can1.write(torque_message);
}

void sendSpeed(int speedValue) {
  CAN_message_t speed_message;
  speed_message.id = BAMO_COMMAND_ID;                // 0x201 for sending to BAMOCAR
  speed_message.len = 3;                             // REGID + 2 bytes for 16-bit torque value
  speed_message.buf[0] = 0x31;                       // REGID for TORQUE_SETPOINT
  speed_message.buf[1] = speedValue & 0xFF;          // Lower byte
  speed_message.buf[2] = (speedValue >> 8) & 0xFF;   // Upper byte

  can1.write(speed_message);
}

void sendposition(int positionValue) {
  CAN_message_t position_message;
  position_message.id = BAMO_COMMAND_ID;                // 0x201 for sending to BAMOCAR
  position_message.len = 6;                             // REGID + 2 bytes for 16-bit torque value
  position_message.buf[0] = 0x6E;                       // REGID for TORQUE_SETPOINT
  position_message.buf[1] = positionValue & 0xFF;          // Lower byte
  position_message.buf[2] = (positionValue >> 8) & 0xFF;   // Upper byte
  position_message.buf[3] = (positionValue >> 16) & 0xFF;   // Upper byte
  position_message.buf[4] = (positionValue >> 24) & 0xFF;   // Upper byte

  can1.write(position_message);
}

int checkSerialInput() {
  while (Serial.available() > 0) {
    char incomingChar = Serial.read();

    // Process on newline (Enter key)
    if (incomingChar == '\n' || incomingChar == '\r') {
      if(inputBuffer == "exit") {
        Serial.println("Exiting...");
        return 1;
      }
      if (inputBuffer.length() > 0) {
        // Convert string to integer
        int newTorque = inputBuffer.toInt();

        if (newTorque != 0) {  // Ignore if input was just "0" or invalid
          userControl = newTorque;
          Serial.print("New torque value set: ");
          Serial.println(userControl);
        }

        // Clear the buffer for next input
        inputBuffer = "";
      }
    } else {
      inputBuffer += incomingChar;
    }
  }
  return 0;
}

void speedSM(){
  switch (currentState) {
    case CLEAR_ERRORS:
      if (!commandSent) {
        can1.write(clearErrors);
        commandSent = true;
        stateStartTime = currentTime;
      }
      if (currentTime - stateStartTime >= 100) {
        currentState = CHECK_BTB;
        Serial.println("Errors cleared, enabling transmission");
        commandSent = false;
        stateStartTime = currentTime;
        lastActionTime = currentTime;
      }
      break;

    case CHECK_BTB:
      if (currentTime - lastActionTime >= actionInterval) {
        can1.write(checkBTBStatus);
        lastActionTime = currentTime;
      }
      if (BTBReady) {
        currentState = DISABLE;
        commandSent = false;
      } else if (currentTime - stateStartTime >= timeout) {
        Serial.println("Timeout checking BTB");
        Serial.println("Error during initialization");
        currentState = ERROR;
      }
      break;

    case DISABLE:
      //can1.write(disable); /* TRY THIS - 1 (there's 3) */
      currentState = ENABLE_TRANSMISSION;
    case ENABLE_TRANSMISSION:
      if (currentTime - lastActionTime >= actionInterval) {
        Serial.println("Enabling transmission");
        can1.write(enableTransmission);
        lastActionTime = currentTime;
      }
      if (transmissionEnabled) {
        currentState = REMOVE_DISABLE;
        commandSent = false;
        stateStartTime = currentTime;
        lastActionTime = currentTime;
      } else if (currentTime - stateStartTime >= timeout) {
        Serial.println("Timeout enabling transmission");
        Serial.println("Error during initialization");
        currentState = ERROR;
      }
      break;

    case REMOVE_DISABLE:
      if (!commandSent) {
        can1.write(removeDisable);
        commandSent = true;
        currentState = ACC_RAMP;
        // request_dataLOG_messages();
      }
      break;

    case ACC_RAMP:
      can1.write(rampAccRequest);
      currentState = DEC_RAMP;
      break;

    case DEC_RAMP:
      can1.write(rampDecRequest);
      currentState = CONTROL_VALUE_INIT;
      break;
    
    case CONTROL_VALUE_INIT:
      can1.write(speedRequestFirst);
      currentState = ACTUAL_CONTROL_REQUEST ;
      break;
    case ACTUAL_CONTROL_REQUEST:
      can1.write(actualSpeedRequest);
      currentState = INITIALIZED;
      break;
    case INITIALIZED:
      // Send torque command every 1 second if BTBReady and transmissionEnabled
      if (checkSerialInput()) {
        can1.write(speedRequestZero);
        delay(1000);
        can1.write(disable);
        // exit program
        currentState = END;
        break;
      }
      if (currentTime - lastTorqueTime >= torqueInterval) {
        if (BTBReady && transmissionEnabled) {
          // sendTorque(userControl);
          sendSpeed(userControl);
          Serial.print("Torque command sent: ");
          Serial.println(userControl);
          lastTorqueTime = currentTime;
        }
      }
      break;
    case END:
      Serial.println("Program ended");
      break;
    case ERROR:
      break;
  }
}

void torqueSM(){
  switch (currentState) {
    case CLEAR_ERRORS:
      if (!commandSent) {
        can1.write(clearErrors);
        commandSent = true;
        stateStartTime = currentTime;
      }
      if (currentTime - stateStartTime >= 100) {
        currentState = CHECK_BTB;
        Serial.println("Errors cleared, enabling transmission");
        commandSent = false;
        stateStartTime = currentTime;
        lastActionTime = currentTime;
      }
      break;

    case CHECK_BTB:
      if (currentTime - lastActionTime >= actionInterval) {
        can1.write(checkBTBStatus);
        lastActionTime = currentTime;
      }
      if (BTBReady) {
        currentState = DISABLE;
        commandSent = false;
      } else if (currentTime - stateStartTime >= timeout) {
        Serial.println("Timeout checking BTB");
        Serial.println("Error during initialization");
        currentState = ERROR;
      }
      break;

    case DISABLE:
      //can1.write(disable); /* TRY THIS TOO - 2  */
      currentState = ENABLE_TRANSMISSION;
    case ENABLE_TRANSMISSION:
      if (currentTime - lastActionTime >= actionInterval) {
        Serial.println("Enabling transmission");
        can1.write(enableTransmission);
        lastActionTime = currentTime;
      }
      if (transmissionEnabled) {
        currentState = REMOVE_DISABLE;
        commandSent = false;
        stateStartTime = currentTime;
        lastActionTime = currentTime;
      } else if (currentTime - stateStartTime >= timeout) {
        Serial.println("Timeout enabling transmission");
        Serial.println("Error during initialization");
        currentState = ERROR;
      }
      break;

    case REMOVE_DISABLE:
      if (!commandSent) {
        can1.write(removeDisable);
        commandSent = true;
        currentState = ACC_RAMP;
        // request_dataLOG_messages();
      }
      break;

    case ACC_RAMP:
      can1.write(rampAccRequest);
      currentState = DEC_RAMP;
      break;

    case DEC_RAMP:
      can1.write(rampDecRequest);
      currentState = CONTROL_VALUE_INIT;
      break;
    
    case CONTROL_VALUE_INIT:
      can1.write(torqueRequestFirst);
      currentState = ACTUAL_CONTROL_REQUEST ;
      break;
    case ACTUAL_CONTROL_REQUEST:
      can1.write(actualTorqueRequest);
      currentState = INITIALIZED;
      break;
    case INITIALIZED:
      // Send torque command every 1 second if BTBReady and transmissionEnabled
      if (checkSerialInput()) {
        can1.write(torqueRequestZero);
        delay(1000);
        can1.write(disable);
        // exit program
        currentState = END;
        break;
      }
      if (currentTime - lastTorqueTime >= torqueInterval) {
        if (BTBReady && transmissionEnabled) {
          // sendTorque(userControl);
          sendSpeed(userControl);
          Serial.print("Torque command sent: ");
          Serial.println(userControl);
          lastTorqueTime = currentTime;
        }
      }
      break;
    case END:
      Serial.println("Program ended");
      break;
    case ERROR:
      break;
  }
}


void setup() {
  Serial.begin(115200);
  canSetup();

  Serial.println("Enter torque value and press Enter to change the torque command.");
  Serial.println("Default torque is 1000.");

  delay(10000);

  can1.write(disable);
  //can1.write(statusRequest);
  //can1.write(DCVoltageRequest);
}

void loop() {
  currentTime = millis();  // Get current time
  speedSM();
  //torqueSM();
  
}