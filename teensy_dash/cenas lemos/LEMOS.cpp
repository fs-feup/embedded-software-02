#include <Arduino.h>
#include <FlexCAN_T4.h>

#include "../../CAN_IDs.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

CAN_message_t imax_msg = {.id = BAMO_COMMAND_ID, .len = 3, .buf = {0xc4, 0x00, 0x00}};
CAN_message_t icon_msg = {.id = BAMO_COMMAND_ID, .len = 3, .buf = {0xc5, 0x00, 0x00}};
CAN_message_t rampsacc_msg = {.id = BAMO_COMMAND_ID, .len = 5, .buf = {0x35, 0x00, 0x00}};
CAN_message_t rampsdec_msg = {.id = BAMO_COMMAND_ID, .len = 5, .buf = {0xed, 0x00, 0x00}};

CAN_message_t imax_msg_request = {.id = BAMO_COMMAND_ID, .len = 3, .buf = {0x3D, 0xC4, 0xFE}};
CAN_message_t speed_msG = {.id = BAMO_COMMAND_ID, .len = 3, .buf = {0x34, 0x00, 0x00}};
CAN_message_t speed_msG_request = {.id = BAMO_COMMAND_ID, .len = 3, .buf = {0x3D, 0x34, 0xFE}};

CAN_message_t clearErrors = {.id = BAMO_COMMAND_ID, .len = 3, .buf = {0x8E, 0x44, 0x4D}};
CAN_message_t enableTransmission = {.id = BAMO_COMMAND_ID, .len = 3, .buf = {0x3D, 0xE8, 0x00}};
CAN_message_t checkBTBStatus = {.id = BAMO_COMMAND_ID, .len = 3, .buf = {0x3D, 0xE2, 0x00}};
CAN_message_t removeDisable = {.id = BAMO_COMMAND_ID, .len = 3, .buf = {0x51, 0x00, 0x00}};
CAN_message_t rampAccRequest = {.id = BAMO_COMMAND_ID, .len = 3, .buf = {0x35, 0xF4, 0x01}};
CAN_message_t rampDecRequest = {.id = BAMO_COMMAND_ID, .len = 3, .buf = {0xED, 0xE8, 0x03}};
CAN_message_t speedRequestFirst = {.id = BAMO_COMMAND_ID, .len = 3, .buf = {0x31, 0xD4, 0x03}};
CAN_message_t actualSpeedRequest = {
    .id = BAMO_COMMAND_ID, .len = 3, .buf = {0x3D, 0x30, 0xFE}};  // every 0xFE mseconds (254)
CAN_message_t speedRequestZero = {.id = BAMO_COMMAND_ID, .len = 3, .buf = {0x31, 0x00, 0x00}};

CAN_message_t torqueRequestFirst = {.id = BAMO_COMMAND_ID, .len = 3, .buf = {0x90, 0xFC, 0x3F}};
CAN_message_t actualTorqueRequest = {
    .id = BAMO_COMMAND_ID, .len = 3, .buf = {0x3D, 0xA0, 0xFE}};  // every 0xFE mseconds (254)

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
int userControl = 0;                        // Default torque value (non-zero)
String inputBuffer = "";                    // Buffer to store incoming characters
bool newInput = false;                      // Flag to indicate new input is available
uint16_t rawSpeed = 0;
float speedPercentage = 0.0f;

void request_dataLOG_messages() {
  rpmRequest.id = BAMO_COMMAND_ID;
  rpmRequest.len = 3;
  rpmRequest.buf[0] = 0x3D;
  rpmRequest.buf[1] = 0xCE;
  rpmRequest.buf[2] = 0x0A;
  // can1.write(rpmRequest);

  speedRequest.id = BAMO_COMMAND_ID;
  speedRequest.len = 3;
  speedRequest.buf[0] = 0x3D;
  speedRequest.buf[1] = 0x30;
  speedRequest.buf[2] = 0x0A;
  // can1.write(speedRequest);

  currentMOTOR.id = BAMO_COMMAND_ID;
  currentMOTOR.len = 3;
  currentMOTOR.buf[0] = 0x3D;
  currentMOTOR.buf[1] = 0x5f;
  currentMOTOR.buf[2] = 0x0A;
  // can1.write(currentMOTOR);

  tempMOTOR.id = BAMO_COMMAND_ID;
  tempMOTOR.len = 3;
  tempMOTOR.buf[0] = 0x3D;
  tempMOTOR.buf[1] = 0x49;
  tempMOTOR.buf[2] = 0x0A;
  // can1.write(tempMOTOR);

  tempBAMO.id = BAMO_COMMAND_ID;
  tempBAMO.len = 3;
  tempBAMO.buf[0] = 0x3D;
  tempBAMO.buf[1] = 0x4A;
  tempBAMO.buf[2] = 0x0A;
  // can1.write(tempBAMO);

  torque_motor.id = BAMO_COMMAND_ID;
  torque_motor.len = 3;
  torque_motor.buf[0] = 0x3D;
  torque_motor.buf[1] = 0xA0;
  torque_motor.buf[2] = 0x0A;
  // can1.write(torque_motor);

  VoltageMotor.id = BAMO_COMMAND_ID;
  VoltageMotor.len = 3;
  VoltageMotor.buf[0] = 0x3D;
  VoltageMotor.buf[1] = 0x8A;
  VoltageMotor.buf[2] = 0x0A;
  // can1.write(VoltageMotor);

  battery_voltage.id = BAMO_COMMAND_ID;
  battery_voltage.len = 3;
  battery_voltage.buf[0] = 0x3D;
  battery_voltage.buf[1] = 0xeb;
  battery_voltage.buf[2] = 0x0A;
  // can1.write(battery_voltage);
}

void bamocar_callback(const uint8_t* msg_data) {
  // Serial.println("Received BAMOCAR response");

  // Serial.print("Data: ");
  // for (int i = 0; i < 8; i++) {                 // CAN messages can be up to 8 bytes
  //   if (msg_data[i] < 0x10) Serial.print("0");  // Add leading zero for values < 0x10
  //   Serial.print(msg_data[i], HEX);
  //   Serial.print(" ");
  // }
  // Serial.println();

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
      Serial.print(msg_data[1] | (msg_data[2] << 8) | (msg_data[3] << 16));
      Serial.print(" OR ");
      Serial.println(msg_data[3] | (msg_data[2] << 8) | (msg_data[1] << 16));
      break;
    case 0xA0:
      // Serial.print("Motor torque: ");
      // Serial.print(msg_data[1] | (msg_data[2] << 8) | (msg_data[3] << 16));
      // Serial.print(" OR ");
      // Serial.println(msg_data[3] | (msg_data[2] << 8) | (msg_data[1] << 16));
    case 0x34: {
      Serial.print("SPEEDI: ");
      int a = msg_data[1] | (msg_data[2] << 8);
      int a1 = map(a, 0, 32767, 0, 100);

      Serial.println(a1);
      break;
    }
    case 0xC4: {
      Serial.print("~imax: ");
      int a = msg_data[1] | (msg_data[2] << 8);
      int a1 = map(a, 0, 8355, 0, 100);

      Serial.println(a1);
      break;
    }
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
  // Serial.print("Sending torque: ");
  // Serial.println(torqueValue);
  can1.write(torque_message);
}

void sendSpeed(int speedValue) {
  CAN_message_t speed_message;
  speed_message.id = BAMO_COMMAND_ID;               // 0x201 for sending to BAMOCAR
  speed_message.len = 3;                            // REGID + 2 bytes for 16-bit torque value
  speed_message.buf[0] = 0x31;                      // REGID for TORQUE_SETPOINT
  speed_message.buf[1] = speedValue & 0xFF;         // Lower byte
  speed_message.buf[2] = (speedValue >> 8) & 0xFF;  // Upper byte
  Serial.print("Sending speed: ");
  Serial.println(speedValue);
  can1.write(speed_message);
}

void sendposition(int positionValue) {
  CAN_message_t position_message;
  position_message.id = BAMO_COMMAND_ID;                  // 0x201 for sending to BAMOCAR
  position_message.len = 6;                               // REGID + 2 bytes for 16-bit torque value
  position_message.buf[0] = 0x6E;                         // REGID for TORQUE_SETPOINT
  position_message.buf[1] = positionValue & 0xFF;         // Lower byte
  position_message.buf[2] = (positionValue >> 8) & 0xFF;  // Upper byte
  position_message.buf[3] = (positionValue >> 16) & 0xFF;  // Upper byte
  position_message.buf[4] = (positionValue >> 24) & 0xFF;  // Upper byte

  can1.write(position_message);
}

int checkSerialInput() {
  while (Serial.available() > 0) {
    char incomingChar = Serial.read();

    // Process on newline (Enter key)
    if (incomingChar == '\n' || incomingChar == '\r') {
      if (inputBuffer == "exit") {
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

void torqueSM() {
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
        Serial.println("Checking BTB status");
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
      Serial.println("Disabling");
      can1.write(disable);
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
        Serial.println("Removing disable");
        can1.write(removeDisable);
        commandSent = true;
        currentState = ACC_RAMP;
        // request_dataLOG_messages();
      }
      break;

    case ACC_RAMP:
      Serial.print("Transmitting acceleration ramp: ");
      Serial.print(rampAccRequest.buf[1] | (rampAccRequest.buf[2] << 8));
      Serial.print("ms");
      can1.write(rampAccRequest);
      currentState = DEC_RAMP;
      break;

    case DEC_RAMP:
      Serial.print("Transmitting deceleration ramp: ");
      Serial.print(rampDecRequest.buf[1] | (rampDecRequest.buf[2] << 8));
      Serial.print("ms");
      can1.write(rampDecRequest);
      currentState = CONTROL_VALUE_INIT;
      break;

    case CONTROL_VALUE_INIT:
      rawSpeed = (torqueRequestFirst.buf[2] << 8) | torqueRequestFirst.buf[1];
      speedPercentage = (rawSpeed * 100.0f) / 32767.0f;
      Serial.print("Transmitting first torque request: Raw=");
      Serial.print(rawSpeed);
      Serial.print(" (");
      Serial.print(speedPercentage, 2);  // Print with 2 decimal places
      Serial.println("%)");

      sendTorque(0x03D4);
      currentState = ACTUAL_CONTROL_REQUEST;
      break;
    case ACTUAL_CONTROL_REQUEST:
      Serial.print("Requesting actual torque: ");
      Serial.print(actualTorqueRequest.buf[1] | (actualTorqueRequest.buf[2] << 8));
      // can1.write(actualTorqueRequest);
      // can1.write(speed_msG_request);
      // can1.write(imax_msg_request);
      currentState = INITIALIZED;
      break;
    case INITIALIZED:
      // Send torque command every 1 second if BTBReady and transmissionEnabled
      if (checkSerialInput()) {  // if user typed "exit" in serial monitor
        sendTorque(0);
        delay(1000);
        can1.write(disable);
        // exit program
        currentState = END;
        break;
      }
      if (currentTime - lastTorqueTime >= torqueInterval) {
        if (BTBReady && transmissionEnabled) {
          sendTorque(userControl);
          // Serial.print("Torque command sent: ");
          // Serial.println(userControl);
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
  pinMode(40, INPUT);
  can1.write(disable);
  // can1.write(statusRequest);
  // can1.write(DCVoltageRequest);
}
int imax = 0;
int speed = 0;
int icon = 0;
int macc = 0, mdec = 0;
int nacc = 0, ndec = 0;
String mode = "0";

int pos = 0;
int previous_pos = 0;

bool positionChanged = false;

void loop() {
  currentTime = millis();  // Get current time
  int pos = map(analogRead(40), 0, 1023, 1, 12);

  if (pos != previous_pos) {
    // Debounce the reading
    delay(50);  // Short delay for switch stabilization
    int new_reading = map(analogRead(40), 0, 1023, 1, 12);

    // Confirm the change is stable
    if (new_reading == pos) {
      positionChanged = true;
      // Serial.print("Position changed from ");
      // Serial.print(previous_pos);
      // Serial.print(" to ");
      //Serial.println(pos);
    }
  }

  if (positionChanged) {
    switch (pos) {
      case 1:
        /* MODE 0 */
        mode = "MODE 0";
        imax = 0;
        speed = 0;
        icon = 0;
        nacc = 100, macc = 10;
        ndec = 100, mdec = 10;
        break;
      case 2:
        /* MODE CAVALETES */
        mode = "CAVALETES";
        imax = 3;
        speed = 5;
        icon = 3;
        nacc = 1000, macc = 100;
        ndec = 1000, mdec = 100;
        break;
      case 3:
        /* MODE LIMITER */
        mode = "LIMITER";
        imax = 10;
        speed = 10;
        icon = 10;
        nacc = 1000, macc = 100;
        ndec = 1000, mdec = 100;
        break;
      case 4:
        /* MODE BRAKE TEST */
        mode = "BRAKE TEST";
        imax = 70;
        speed = 84;
        icon = 70;
        nacc = 200, macc = 20;
        ndec = 0, mdec = 0;
        break;
      case 5:
        /* MODE SKIDPAD */
        mode = "SKIDPAD";
        imax = 70;
        speed = 70;
        icon = 70;
        nacc = 500, macc = 50;
        ndec = 500, mdec = 50;
        break;
      case 6:
        /* MODE ENDURANCE */
        mode = "ENDURANCE";
        imax = 70;
        speed = 84;
        icon = 70;
        nacc = 200, macc = 20;
        ndec = 200, mdec = 20;
        break;
      case 7:
        /* MODE MAX ATTACK */
        mode = "MAX ATTACK";
        imax = 100;
        speed = 84;
        icon = 100;
        nacc = 100, macc = 10;
        ndec = 100, mdec = 10;
        break;
      case 8:
        mode = "NULL";
        imax = 40;
        speed = 40;
        icon = 40;
        nacc = 400, macc = 40;
        ndec = 400, mdec = 40;
        break;
      case 9:
        mode = "NULL";
        imax = 45;
        speed = 45;
        icon = 45;
        nacc = 450, macc = 45;
        ndec = 450, mdec = 45;
        break;
      case 10:
        mode = "NULL";
        imax = 50;
        speed = 50;
        icon = 50;
        nacc = 500, macc = 50;
        ndec = 500, mdec = 50;
        break;
      case 11:
        mode = "NULL";
        imax = 55;
        speed = 55;
        icon = 55;
        nacc = 550, macc = 55;
        ndec = 550, mdec = 55;
        break;
      case 12:
        mode = "NULL";
        imax = 60;
        speed = 60;
        icon = 60;
        nacc = 600, macc = 60;
        ndec = 600, mdec = 60;
        break;
      default:
        break;
    }

    positionChanged = false;

    delay(100);
    // update the imax and speed messages

    int imaxnew = map(imax, 0, 100, 0, 16369);
    int iconnew = map(icon, 0, 100, 0, 16369);
    int espeednew = map(speed, 0, 100, 0, 32767);

    imax_msg.buf[1] = imaxnew & 0xFF;            // Lower byte
    imax_msg.buf[2] = (imaxnew >> 8) & 0xFF;     // Upper byte
    speed_msG.buf[1] = espeednew & 0xFF;         // Lower byte
    speed_msG.buf[2] = (espeednew >> 8) & 0xFF;  // Upper byte
    icon_msg.buf[1] = iconnew & 0xFF;            // Lower byte
    icon_msg.buf[2] = (iconnew >> 8) & 0xFF;     // Upper byte

    rampsacc_msg.buf[1] = nacc & 0xFF;         // Lower byte
    rampsacc_msg.buf[2] = (nacc >> 8) & 0xFF;  // Upper byte
    rampsacc_msg.buf[3] = macc & 0xFF;         // Upper byte
    rampsacc_msg.buf[4] = (macc >> 8) & 0xFF;  // Upper byte

    rampsdec_msg.buf[1] = ndec & 0xFF;         // Lower byte
    rampsdec_msg.buf[2] = (ndec >> 8) & 0xFF;  // Upper byte
    rampsdec_msg.buf[3] = mdec & 0xFF;         // Upper byte
    rampsdec_msg.buf[4] = (mdec >> 8) & 0xFF;  // Upper byte

    // send the messages
    can1.write(imax_msg);
    can1.write(speed_msG);
    can1.write(icon_msg);
    can1.write(rampsacc_msg);
    can1.write(rampsdec_msg);

    // Serial.println(a);
    // torqueSM();
     Serial.println(mode);
  }

  previous_pos = pos;
}