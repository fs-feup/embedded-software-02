#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <elapsedMillis.h>

#include "helper.hpp"
#include "structs.hpp"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

elapsedMillis step;

PARAMETERS param;

bool request = 0;         // BMS request
bool shutdownStatus = 0;  // Charging shutdown status
bool latchingStatus = 0;  // Latching error status



Status CH_Status;     // current state machine status
Status NX_CH_Status;  // next state machine status

void extractValue(uint32_t &paramValue, const uint8_t *buf) {
  paramValue = 0;
  paramValue |= buf[4] << 24;
  paramValue |= buf[5] << 16;
  paramValue |= buf[6] << 8;
  paramValue |= buf[7];
}

void printValue(const char *label, const uint32_t value) {
  Serial.print(label);
  Serial.print(value);
}

void handleSetDataResponse(const CAN_message_t &message) {
  switch (message.buf[1]) {
    case SET_VOLTAGE_RESPONSE: {
      extractValue(param.setVoltage, message.buf);
      printValue("Voltage Set= ", param.setVoltage);
      break;
    }

    case SET_CURRENT_RESPONSE: {
      extractValue(param.setCurrent, message.buf);
      printValue("Current Set= ", param.setCurrent);
      break;
    }

    default: {
      break;
    }
  }
}

void handleReadDataResponse(const CAN_message_t &message) {
  switch (message.buf[1]) {
    case CURRENT_VOLTAGE_RESPONSE: {
      extractValue(param.currVoltage, message.buf);
      printValue("Current voltage= ", param.currVoltage);
      break;
    }

    case CURRENT_CURRENT_RESPONSE: {
      extractValue(param.currCurrent, message.buf);
      printValue("Current Current= ", param.currCurrent);
      break;
    }

    default: {
      break;
    }
  }
}

void parseChargerMessage(const CAN_message_t &message) {
  switch (message.buf[0]) {
    case SET_DATA_RESPONSE: {
      handleSetDataResponse(message);
      break;
    }

    case READ_DATA_RESPONSE: {
      handleReadDataResponse(message);
      break;
    }

    default: {
      break;
    }
  }
}

void parseMessage(const CAN_message_t &message) {
  switch (message.id) {
    case CHARGER_ID: {
      parseChargerMessage(message);
      break;
    }

    case BMS_ID_CCL: {
      param.ccl = message.buf[0] * 1000;
      break;
    }

    case TEMPERATURES_ID: {//TBD
      int offset = (message.buf[0] & 0xF) * 7;
      for (int i = 0; i < 7 && (i + offset) < 60; i++) {
        param.temp[i + offset] = message.buf[i + 1];
      }
      break;
    }

    default: {
      break;
    }
  }
}

void canint(const CAN_message_t &message) { parseMessage(message); }
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
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
  can1.setFIFOFilter(3, TEMPERATURES_ID, STD);
  can1.onReceive(canint);

  param.setVoltage = MAX_VOLTAGE;
}

void chargerMachine() {
  switch (CH_Status) {
    case Status::IDLE: {
      if (shutdownStatus) {
        NX_CH_Status = Status::SHUTDOWN;
      } else if (request == 1 and latchingStatus == 1) {
        NX_CH_Status = Status::CHARGING;
      }
      break;
    }
    case Status::CHARGING: {
      if (shutdownStatus) {
        NX_CH_Status = Status::SHUTDOWN;
      } else if (request == 0 or latchingStatus == 0) {
        NX_CH_Status = Status::IDLE;
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

void readInputs() {
  shutdownStatus = digitalRead(SHUTDOWN_PIN);
  latchingStatus = digitalRead(LATCHING_ERROR_PIN);
}

void powerOnModule(const bool OnOff) {
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

void setVoltage(uint32_t voltage) {
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

void setCurrent(const uint32_t current) {
  CAN_message_t currentMsg;

  currentMsg.id = CHARGER_ID;
  currentMsg.flags.extended = 1;
  currentMsg.len = 8;
  currentMsg.buf[0] = 0x10;
  currentMsg.buf[1] = 0x03;
  currentMsg.buf[2] = 0x00;
  currentMsg.buf[3] = 0x00;
  currentMsg.buf[4] = current >> 24 & 0xff;
  currentMsg.buf[5] = current >> 16 & 0xff;
  currentMsg.buf[6] = current >> 8 & 0xff;
  currentMsg.buf[7] = current & 0xff;

  can1.write(currentMsg);  // send message
}

void setLOW() {
  CAN_message_t LowMsg;

  LowMsg.id = CHARGER_ID;
  LowMsg.flags.extended = 1;
  LowMsg.len = 8;
  LowMsg.buf[0] = 0x10;
  LowMsg.buf[1] = 0x5f;
  LowMsg.buf[2] = 0x00;
  LowMsg.buf[3] = 0x00;
  LowMsg.buf[4] = 0x00;
  LowMsg.buf[5] = 0x00;
  LowMsg.buf[6] = 0x00;
  LowMsg.buf[7] = 0x00;

  can1.write(LowMsg);  // send message
}

void readCurrent() {
  CAN_message_t requestMsg;
  requestMsg.id = REQUEST_CURRENT_ID;
  requestMsg.len = 8;
  requestMsg.buf[0] = 0x12;  // Byte0: GroupAddress and MessageType
  requestMsg.buf[1] = 0x01;  // Byte1: CommandType (Read current)
  requestMsg.buf[2] = 0x00;  // Byte2: Reserved
  requestMsg.buf[3] = 0x00;  // Byte3: Reserved
  requestMsg.buf[4] = 0x00;  // Byte4: Reserved
  requestMsg.buf[5] = 0x00;  // Byte5: Reserved
  requestMsg.buf[6] = 0x00;  // Byte6: Reserved
  requestMsg.buf[7] = 0x00;  // Byte7: Reserved

  can1.write(requestMsg);
}
void updateCharger(Status CH_Status) {
  setCurrent(param.allowedCurrent);
  setVoltage(MAX_VOLTAGE);
  powerOnModule(CH_Status == Status::CHARGING);
}

void loop() {
  if (step < 1000) {
    return;
  }

  step = 0;

  readInputs();
  chargerMachine();
  CH_Status = NX_CH_Status;

  if (param.ccl < MAX_CURRENT) {
    param.allowedCurrent = param.ccl;
  } else {
    param.allowedCurrent = MAX_CURRENT;
  }

  for (int i = 0; i < 60; i++) {
    Serial.print("temp");
    Serial.print(i);
    Serial.print(": ");

    if (i == 20 || i == 21 || i == 29) {
      Serial.println(param.temp[19]);
      continue;
    }
    if (param.temp[i] == 0) {
      Serial.println("Thermistor turned off");
    } else {
      Serial.println(param.temp[i]);
    }
  }

  updateCharger(CH_Status);

  Serial.printf("ccl: %d\n", param.ccl);

  Serial.println("loop");
}