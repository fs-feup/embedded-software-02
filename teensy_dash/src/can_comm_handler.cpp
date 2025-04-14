#include "can_comm_handler.hpp"

#include <array>
#include <cstring>
#include <utils.hpp>

#include "../../CAN_IDs.h"

CanCommHandler::CanCommHandler(SystemData& system_data,
                               volatile SystemVolatileData& volatile_updatable_data,
                               SystemVolatileData& volatile_updated_data)
    : data(system_data),
      updatable_data(volatile_updatable_data),
      updated_data(volatile_updated_data) {
  instance = this;
}

void CanCommHandler::setup() {
  can1.begin();
  can1.setBaudRate(1'000'000);
  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  can1.setFIFOFilter(REJECT_ALL);
  can1.setFIFOFilter(2, BMS_ID, STD);
  can1.setFIFOFilter(3, BAMO_RESPONSE_ID, STD);
  can1.setFIFOFilter(4, MASTER_ID, STD);
  can1.onReceive(can_snifflas);
}

void CanCommHandler::can_snifflas(const CAN_message_t& msg) {
  switch (msg.id) {
    case BMS_ID:
      break;
    case BAMO_RESPONSE_ID:
      if (instance) instance->bamocar_callback(msg.buf, msg.len);
      break;
    case MASTER_ID:
      if (instance) instance->master_callback(msg.buf, msg.len);
      break;
    default:
      break;
  }
}

void CanCommHandler::bamocar_callback(const uint8_t* const msg_data, const uint8_t len) {
  // All received messages are 3 bytes long, meaning 2 bytes of data,
  // unless otherwise specified (where it would be 4 bytes)
  // COB-ID   | DLC | Byte 1      | Byte 2      | Byte 3      | Byte 4
  // ---------|-----|-------------|-------------|-------------|-----------
  // TX       | 4   | RegID       | Data 07..00 | Data 15..08 | Stuff
  // |--------------| msg_data[0] | msg_data[1] | msg_data[2] | msg_data[3]
  // "To get the drive to send all replies as 6 byte messages (32-bit data) a bit in RegID 0xDC has
  // to be manually modified." - CAN-BUS BAMOCAR Manual

  // almost all messages seem to be signed
  int32_t message_value = 0;
  if (len == 4) {
    // Standard 16-bit data format
    message_value = (msg_data[2] << 8) | msg_data[1];
  } else if (len == 6) {
    // Extended 32-bit data format
    message_value = (msg_data[4] << 24) | (msg_data[3] << 16) | (msg_data[2] << 8) | msg_data[1];
  }

  switch (msg_data[0]) {
    case DC_VOLTAGE: {
      updatable_data.TSOn = (message_value >= DC_THRESHOLD);
      break;
    }
    case BTB_READY_0:
      btb_ready = check_sequence(msg_data, BTB_READY_SEQUENCE);
      Serial.println("BTB ready");
      break;

    case ENABLE_0:
      transmission_enabled = check_sequence(msg_data, ENABLE_SEQUENCE);
      Serial.println("Transmission enabled");
      break;

    case SPEED_ACTUAL:
      updatable_data.speed = message_value;
      break;

    case SPEED_LIMIT:
      // For debug purpose only, fow now
      const int normalized_limit = map(message_value, 0, 32767, 0, 100);
      Serial.print("Speed limit:");
      Serial.println(normalized_limit);
      break;

    case DEVICE_I_MAX:
      // Need to check if the in_max is correct or not: this value matches the ndrive read values
      const int normalized_imax = map(message_value, 0, 16369, 0, 100);
      Serial.print("I_max:");
      Serial.println(normalized_imax);
    default:
      break;
  }
}

void CanCommHandler::master_callback(const uint8_t* const msg_data, const uint8_t len) {
  switch (msg_data[0]) {
    case HYDRAULIC_LINE:
      updatable_data.brake_pressure = (msg_data[2] << 8) | msg_data[1];
      break;

    case ASMS:
      updatable_data.asms_on = msg_data[1];
      break;

    // TODO MAY BE IMPORTANT: src/can_comm_handler.cpp:78:5: warning: case label value exceeds
    // maximum value for type [-Wswitch-outside-range]
    case SOC_MSG:
      updatable_data.soc = msg_data[1];
      break;

    case STATE_MSG:
      updatable_data.as_state = msg_data[1];
      break;
    default:
      break;
  }
}

void CanCommHandler::write_periodic_messages() {
  if (rpm_timer >= RPM_MSG_PERIOD_MS) {
    write_rpm();
    rpm_timer = 0;
  }

  if (apps_timer >= APPS_MSG_PERIOD_MS) {
    write_apps();
    apps_timer = 0;
  }
}

void CanCommHandler::write_rpm() {
  CAN_message_t rpm_message;
  rpm_message.id = DASH_ID;
  rpm_message.len = 5;

  auto send_rpm = [this, &rpm_message](const uint8_t rpm_type, const float rpm_value) {
    const auto rpm_bytes = rpm_to_bytes(rpm_value);
    rpm_message.buf[0] = rpm_type;
    rpm_message.buf[1] = rpm_bytes[0];
    rpm_message.buf[2] = rpm_bytes[1];
    rpm_message.buf[3] = rpm_bytes[2];
    rpm_message.buf[4] = rpm_bytes[3];
    this->can1.write(rpm_message);
  };

  send_rpm(FR_RPM, data.fr_rpm);
  send_rpm(FL_RPM, data.fl_rpm);
}

void CanCommHandler::write_apps() {
  const int32_t apps_higher = average_queue(data.apps_higher_readings);
  const int32_t apps_lower = average_queue(data.apps_lower_readings);

  CAN_message_t apps_message;
  apps_message.id = DASH_ID;
  apps_message.len = 5;

  auto send_apps = [this, &apps_message](const uint8_t apps_type, const int32_t apps_value) {
    apps_message.buf[0] = apps_type;
    apps_message.buf[1] = (apps_value >> 0) & 0xFF;
    apps_message.buf[2] = (apps_value >> 8) & 0xFF;
    apps_message.buf[3] = (apps_value >> 16) & 0xFF;
    apps_message.buf[4] = (apps_value >> 24) & 0xFF;
    can1.write(apps_message);
  };

  send_apps(APPS_HIGHER, apps_higher);
  send_apps(APPS_LOWER, apps_lower);
}

bool CanCommHandler::init_bamocar() {
  constexpr unsigned long actionInterval = 100;
  constexpr unsigned long timeout = 2000;

  // Drive disabled Enable internally switched off
  constexpr CAN_message_t setEnableOff = {
      .id = BAMO_COMMAND_ID, .len = 3, .buf = {0x51, 0x04, 0x00}};
  constexpr CAN_message_t removeDisable = {
      .id = BAMO_COMMAND_ID, .len = 3, .buf = {0x51, 0x00, 0x00}};
  constexpr CAN_message_t checkBTBStatus = {
      .id = BAMO_COMMAND_ID, .len = 3, .buf = {0x3D, 0xE2, 0x00}};
  constexpr CAN_message_t enableTransmission = {
      .id = BAMO_COMMAND_ID, .len = 3, .buf = {0x3D, 0xE8, 0x00}};
  constexpr CAN_message_t rampAccRequest = {
      .id = BAMO_COMMAND_ID, .len = 3, .buf = {0x35, 0xF4, 0x01}};
  constexpr CAN_message_t rampDecRequest = {
      .id = BAMO_COMMAND_ID, .len = 3, .buf = {0xED, 0xE8, 0x03}};

  static BamocarState bamocarState = CHECK_BTB;
  static unsigned long stateStartTime = 0;
  static unsigned long lastActionTime = 0;
  static bool commandSent = false;
  static bool transmissionEnabled = false;
  static unsigned long currentTime = 0;
  currentTime = millis();

  switch (bamocarState) {
    case CHECK_BTB:
      if (currentTime - lastActionTime >= actionInterval) {
        Serial.println("Checking BTB status");
        can1.write(checkBTBStatus);
        lastActionTime = currentTime;
      }
      if (btb_ready) {  // CLION SAYS "Condition is always false": CAP
        bamocarState = ENABLE_OFF;
        commandSent = false;
      } else if (currentTime - stateStartTime >= timeout) {
        Serial.println("Timeout checking BTB");
        Serial.println("Error during initialization");
        bamocarState = ERROR;
      }
      break;

    case ENABLE_OFF:
      Serial.println("Disabling");
      can1.write(setEnableOff);
      bamocarState = ENABLE_TRANSMISSION;
    case ENABLE_TRANSMISSION:
      if (currentTime - lastActionTime >= actionInterval) {
        Serial.println("Enabling transmission");
        can1.write(enableTransmission);
        lastActionTime = currentTime;
      }
      if (transmissionEnabled) {
        bamocarState = ENABLE;
        commandSent = false;
        stateStartTime = currentTime;
        lastActionTime = currentTime;
      } else if (currentTime - stateStartTime >= timeout) {
        Serial.println("Timeout enabling transmission");
        Serial.println("Error during initialization");
        bamocarState = ERROR;
      }
      break;

    case ENABLE:
      if (!commandSent) {
        Serial.println("Removing disable");
        can1.write(removeDisable);
        commandSent = true;
        bamocarState = ACC_RAMP;
        // request_dataLOG_messages();
      }
      break;

    case ACC_RAMP:
      Serial.print("Transmitting acceleration ramp: ");
      Serial.print(rampAccRequest.buf[1] | (rampAccRequest.buf[2] << 8));
      Serial.print("ms");
      can1.write(rampAccRequest);
      bamocarState = DEC_RAMP;
      break;

    case DEC_RAMP:
      Serial.print("Transmitting deceleration ramp: ");
      Serial.print(rampDecRequest.buf[1] | (rampDecRequest.buf[2] << 8));
      Serial.print("ms");
      can1.write(rampDecRequest);
      bamocarState = INITIALIZED;
      break;
    case INITIALIZED:
      return true;
    case ERROR:
      break;
  }

  return false;
}

void CanCommHandler::stop_bamocar() {
  constexpr CAN_message_t setEnableOff = {
      .id = BAMO_COMMAND_ID, .len = 3, .buf = {0x51, 0x04, 0x00}};

  can1.write(setEnableOff);
}

void CanCommHandler::send_torque(const int torque) {
  if (torque_timer >= TORQUE_MSG_PERIOD_MS) {
    CAN_message_t torque_message;
    torque_message.id = BAMO_COMMAND_ID;
    torque_message.len = 3;
    torque_message.buf[0] = 0x90;
    torque_message.buf[1] = torque & 0xFF;         // Lower byte
    torque_message.buf[2] = (torque >> 8) & 0xFF;  // Upper byte

    can1.write(torque_message);
    torque_timer = 0;
  }
}