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
  staticCallback = [this](const CAN_message_t& msg) { this->handleCanMessage(msg); };
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
  delay(20);

  CAN_message_t DCVoltageRequest = {.id = BAMO_COMMAND_ID, .len = 3, .buf = {0x3D, 0xEB, 0x64}};
  can1.write(DCVoltageRequest);
}

void CanCommHandler::can_snifflas(const CAN_message_t& msg) {
  if (staticCallback) {
    staticCallback(msg);
  }
}
void CanCommHandler::handleCanMessage(const CAN_message_t& msg) {
  switch (msg.id) {
    case BMS_ID:
      break;
    case BAMO_RESPONSE_ID:
      bamocar_callback(msg.buf, msg.len);
      break;
    case MASTER_ID:
      master_callback(msg.buf, msg.len);
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
      DEBUG_PRINTLN("BTB ready");
      break;

    case ENABLE_0:
      transmission_enabled = check_sequence(msg_data, ENABLE_SEQUENCE);
      DEBUG_PRINTLN("Transmission enabled");
      break;

    case SPEED_ACTUAL:
      updatable_data.speed = message_value;
      break;

    case SPEED_LIMIT: {
      // For debug purpose only, fow now
      const int normalized_limit = map(message_value, 0, 32767, 0, 100);
      DEBUG_PRINT("Speed limit:");
      DEBUG_PRINTLN(normalized_limit);
      break;
    }

    case DEVICE_I_MAX: {
      // Need to check if the in_max is correct or not: this value matches the ndrive read values
      const int normalized_imax = map(message_value, 0, 16369, 0, 100);
      DEBUG_PRINT("I_max:");
      DEBUG_PRINTLN(normalized_imax);
      break;
    }
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

void CanCommHandler::write_messages() {
  if (rpm_timer >= RPM_MSG_PERIOD_MS) {
    write_rpm();
    rpm_timer = 0;
  }

  if (hydraulic_timer >= HYDRAULIC_MSG_PERIOD_MS) {
    write_hydraulic_line();
    hydraulic_timer = 0;
  }

  if (apps_timer >= APPS_MSG_PERIOD_MS) {
    write_apps();
    apps_timer = 0;
  }

  const auto& current_mode = data.switch_mode;
  static auto previous_mode = current_mode;
  if (previous_mode != current_mode) {
    write_inverter_mode(current_mode);
    previous_mode = current_mode;
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

void CanCommHandler::write_hydraulic_line() {
  const uint16_t hydraulic_value = average_queue(data.brake_readings);

  CAN_message_t hydraulic_message;
  hydraulic_message.id = DASH_ID;
  hydraulic_message.len = 3;

  hydraulic_message.buf[0] = HYDRAULIC_LINE;
  hydraulic_message.buf[1] = hydraulic_value & 0xFF;         // Lower byte
  hydraulic_message.buf[2] = (hydraulic_value >> 8) & 0xFF;  // Upper byte

  can1.write(hydraulic_message);
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

void CanCommHandler::write_inverter_mode(const SwitchMode switch_mode) {
  constexpr int MAX_I_VALUE = 16383;
  constexpr int MAX_SPEED_VALUE = 32767;

  InverterModeParams params = get_inverter_mode_config(switch_mode);

#ifdef DEBUG_PRINTS
  auto mode_to_string = [](SwitchMode mode) -> const char* {
    switch (mode) {
      case SwitchMode::INVERTER_MODE_0:
        return "MODE_0";
      case SwitchMode::INVERTER_MODE_CAVALETES:
        return "CAVALETES";
      case SwitchMode::INVERTER_MODE_LIMITER:
        return "LIMITER";
      case SwitchMode::INVERTER_MODE_BRAKE_TEST:
        return "BRAKE_TEST";
      case SwitchMode::INVERTER_MODE_SKIDPAD:
        return "SKIDPAD";
      case SwitchMode::INVERTER_MODE_ENDURANCE:
        return "ENDURANCE";
      case SwitchMode::INVERTER_MODE_MAX_ATTACK:
        return "MAX_ATTACK";
      case SwitchMode::INVERTER_MODE_NULL:
        return "NULL";
      case SwitchMode::INVERTER_MODE_NULL2:
        return "NULL2";
      case SwitchMode::INVERTER_MODE_NULL3:
        return "NULL3";
      case SwitchMode::INVERTER_MODE_NULL4:
        return "NULL4";
      case SwitchMode::INVERTER_MODE_NULL5:
        return "NULL5";
      default:
        return "UNKNOWN";
    }
  };

  DEBUG_PRINT("Mode: ");
  DEBUG_PRINT(mode_to_string(switch_mode));
  DEBUG_PRINT(" | i_max: ");
  DEBUG_PRINT(params.i_max_pk_percent);
  DEBUG_PRINT("% | speed: ");
  DEBUG_PRINT(params.speed_limit_percent);
  DEBUG_PRINT("% | i_cont: ");
  DEBUG_PRINT(params.i_cont_percent);
  DEBUG_PRINT("% | s_acc: ");
  DEBUG_PRINT(params.speed_ramp_acc);
  DEBUG_PRINT(" | m_acc: ");
  DEBUG_PRINT(params.moment_ramp_acc);
  DEBUG_PRINT(" | s_brk: ");
  DEBUG_PRINT(params.speed_ramp_brake);
  DEBUG_PRINT(" | m_dec: ");
  DEBUG_PRINTLN(params.moment_ramp_decc);
#endif

  int i_max_pk = map(params.i_max_pk_percent, 0, 100, 0, MAX_I_VALUE);
  int i_cont = map(params.i_cont_percent, 0, 100, 0, MAX_I_VALUE);
  int speed_lim = map(params.speed_limit_percent, 0, 100, 0, MAX_SPEED_VALUE);

  CAN_message_t speed_limit_msg = {
      .id = BAMO_COMMAND_ID, .len = 3, .buf = {SPEED_LIMIT, 0x00, 0x00}};
  CAN_message_t i_max_msg = {.id = BAMO_COMMAND_ID, .len = 3, .buf = {DEVICE_I_MAX, 0x00, 0x00}};
  CAN_message_t i_cont_msg = {.id = BAMO_COMMAND_ID, .len = 3, .buf = {DEVICE_I_CNT, 0x00, 0x00}};
  CAN_message_t accRamp_msg = {
      .id = BAMO_COMMAND_ID, .len = 5, .buf = {SPEED_DELTAMA_ACC, 0x00, 0x00}};
  CAN_message_t deccRamp_msg = {
      .id = BAMO_COMMAND_ID, .len = 5, .buf = {SPEED_DELTAMA_DECC, 0x00, 0x00}};

  i_max_msg.buf[1] = i_max_pk & 0xFF;                // Lower byte
  i_max_msg.buf[2] = (i_max_pk >> 8) & 0xFF;         // Upper byte
  speed_limit_msg.buf[1] = speed_lim & 0xFF;         // Lower byte
  speed_limit_msg.buf[2] = (speed_lim >> 8) & 0xFF;  // Upper byte
  i_cont_msg.buf[1] = i_cont & 0xFF;                 // Lower byte
  i_cont_msg.buf[2] = (i_cont >> 8) & 0xFF;          // Upper byte

  accRamp_msg.buf[1] = params.speed_ramp_acc & 0xFF;          // Lower byte
  accRamp_msg.buf[2] = (params.speed_ramp_acc >> 8) & 0xFF;   // Upper byte
  accRamp_msg.buf[3] = params.moment_ramp_acc & 0xFF;         // Upper byte
  accRamp_msg.buf[4] = (params.moment_ramp_acc >> 8) & 0xFF;  // Upper byte

  deccRamp_msg.buf[1] = params.speed_ramp_brake & 0xFF;         // Lower byte
  deccRamp_msg.buf[2] = (params.speed_ramp_brake >> 8) & 0xFF;  // Upper byte
  deccRamp_msg.buf[3] = params.moment_ramp_decc & 0xFF;         // Upper byte
  deccRamp_msg.buf[4] = (params.moment_ramp_decc >> 8) & 0xFF;  // Upper byte

  can1.write(i_max_msg);
  can1.write(speed_limit_msg);
  can1.write(i_cont_msg);
  can1.write(accRamp_msg);
  can1.write(deccRamp_msg);
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
  static unsigned long stateStartTime = millis();
  static unsigned long lastActionTime = 0;
  static bool commandSent = false;
  static unsigned long currentTime = 0;
  currentTime = millis();

  switch (bamocarState) {
    case CHECK_BTB:
      if (currentTime - lastActionTime >= actionInterval) {
        DEBUG_PRINTLN("Checking BTB status");
        can1.write(checkBTBStatus);
        lastActionTime = currentTime;
      }
      if (btb_ready) {
        bamocarState = ENABLE_OFF;
        commandSent = false;
      } else if (currentTime - stateStartTime >= timeout) {
        DEBUG_PRINTLN("Timeout checking BTB");
        DEBUG_PRINTLN("Error during initialization");
        bamocarState = ERROR;
      }
      break;

    case ENABLE_OFF:
      DEBUG_PRINTLN("Disabling");
      can1.write(setEnableOff);
      bamocarState = ENABLE_TRANSMISSION;
      break;

    case ENABLE_TRANSMISSION:
      if (currentTime - lastActionTime >= actionInterval) {
        DEBUG_PRINTLN("Enabling transmission");
        can1.write(enableTransmission);
        lastActionTime = currentTime;
      }
      if (transmission_enabled) {
        bamocarState = ENABLE;
        commandSent = false;
        stateStartTime = currentTime;
        lastActionTime = currentTime;
      } else if (currentTime - stateStartTime >= timeout) {
        DEBUG_PRINTLN("Timeout enabling transmission");
        DEBUG_PRINTLN("Error during initialization");
        bamocarState = ERROR;
      }
      break;

    case ENABLE:
      if (!commandSent) {
        DEBUG_PRINTLN("Removing disable");
        can1.write(removeDisable);
        commandSent = true;
        bamocarState = ACC_RAMP;
        // request_dataLOG_messages();
      }
      break;

    case ACC_RAMP:
      DEBUG_PRINT("Transmitting acceleration ramp: ");
      DEBUG_PRINT(rampAccRequest.buf[1] | (rampAccRequest.buf[2] << 8));
      DEBUG_PRINTLN("ms");
      can1.write(rampAccRequest);
      bamocarState = DEC_RAMP;
      break;

    case DEC_RAMP:
      DEBUG_PRINT("Transmitting deceleration ramp: ");
      DEBUG_PRINT(rampDecRequest.buf[1] | (rampDecRequest.buf[2] << 8));
      DEBUG_PRINTLN("ms");
      can1.write(rampDecRequest);
      bamocarState = INITIALIZED;
      break;
    case INITIALIZED:
      return true;
    case ERROR:  // in the future add a retry mechanism
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