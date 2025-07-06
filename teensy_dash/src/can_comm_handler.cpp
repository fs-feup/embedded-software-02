#include "can_comm_handler.hpp"

#include <array>
#include <cstring>
#include <utils.hpp>

#include "../../CAN_IDs.h"
#include "io_settings.hpp"

CanCommHandler::CanCommHandler(SystemData& system_data,
                               volatile SystemVolatileData& volatile_updatable_data,
                               SystemVolatileData& volatile_updated_data,
                               SPI_MSTransfer_T4<&SPI>& display_spi)
    : data(system_data),
      updatable_data(volatile_updatable_data),
      updated_data(volatile_updated_data),
      display_spi(display_spi) {
  static_callback = [this](const CAN_message_t& msg) { this->handle_can_message(msg); };
}

void CanCommHandler::setup() {
  DEBUG_PRINTLN("Setting up CAN communication handler...");
  can1.begin();
  can1.setBaudRate(1'000'000);
  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  can1.setFIFOFilter(REJECT_ALL);
  can1.setFIFOFilter(0, BMS_THERMISTOR_ID, STD);
  can1.setFIFOFilter(1, BAMO_RESPONSE_ID, STD);
  can1.setFIFOFilter(2, MASTER_ID, STD);
  can1.onReceive(can_snifflas);
  delay(100);

  send_bamo_requests();
}

void CanCommHandler::send_bamo_requests() {
  constexpr CAN_message_t disable = {.id = BAMO_COMMAND_ID, .len = 3, .buf = {0x51, 0x04, 0x00}};

  // DC voltage request
  constexpr CAN_message_t dc_voltage_request = {
      .id = BAMO_COMMAND_ID, .len = 3, .buf = {0x3D, 0xEB, 0x64}};

  // Speed actual request
  constexpr CAN_message_t speed_actual_request = {
      .id = BAMO_COMMAND_ID, .len = 3, .buf = {0x3D, SPEED_ACTUAL, 0xFB}};
  constexpr CAN_message_t current_actual_request = {
      .id = BAMO_COMMAND_ID, .len = 3, .buf = {0x3D, CURRENT_ACTUAL, 0xFA}};
  constexpr CAN_message_t logicmap_errors_request = {
      .id = BAMO_COMMAND_ID, .len = 3, .buf = {0x3D, LOGICMAP_ERRORS, 0xEE}};
  constexpr CAN_message_t motor_temperature_request = {
      .id = BAMO_COMMAND_ID, .len = 3, .buf = {0x3D, MOTOR_TEMPERATURE, 0xEF}};
  // Send all messages (don't exceed 8 requests)
  can1.write(disable);
  can1.write(dc_voltage_request);
  can1.write(speed_actual_request);
  can1.write(current_actual_request);
  can1.write(logicmap_errors_request);
  can1.write(motor_temperature_request);
}

void CanCommHandler::can_snifflas(const CAN_message_t& msg) {
  if (static_callback) {
    static_callback(msg);
  }
}
void CanCommHandler::handle_can_message(const CAN_message_t& msg) {
  switch (msg.id) {
    case BMS_THERMISTOR_ID:
      bms_callback(msg.buf, msg.len);
      break;
    case BAMO_RESPONSE_ID:
      bamocar_callback(msg.buf, msg.len);
      break;
    case MASTER_ID:
      master_callback(msg.buf, msg.len);
      break;
    case BMS_ERRORS_ID: {
      // yves: estes são menos importantes mas se der mete tb
      DEBUG_PRINTLN("BMS Error ID received - Raw msg data:");
      for (uint8_t i = 0; i < msg.len; i++) {
        DEBUG_PRINTLN("  msg_data[" + String(i) + "] = 0x" + String(msg.buf[i], HEX));
      }

      // Handle DTC Status #1 (indices 0 and 1)
      if (msg.len >= 2) {
        uint16_t error_bitmap_1 = (msg.buf[1] << 8) | msg.buf[0];
        DEBUG_PRINTLN("BMS ERRORS #1: 0x" + String(error_bitmap_1, HEX));
        DEBUG_PRINTLN("DTC Status #1 error bits:");
        DEBUG_PRINTLN("  Bit 1 (0x0001): P0A07 (Discharge Limit Enforcement Fault): " +
                      String((error_bitmap_1 & (1 << 0)) ? 1 : 0));
        DEBUG_PRINTLN("  Bit 2 (0x0002): P0A08 (Charger Safety Relay Fault): " +
                      String((error_bitmap_1 & (1 << 1)) ? 1 : 0));
        DEBUG_PRINTLN("  Bit 3 (0x0004): P0A09 (Internal Hardware Fault): " +
                      String((error_bitmap_1 & (1 << 2)) ? 1 : 0));
        DEBUG_PRINTLN("  Bit 4 (0x0008): P0A0A (Internal Heatsink Thermistor Fault): " +
                      String((error_bitmap_1 & (1 << 3)) ? 1 : 0));
        DEBUG_PRINTLN("  Bit 5 (0x0010): P0A0B (Internal Software Fault): " +
                      String((error_bitmap_1 & (1 << 4)) ? 1 : 0));
        DEBUG_PRINTLN("  Bit 6 (0x0020): P0A0C (Highest Cell Voltage Too High Fault): " +
                      String((error_bitmap_1 & (1 << 5)) ? 1 : 0));
        DEBUG_PRINTLN("  Bit 7 (0x0040): P0A0E (Lowest Cell Voltage Too Low Fault): " +
                      String((error_bitmap_1 & (1 << 6)) ? 1 : 0));
        DEBUG_PRINTLN("  Bit 8 (0x0080): P0A10 (Pack Too Hot Fault): " +
                      String((error_bitmap_1 & (1 << 7)) ? 1 : 0));
      }

      // Handle DTC Status #2 (indices 2 and 3)
      if (msg.len >= 4) {
        uint16_t error_bitmap_2 = (msg.buf[3] << 8) | msg.buf[2];
        DEBUG_PRINTLN("BMS ERRORS #2: 0x" + String(error_bitmap_2, HEX));
        DEBUG_PRINTLN("DTC Status #2 error bits:");
        DEBUG_PRINTLN("  Bit 1 (0x0001): P0A1F (Internal Communication Fault): " +
                      String((error_bitmap_2 & (1 << 0)) ? 1 : 0));
        DEBUG_PRINTLN("  Bit 2 (0x0002): P0A12 (Cell Balancing Stuck Off Fault): " +
                      String((error_bitmap_2 & (1 << 1)) ? 1 : 0));
        DEBUG_PRINTLN("  Bit 3 (0x0004): P0A80 (Weak Cell Fault): " +
                      String((error_bitmap_2 & (1 << 2)) ? 1 : 0));
        DEBUG_PRINTLN("  Bit 4 (0x0008): P0AFA (Low Cell Voltage Fault): " +
                      String((error_bitmap_2 & (1 << 3)) ? 1 : 0));
        DEBUG_PRINTLN("  Bit 5 (0x0010): P0A04 (Open Wiring Fault): " +
                      String((error_bitmap_2 & (1 << 4)) ? 1 : 0));
        DEBUG_PRINTLN("  Bit 6 (0x0020): P0AC0 (Current Sensor Fault): " +
                      String((error_bitmap_2 & (1 << 5)) ? 1 : 0));
        DEBUG_PRINTLN("  Bit 7 (0x0040): P0A0D (Highest Cell Voltage Over 5V Fault): " +
                      String((error_bitmap_2 & (1 << 6)) ? 1 : 0));
        DEBUG_PRINTLN("  Bit 8 (0x0080): P0A0F (Cell ASIC Fault): " +
                      String((error_bitmap_2 & (1 << 7)) ? 1 : 0));
        DEBUG_PRINTLN("  Bit 9 (0x0100): P0A02 (Weak Pack Fault): " +
                      String((error_bitmap_2 & (1 << 8)) ? 1 : 0));
        DEBUG_PRINTLN("  Bit 10 (0x0200): P0A81 (Fan Monitor Fault): " +
                      String((error_bitmap_2 & (1 << 9)) ? 1 : 0));
        DEBUG_PRINTLN("  Bit 11 (0x0400): P0A9C (Thermistor Fault): " +
                      String((error_bitmap_2 & (1 << 10)) ? 1 : 0));
        DEBUG_PRINTLN("  Bit 12 (0x0800): U0100 (External Communication Fault): " +
                      String((error_bitmap_2 & (1 << 11)) ? 1 : 0));
        DEBUG_PRINTLN("  Bit 13 (0x1000): P0560 (Redundant Power Supply Fault): " +
                      String((error_bitmap_2 & (1 << 12)) ? 1 : 0));
        DEBUG_PRINTLN("  Bit 14 (0x2000): P0AA6 (High Voltage Isolation Fault): " +
                      String((error_bitmap_2 & (1 << 13)) ? 1 : 0));
        DEBUG_PRINTLN("  Bit 15 (0x4000): P0A05 (Input Power Supply Fault): " +
                      String((error_bitmap_2 & (1 << 14)) ? 1 : 0));
        DEBUG_PRINTLN("  Bit 16 (0x8000): P0A06 (Charge Limit Enforcement Fault): " +
                      String((error_bitmap_2 & (1 << 15)) ? 1 : 0));
      }
    } break;
    default:
      break;
  }
}
void CanCommHandler::bms_callback(const uint8_t* msg_data, uint8_t len) {
  if (len < 8) {
    DEBUG_PRINTLN("BMS message too short");
    return;
  }
  const auto min_temp = static_cast<uint16_t>(msg_data[1]);
  const auto max_temp = static_cast<uint16_t>(msg_data[2]);

  display_spi.transfer16(&min_temp, 1, WIDGET_CELLS_MIN, millis() & 0xFFFF);
  display_spi.transfer16(&max_temp, 1, WIDGET_CELLS_MAX, millis() & 0xFFFF);
}

void CanCommHandler::bamocar_callback(const uint8_t* const msg_data, const uint8_t len) {
  // All received messages are 3 bytes long, meaning 2 bytes of data,
  // unless otherwise specified (where it would be 4 bytes)
  // COB-ID   | DLC | Byte 1      | Byte 2      | Byte 3      | Byte 4
  // ---------|-----|-------------|-------------|-------------|-----------
  // TX       | 4   | RegID       | Data 07..00 | Data 15..08 | Stuff
  // |--------------| msg_data[0] | msg_data[1] | msg_data[2] | msg_data[3]
  // "To get the drive to send all replies as 6 byte messages (32-bit data) a bit in RegID 0xDC
  // has to be manually modified." - CAN-BUS BAMOCAR Manual

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
      // DEBUG_PRINTLN("BAMOCAR SPEED: " + String(message_value));
      break;
    case CURRENT_ACTUAL:
      // yves: corrente para o display
      updatable_data.motor_current = message_value;
      DEBUG_PRINTLN("BAMOCAR CURRENT: " + String(message_value));
      break;

    case LOGICMAP_ERRORS: {
      // Handle error bitmap
      const auto error_bitmap = static_cast<uint16_t>(message_value & 0x00FF);
      // updatable_data.inverter_errors = error_bitmap;

      display_spi.transfer16(&error_bitmap, 1, WIDGET_INVERTER_ERRORS, millis() & 0xFFFF);

      DEBUG_PRINTLN("LOGICMAP_ERRORS - Raw msg data:");
      for (uint8_t i = 0; i < len; i++) {
        DEBUG_PRINTLN("  msg_data[" + String(i) + "] = 0x" + String(msg_data[i], HEX));
      }

      // yves: estes são os erros que tem de dar display qd acontecerem, faz como achares melhor
      DEBUG_PRINTLN("BAMOCAR ERRORS: 0x" + String(error_bitmap, HEX));
      DEBUG_PRINTLN("Individual error bits:");
      DEBUG_PRINTLN("  Bit 0 (BADPARAS): " + String((error_bitmap & (1 << 0)) ? 1 : 0));
      DEBUG_PRINTLN("  Bit 1 (POWER FAULT): " + String((error_bitmap & (1 << 1)) ? 1 : 0));
      DEBUG_PRINTLN("  Bit 2 (RFE): " + String((error_bitmap & (1 << 2)) ? 1 : 0));
      DEBUG_PRINTLN("  Bit 3 (BUS TIMEOUT): " + String((error_bitmap & (1 << 3)) ? 1 : 0));
      DEBUG_PRINTLN("  Bit 4 (FEEDBACK): " + String((error_bitmap & (1 << 4)) ? 1 : 0));
      DEBUG_PRINTLN("  Bit 5 (POWERVOLTAGE): " + String((error_bitmap & (1 << 5)) ? 1 : 0));
      DEBUG_PRINTLN("  Bit 6 (MOTORTEMP): " + String((error_bitmap & (1 << 6)) ? 1 : 0));
      DEBUG_PRINTLN("  Bit 7 (DEVICETEMP): " + String((error_bitmap & (1 << 7)) ? 1 : 0));
      DEBUG_PRINTLN("  Bit 8 (OVERVOLTAGE): " + String((error_bitmap & (1 << 8)) ? 1 : 0));
      DEBUG_PRINTLN("  Bit 9 (I_PEAK): " + String((error_bitmap & (1 << 9)) ? 1 : 0));
      DEBUG_PRINTLN("  Bit 10 (RACEAWAY): " + String((error_bitmap & (1 << 10)) ? 1 : 0));
      DEBUG_PRINTLN("  Bit 11 (USER): " + String((error_bitmap & (1 << 11)) ? 1 : 0));
      DEBUG_PRINTLN("  Bit 12: " + String((error_bitmap & (1 << 12)) ? 1 : 0));
      DEBUG_PRINTLN("  Bit 13: " + String((error_bitmap & (1 << 13)) ? 1 : 0));
      DEBUG_PRINTLN("  Bit 14 (HW_ERR): " + String((error_bitmap & (1 << 14)) ? 1 : 0));
      DEBUG_PRINTLN("  Bit 15 (BALLAST): " + String((error_bitmap & (1 << 15)) ? 1 : 0));
    } break;
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

    case SOC_MSG: {
      updatable_data.soc = msg_data[1];

      const auto soc = static_cast<uint16_t>(msg_data[1]);
      display_spi.transfer16(&soc, 1, WIDGET_SOC, millis() & 0xFFFF);
    } break;
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
    // write_bamocar_speed();
    rpm_timer = 0;
    // DEBUG_PRINTLN("RPM message sent");
  }

  if (hydraulic_timer >= HYDRAULIC_MSG_PERIOD_MS) {
    write_hydraulic_line();
    hydraulic_timer = 0;
    // DEBUG_PRINTLN("Hydraulic line message sent");
  }

  if (apps_timer >= APPS_MSG_PERIOD_MS) {
    write_apps();
    apps_timer = 0;
    // DEBUG_PRINTLN("APPS message sent");
  }

  const auto& current_mode = data.switch_mode;
  static auto previous_mode = SwitchMode::INVERTER_MODE_INIT;
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

  const auto avg_rpm = static_cast<uint16_t>((data.fr_rpm + data.fl_rpm) / 2);
  display_spi.transfer16(&avg_rpm, 1, WIDGET_SPEED, millis() & 0xFFFF);
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

  // TODO CONVERT TO %(0-100)
  display_spi.transfer16(&hydraulic_value, 1, WIDGET_BRAKE, millis() & 0xFFFF);
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

  // send apps %(0-100) to display
  uint16_t torque_value = constrain(apps_higher, config::apps::MIN, config::apps::MAX);
  torque_value = config::apps::MAX - torque_value;
  uint16_t apps_percent = 0;
  if (torque_value > config::apps::DEADBAND) {
    const float normalized =
        static_cast<float>(torque_value - config::apps::DEADBAND) /
        static_cast<float>(config::apps::MAX_FOR_TORQUE - config::apps::DEADBAND);
    apps_percent = static_cast<uint8_t>(normalized * 100.0f);
  }
  display_spi.transfer16(&apps_percent, 1, WIDGET_THROTTLE, millis() & 0xFFFF);
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
  accRamp_msg.buf[3] = params.moment_ramp_acc & 0xFF;         // Lower byte
  accRamp_msg.buf[4] = (params.moment_ramp_acc >> 8) & 0xFF;  // Upper byte

  deccRamp_msg.buf[1] = params.speed_ramp_brake & 0xFF;         // Lower byte
  deccRamp_msg.buf[2] = (params.speed_ramp_brake >> 8) & 0xFF;  // Upper byte
  deccRamp_msg.buf[3] = params.moment_ramp_decc & 0xFF;         // Lower byte
  deccRamp_msg.buf[4] = (params.moment_ramp_decc >> 8) & 0xFF;  // Upper byte

  can1.write(i_max_msg);
  can1.write(speed_limit_msg);
  can1.write(i_cont_msg);
  can1.write(accRamp_msg);
  can1.write(deccRamp_msg);

  const auto inverter_mode = static_cast<uint16_t>(switch_mode);
  display_spi.transfer16(&inverter_mode, 1, WIDGET_INVERTER_MODE, millis() & 0xFFFF);
}

bool CanCommHandler::init_bamocar() {
  constexpr unsigned long actionInterval = 101;
  constexpr unsigned long timeout = 2000;

  // Drive disabled Enable internally switched off
  constexpr CAN_message_t disable = {.id = BAMO_COMMAND_ID, .len = 3, .buf = {0x51, 0x04, 0x00}};
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
  constexpr CAN_message_t clear_error_message = {
      .id = BAMO_COMMAND_ID, .len = 3, .buf = {0x8E, 0x00, 0x00}};

  static unsigned long currentTime = 0;
  currentTime = millis();

  switch (bamocar_state) {
    case CHECK_BTB:
      if (currentTime - last_action_time >= actionInterval) {
        DEBUG_PRINTLN("Checking BTB status");
        can1.write(checkBTBStatus);
        last_action_time = currentTime;
      }
      if (btb_ready) {
        bamocar_state = DISABLE;
        command_sent = false;
      } else if (currentTime - state_start_time >= timeout) {
        DEBUG_PRINTLN("Timeout checking BTB");
        DEBUG_PRINTLN("Error during initialization");
        bamocar_state = ERROR;
      }
      break;

    case DISABLE:
      DEBUG_PRINTLN("Disabling");
      can1.write(disable);
      bamocar_state = ENABLE_TRANSMISSION;
      break;

    case ENABLE_TRANSMISSION:
      if (currentTime - last_action_time >= actionInterval) {
        DEBUG_PRINTLN("Enabling transmission");
        can1.write(enableTransmission);
        last_action_time = currentTime;
      }
      if (transmission_enabled) {
        bamocar_state = ENABLE;
        command_sent = false;
        state_start_time = currentTime;
        last_action_time = currentTime;
      } else if (currentTime - state_start_time >= timeout) {
        DEBUG_PRINTLN("Timeout enabling transmission");
        DEBUG_PRINTLN("Error during initialization");
        bamocar_state = ERROR;
      }
      break;

    case ENABLE:
      if (!command_sent) {
        DEBUG_PRINTLN("Removing disable");
        can1.write(removeDisable);
        command_sent = true;
        bamocar_state = ACC_RAMP;
      }
      break;

    case ACC_RAMP:
      DEBUG_PRINT("Transmitting acceleration ramp: ");
      DEBUG_PRINT(rampAccRequest.buf[1] | (rampAccRequest.buf[2] << 8));
      DEBUG_PRINTLN("ms");
      can1.write(rampAccRequest);
      bamocar_state = DEC_RAMP;
      break;

    case DEC_RAMP:
      DEBUG_PRINT("Transmitting deceleration ramp: ");
      DEBUG_PRINT(rampDecRequest.buf[1] | (rampDecRequest.buf[2] << 8));
      DEBUG_PRINTLN("ms");
      can1.write(rampDecRequest);
      bamocar_state = CLEAR_ERRORS;
      break;
    case CLEAR_ERRORS:
      // This state is not used in the current initialization sequence
      // but can be used in the future to clear errors
      DEBUG_PRINTLN("Clearing errors");
      can1.write(clear_error_message);
      bamocar_state = INITIALIZED;
      break;
    case INITIALIZED:
      return true;
    case ERROR:  // in the future add a retry mechanism
      break;
  }

  return false;
}

void CanCommHandler::reset_bamocar_init() {
  bamocar_state = CHECK_BTB;
  state_start_time = millis();
  last_action_time = 0;
  command_sent = false;
}

void CanCommHandler::stop_bamocar() {
  constexpr CAN_message_t disable = {.id = BAMO_COMMAND_ID, .len = 3, .buf = {0x51, 0x04, 0x00}};

  can1.write(disable);
}

void CanCommHandler::send_torque(const int torque) {
  CAN_message_t torque_message;
  torque_message.id = BAMO_COMMAND_ID;
  torque_message.len = 3;
  torque_message.buf[0] = 0x90;
  torque_message.buf[1] = torque & 0xFF;         // Lower byte
  torque_message.buf[2] = (torque >> 8) & 0xFF;  // Upper byte

  can1.write(torque_message);
}