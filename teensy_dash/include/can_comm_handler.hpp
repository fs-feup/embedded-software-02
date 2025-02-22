#pragma once
#include <Arduino.h>
#include <FlexCAN_T4.h>

#include <cstdint>

#include "../../CAN_IDs.h"
#include "data_struct.hpp"
#include "utils.hpp"

class CanCommHandler {
public:
  CanCommHandler(SystemData& system_data, volatile SystemVolatileData& volatile_updatable_data,
                 SystemVolatileData& volatile_updated_data);

  void setup();
  void init_bamocar();
  void stop_bamocar();
  void write_periodic_messages();
  void send_torque(int torque);

private:
  inline static CanCommHandler* instance = nullptr;
  SystemData& data;
  volatile SystemVolatileData& updatable_data;
  SystemVolatileData& updated_data;
  FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can1;
  elapsedMillis can_timer;
  bool transmission_enabled = false;
  bool btb_ready = false;
  static void can_sniffer(const CAN_message_t& msg);
  void master_callback(const uint8_t* msg_data);
  void bamocar_callback(const uint8_t* msg_data);
  void write_rpm();
  void write_apps();
};

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
  can1.setBaudRate(500000);
  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  can1.setFIFOFilter(REJECT_ALL);
  can1.setFIFOFilter(2, BMS_ID, STD);
  can1.setFIFOFilter(3, BAMO_RESPONSE_ID, STD);
  can1.setFIFOFilter(4, MASTER_ID, STD);
  can1.onReceive(can_sniffer);
}

void CanCommHandler::can_sniffer(const CAN_message_t& msg) {
  switch (msg.id) {
    case BMS_ID:
      break;
    case BAMO_RESPONSE_ID:
      instance->bamocar_callback(msg.buf);
      break;
    case MASTER_ID:
      instance->master_callback(msg.buf);
      break;
    default:
      break;
  }
}

void CanCommHandler::bamocar_callback(const uint8_t* msg_data) {
  switch (msg_data[0]) {
    case DC_VOLTAGE: {
      long dc_voltage = 0;
      dc_voltage = (msg_data[2] << 8) | msg_data[1];
      updatable_data.TSOn = (dc_voltage >= DC_THRESHOLD);
      break;
    }
    case BTB_READY_0:
      btb_ready = check_sequence(msg_data, BTB_READY_SEQUENCE);
      break;

    case ENABLE_0:
      transmission_enabled = check_sequence(msg_data, ENABLE_SEQUENCE);
      break;

    case SPEED_VALUE:
      updatable_data.speed = (msg_data[2] << 8) | msg_data[1];
      break;

    default:
      break;
  }
}

void CanCommHandler::master_callback(const uint8_t* msg_data) {
  if (msg_data[0] == HYDRAULIC_LINE) {
    updatable_data.brake_pressure = (msg_data[2] << 8) | msg_data[1];
  }
  if (msg_data[0] == ASMS) {
    if (msg_data[1] == true) {  // TODO: maybe chnage this to a specif byte define that master will
                                // send when asms on
      updatable_data.asms_on = true;
    }
  }
}

void CanCommHandler::write_periodic_messages() {
  write_rpm();
  write_apps();
}

void CanCommHandler::write_rpm() {
  CAN_message_t rpm_message;
  rpm_message.id = DASH_ID;
  rpm_message.len = 5;

  char fr_rpm_byte[4];
  rpm_2_byte(data.fr_rpm, fr_rpm_byte);

  rpm_message.buf[0] = 0x10;  // TODO: add define
  rpm_message.buf[1] = fr_rpm_byte[0];
  rpm_message.buf[2] = fr_rpm_byte[1];
  rpm_message.buf[3] = fr_rpm_byte[2];
  rpm_message.buf[4] = fr_rpm_byte[3];
  can1.write(rpm_message);

  char fl_rpm_byte[4];
  rpm_2_byte(data.fl_rpm, fl_rpm_byte);

  rpm_message.buf[0] = 0x11;  // TODO: add define
  rpm_message.buf[1] = fl_rpm_byte[0];
  rpm_message.buf[2] = fl_rpm_byte[1];
  rpm_message.buf[3] = fl_rpm_byte[2];
  rpm_message.buf[4] = fl_rpm_byte[3];
  can1.write(rpm_message);
}

void CanCommHandler::write_apps() {
  int32_t apps1 = average_queue(data.apps1_readings);
  int32_t apps2 = average_queue(data.apps2_readings);

  CAN_message_t apps_message;
  apps_message.id = DASH_ID;
  apps_message.len = 5;

  apps_message.buf[0] = 0x20;  // TODO: add define

  apps_message.buf[1] = (apps1 >> 0) & 0xFF;
  apps_message.buf[2] = (apps1 >> 8) & 0xFF;
  apps_message.buf[3] = (apps1 >> 16) & 0xFF;
  apps_message.buf[4] = (apps1 >> 24) & 0xFF;
  can1.write(apps_message);

  apps_message.buf[1] = (apps2 >> 0) & 0xFF;
  apps_message.buf[2] = (apps2 >> 8) & 0xFF;
  apps_message.buf[3] = (apps2 >> 16) & 0xFF;
  apps_message.buf[4] = (apps2 >> 24) & 0xFF;
  can1.write(apps_message);
}

void CanCommHandler::init_bamocar() {
  CAN_message_t clear_errors;  // TODO: precreate msgs
  clear_errors.id = BAMO_COMMAND_ID;
  clear_errors.len = 3;
  clear_errors.buf[0] = 0x8E;
  clear_errors.buf[1] = 0x44;
  clear_errors.buf[2] = 0x4D;
  can1.write(clear_errors);

  CAN_message_t transmissionRequestEnable;
  transmissionRequestEnable.id = BAMO_COMMAND_ID;
  transmissionRequestEnable.len = 3;
  transmissionRequestEnable.buf[0] = 0x3D;
  transmissionRequestEnable.buf[1] = 0xE8;
  transmissionRequestEnable.buf[2] = 0x00;

  CAN_message_t BTBStatus;
  BTBStatus.id = BAMO_COMMAND_ID;
  BTBStatus.len = 3;
  BTBStatus.buf[0] = 0x3D;
  BTBStatus.buf[1] = 0xE2;
  BTBStatus.buf[2] = 0x00;

  CAN_message_t no_disable;
  no_disable.id = BAMO_COMMAND_ID;
  no_disable.len = 3;
  no_disable.buf[0] = 0x51;
  no_disable.buf[1] = 0x00;
  no_disable.buf[2] = 0x00;

  while (!transmission_enabled && can_timer > CAN_TIMEOUT_MS) {
    can1.write(transmissionRequestEnable);
    can_timer = 0;
  }

  while (!btb_ready && can_timer > CAN_TIMEOUT_MS) {
    can1.write(BTBStatus);
    can_timer = 0;
  }

  can1.write(no_disable);
}

void CanCommHandler::stop_bamocar() {
  CAN_message_t disable;
  disable.id = BAMO_COMMAND_ID;
  disable.len = 3;
  disable.buf[0] = 0x51;
  disable.buf[1] = 0x04;
  disable.buf[2] = 0x00;

  can1.write(disable);
}

void CanCommHandler::send_torque(int torque) {
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