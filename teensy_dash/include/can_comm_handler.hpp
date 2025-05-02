#pragma once
#include <Arduino.h>
#include <FlexCAN_T4.h>

#include <cstdint>

#include "data_struct.hpp"

class CanCommHandler {
public:
  CanCommHandler(SystemData& system_data, volatile SystemVolatileData& volatile_updatable_data,
                 SystemVolatileData& volatile_updated_data);

  void setup();
  bool init_bamocar();
  void stop_bamocar();
  void write_messages();
  void send_torque(int torque);

private:
  static inline std::function<void(const CAN_message_t&)> staticCallback;
  static void can_snifflas(const CAN_message_t& msg);
  void handleCanMessage(const CAN_message_t& msg);

  void bamocar_callback(const uint8_t* msg_data, uint8_t len);
  void master_callback(const uint8_t* msg_data, uint8_t len);

  SystemData& data;
  volatile SystemVolatileData& updatable_data;
  SystemVolatileData& updated_data;

  FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1; // yves - had to change this to CAN1 for bamocar
  elapsedMillis can_timer;
  elapsedMillis rpm_timer;     // Timer for RPM messages
  elapsedMillis apps_timer;    // Timer for APPS messages
  elapsedMillis torque_timer;  // Timer for torque commands
  bool transmission_enabled = false;
  bool btb_ready = false;

  void write_rpm();
  void write_apps();
  void write_inverter_mode(SwitchMode switch_mode);
};