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
  elapsedMillis rpm_timer;     // Timer for RPM messages
  elapsedMillis apps_timer;    // Timer for APPS messages
  elapsedMillis torque_timer;  // Timer for torque commands
  bool transmission_enabled = false;
  bool btb_ready = false;
  static void can_sniffer(const CAN_message_t& msg);
  void master_callback(const uint8_t* msg_data);
  void bamocar_callback(const uint8_t* msg_data);
  void write_rpm();
  void write_apps();
};