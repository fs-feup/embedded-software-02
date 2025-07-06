#pragma once
#include <Arduino.h>
#include <FlexCAN_T4.h>

#include <cstdint>

#include "data_struct.hpp"
#include "spi/SPI_MSTransfer_T4.h"

class CanCommHandler {
public:
  CanCommHandler(SystemData& system_data, volatile SystemVolatileData& volatile_updatable_data,
                 SystemVolatileData& volatile_updated_data, SPI_MSTransfer_T4<&SPI>& display_spi);

  void setup();
  bool init_bamocar();
  void reset_bamocar_init();
  void stop_bamocar();
  void write_messages();
  void send_torque(int torque);

private:
  BamocarState bamocar_state = CHECK_BTB;
  unsigned long state_start_time = millis();
  unsigned long last_action_time = 0;
  bool command_sent = false;
  static inline std::function<void(const CAN_message_t&)> static_callback;
  static void can_snifflas(const CAN_message_t& msg);
  void handle_can_message(const CAN_message_t& msg);

  void bms_callback(const uint8_t* str, uint8_t len);
  void bamocar_callback(const uint8_t* msg_data, uint8_t len);
  void master_callback(const uint8_t* msg_data, uint8_t len);

  SystemData& data;
  volatile SystemVolatileData& updatable_data;
  SystemVolatileData& updated_data;

  SPI_MSTransfer_T4<&SPI>& display_spi;

  FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can1;
  elapsedMillis can_timer;
  elapsedMillis rpm_timer;        // Timer for RPM messages
  elapsedMillis hydraulic_timer;  // Timer for brake messages
  elapsedMillis apps_timer;       // Timer for APPS messages
  volatile bool transmission_enabled = false;
  volatile bool btb_ready = false;

  void send_bamo_requests();
  void write_rpm();
  void write_apps();
  void write_hydraulic_line();
  void write_inverter_mode(SwitchMode switch_mode);
};