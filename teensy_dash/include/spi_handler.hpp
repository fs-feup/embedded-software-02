#pragma once
#include <Arduino.h>

#include "data_struct.hpp"
#include "io_settings.hpp"
#include "spi/SPI_MSTransfer_T4.h"

class SpiHandler {
private:
  SPI_MSTransfer_T4<&SPI>& display_spi;
  uint16_t current_form;
  elapsedMillis temp_timer;
  elapsedMillis error_timer;
  elapsedMillis soc_timer;
  elapsedMillis all_temps_timer;
  elapsedMillis inverter_timer;
  elapsedMillis fast_timer;

  static constexpr uint16_t TEMP_INTERVAL = 2000;       // 2 seconds
  static constexpr uint16_t ERROR_INTERVAL = 500;       // 500ms
  static constexpr uint16_t SOC_INTERVAL = 3000;        // 3 seconds
  static constexpr uint16_t INVERTER_INTERVAL = 500;    // 500ms
  static constexpr uint16_t FAST_UPDATE_INTERVAL = 30;  // 30ms
  static constexpr uint16_t ALL_TEMPS_INTERVAL = 2000;  // 2 seconds

public:
  SpiHandler(SPI_MSTransfer_T4<&SPI>& spi);

  void setup();
  void handle_display_update(SystemData& data, const SystemVolatileData& updated_data);
};
