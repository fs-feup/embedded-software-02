#include "spi_handler.hpp"

#include "../../CAN_IDs.h"

SpiHandler::SpiHandler(SPI_MSTransfer_T4<&SPI>& spi) : display_spi(spi), current_form(0) {}

void SpiHandler::setup() {
  display_spi.begin();
  delay(100);
}

void SpiHandler::handle_display_update(SystemData& data, const SystemVolatileData& updated_data) {
  if (data.display_pressed) {
    current_form = (current_form % 3) + 1;
    display_spi.transfer16(&current_form, 1, WIDGET_FORM_CMD, millis() & 0xFFFF);
    data.display_pressed = false;
  }

  if (fast_timer >= FAST_UPDATE_INTERVAL) {
    fast_timer = 0;
    // Fast updates (every loop iteration) - critical for pilot feedback
    const uint16_t apps_higher = average_queue(data.apps_higher_readings);
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

    // Hydraulic brake - fast for pilot feedback
    const uint16_t hydraulic_value = average_queue(data.brake_readings);
    display_spi.transfer16(&hydraulic_value, 1, WIDGET_BRAKE, millis() & 0xFFFF);

    // Speed - fast for pilot feedback
    const uint16_t avg_rpm = static_cast<uint16_t>((data.fr_rpm + data.fl_rpm) / 2);
    const uint16_t speed_kmh = avg_rpm * 0.07656;
    display_spi.transfer16(&speed_kmh, 1, WIDGET_SPEED, millis() & 0xFFFF);
  }

  // Temperature updates every 2 seconds
  if (temp_timer >= TEMP_INTERVAL) {
    uint16_t min_temp_16 = updated_data.min_temp;
    display_spi.transfer16(&min_temp_16, 1, WIDGET_CELLS_MIN, millis() & 0xFFFF);
    uint16_t max_temp_16 = updated_data.max_temp;
    display_spi.transfer16(&max_temp_16, 1, WIDGET_CELLS_MAX, millis() & 0xFFFF);
    temp_timer = 0;
  }

  // Error updates every 300ms
  if (error_timer >= ERROR_INTERVAL) {
    uint16_t error_bitmap = updated_data.error_bitmap;
    display_spi.transfer16(&error_bitmap, 1, WIDGET_INVERTER_ERRORS, millis() & 0xFFFF);
    uint16_t warning_bitmap = updated_data.warning_bitmap;
    display_spi.transfer16(&warning_bitmap, 1, WIDGET_INVERTER_WARNINGS, millis() & 0xFFFF);
    error_timer = 0;
  }

  // SOC updates every 2/3 seconds
  if (soc_timer >= SOC_INTERVAL) {
    uint16_t soc = updated_data.soc;
    display_spi.transfer16(&soc, 1, WIDGET_SOC, millis() & 0xFFFF);
    soc_timer = 0;
  }

  // Inverter mode updates every 500ms
  if (inverter_timer >= INVERTER_INTERVAL) {
    const auto inverter_mode = static_cast<uint16_t>(data.switch_mode);
    display_spi.transfer16(&inverter_mode, 1, WIDGET_INVERTER_MODE, millis() & 0xFFFF);
    inverter_timer = 0;
    //publish autonomous mission
    uint16_t autonomous_mission = updated_data.autonomous_mission;
    display_spi.transfer16(&autonomous_mission, 1, WIDGET_AUTONOMOUS_MISSION, millis() & 0xFFFF); 
  }
}