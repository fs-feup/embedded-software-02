#pragma once
#include <Bounce2.h>

#include <cstdint>

#include "data_struct.hpp"

class IOManager {
public:
  IOManager(SystemData& system_data, volatile SystemVolatileData& volatile_updatable_data,
            SystemVolatileData& volatile_updated_data);

  void setup();
  void manage();
  void read_apps();
  void play_r2d_sound();
  void play_buzzer(uint8_t duration_seconds);
  void calculate_rpm();
  void manage_ats();
  void read_rotative_switch();
  void read_hydraulic_pressure();
  void update_R2D_timer();

private:
  SystemData& data;
  volatile SystemVolatileData& updatable_data;
  SystemVolatileData& updated_data;
  inline static IOManager* instance = nullptr;
  void update_buzzer();
  void read_pins_handle_leds();
  Bounce r2d_button = Bounce();
  Bounce ats_button = Bounce();
  Bounce display_button = Bounce();
};

