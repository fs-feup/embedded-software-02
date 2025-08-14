#pragma once
#include <Arduino.h>
#include <elapsedMillis.h>

#include <deque>

enum class State { IDLE, INITIALIZING_DRIVING, DRIVING, INITIALIZING_AS_DRIVING, AS_DRIVING };

enum class SwitchMode {
  INVERTER_MODE_SCRUT,
  INVERTER_MODE_CRUISING,
  INVERTER_MODE_AS_ACCELERATION,
  INVERTER_MODE_SKIDPAD,
  INVERTER_MODE_ENDURANCE,
  INVERTER_MODE_AUTOCROSS,
  INVERTER_MODE_ACCELERATION,
  INVERTER_MODE_FAST_ACCELERATION,
  INVERTER_MODE_INIT  // for the initial previous mode
};

/* From NDrive Manual:
 * "Sequence for enabling with hardwired RFE and RUN (FRG) input:
 * 1. First lock the servo with the command ENABLE OFF (MODE BIT 0x51Bit 2 = 1).
 * 2. Then unlock the servo with the command NOT ENABLE OFF (MODE BIT 0x51Bit 2 = 0).
 *    The servo is enabled without delay.
 *    → Only in this order can an enabled be achieved.
 *    → At the same time, all stored errors are deleted."
 *
 * NOTE: Search for MODE BITS in Ndrive Manual for info about the 0x51 command.
 */
enum BamocarState {
  ENABLE_TRANSMISSION,
  CHECK_BTB,
  DISABLE,
  ENABLE,
  ACC_RAMP,
  DEC_RAMP,
  INITIALIZED,
  ERROR,
  CLEAR_ERRORS
};
constexpr unsigned long STABLE_TIME_MS = 150;

struct InverterModeParams {
  int i_max_pk_percent = 0;
  int speed_limit_percent = 0;
  int i_cont_percent = 0;
  int speed_ramp_acc = 0;    // 0..30'000
  int moment_ramp_acc = 0;   // 0..4000
  int speed_ramp_brake = 0;  // 0..30'000
  int moment_ramp_decc = 0;  // 0..4000
};

struct SystemData {
  State current_state = State::IDLE;
  bool r2d_pressed = false;
  bool ats_pressed = false;
  bool implausibility = false;
  bool display_pressed = false;
  SwitchMode switch_mode = SwitchMode::INVERTER_MODE_SCRUT;
  bool buzzer_active = false;
  unsigned long buzzer_start_time;
  unsigned long buzzer_duration_ms;
  bool emergency_buzzer_active = false;
  unsigned long emergency_buzzer_start_time;
  bool emergency_buzzer_state = false;
  std::deque<uint16_t> apps_higher_readings;
  std::deque<uint16_t> apps_lower_readings;
  float fr_rpm = 0;
  float fl_rpm = 0;
  std::deque<uint16_t> brake_readings;

  elapsedMillis r2d_brake_timer = 0;
};

struct SystemVolatileData {
  bool TSOn = false;
  uint8_t as_state = 0;
  bool asms_on = false;
  int brake_pressure = 0;
  int speed = 0;
  uint8_t soc = 0;
  int32_t motor_current = 0;
  uint8_t min_temp = 0;
  uint8_t max_temp = 0;
  uint16_t error_bitmap = 0;
  uint16_t warning_bitmap = 0;
  uint8_t autonomous_mission = 0;
  uint8_t hv_soc = 0;

  int8_t cell_board_all_temps[6][18] = {{0}};

  unsigned long last_wheel_pulse_fr = 0;
  unsigned long second_to_last_wheel_pulse_fr = 0;
  unsigned long last_wheel_pulse_fl = 0;
  unsigned long second_to_last_wheel_pulse_fl = 0;
};

inline void copy_volatile_data(SystemVolatileData& dest, volatile SystemVolatileData const& src) {
  noInterrupts();
  dest.TSOn = src.TSOn;
  dest.as_state = src.as_state;
  dest.asms_on = src.asms_on;
  dest.brake_pressure = src.brake_pressure;
  dest.speed = src.speed;
  dest.soc = src.soc;
  dest.last_wheel_pulse_fr = src.last_wheel_pulse_fr;
  dest.second_to_last_wheel_pulse_fr = src.second_to_last_wheel_pulse_fr;
  dest.last_wheel_pulse_fl = src.last_wheel_pulse_fl;
  dest.second_to_last_wheel_pulse_fl = src.second_to_last_wheel_pulse_fl;
  dest.motor_current = src.motor_current;
  dest.min_temp = src.min_temp;
  dest.max_temp = src.max_temp;
  dest.error_bitmap = src.error_bitmap;
  dest.warning_bitmap = src.warning_bitmap;
  dest.autonomous_mission = src.autonomous_mission;
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 18; ++j) {
      dest.cell_board_all_temps[i][j] = src.cell_board_all_temps[i][j];
    }
  }
  dest.hv_soc = src.hv_soc;
  interrupts();
}