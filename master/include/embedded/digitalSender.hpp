#pragma once

#include <Arduino.h>

#include "hardwareSettings.hpp"
#include "metro.h"

/**
 * @brief Class responsible for controlling digital outputs in the Master Teensy.
 *
 * The DigitalSender class handles various operations such as controlling LEDs,
 * EBS valves, SDC state, and watchdog signals. It also manages different operational states
 * such as emergency, manual, ready, driving, and finish.
 */
class DigitalSender {
private:
public:
  // Array of valid output pins
  static constexpr std::array<int, 9> validOutputPins = {
      ASSI_BLUE_PIN, ASSI_YELLOW_PIN, EBS_VALVE_REAR_PIN, EBS_VALVE_FRONT_PIN, SDC_BSPD_OUT,
      CLOSE_SDC,     BRAKE_LIGHT,     WD_SDC_CLOSE,    WD_ALIVE
      // SDC_LOGIC_WATCHDOG_OUT_PIN
  };

  /**
   * @brief Constructor for the DigitalSender class.
   *
   * Initializes the pins for output as defined in validOutputPins.
   */
  DigitalSender() {
    for (const auto pin : validOutputPins) {
      pinMode(pin, OUTPUT);
    }
    activate_ebs();
  }
  /**
   * @brief Turns off both ASSI LEDs (yellow and blue).
   */
  static void turn_off_assi();
  /**
   * @brief Opens the SDC in Master and SDC Logic.
   */
  static void open_sdc();

  /**
   * @brief Closes the SDC in Master and SDC Logic.
   */
  static void close_sdc();

  /**
   * @brief Activates the solenoid EBS valves.
   */
  static void activate_ebs();

  /**
   * @brief Deactivates the solenoid EBS valves.
   */
  static void deactivate_ebs();
  // Add these function declarations in the public section of the DigitalSender class:

  /**
   * @brief Disables EBS actuator 1.
   */
  static void disable_ebs_actuator_REAR();

  /**
   * @brief Enables EBS actuator 1.
   */
  static void enable_ebs_actuator_REAR();

  /**
   * @brief Disables EBS actuator 2.
   */
  static void disable_ebs_actuator_FRONT();

  /**
   * @brief Enables EBS actuator 2.
   */
  static void enable_ebs_actuator_FRONT();
  /**
   * @brief Blinks the LED at the given pin.
   * @param pin The pin to blink.
   */
  void blink_led(int pin);
  /**
   * @brief Turns on the brake light.
   */
  void turn_on_brake_light();
  /**
   * @brief Turns off the brake light.
   */
  void turn_off_brake_light();
  /**
   * @brief Turns on the BSPD error signal.
   */
  void bspd_error();
  /**
   * @brief Turns off the BSPD error signal.
   */
  void no_bspd_error();
  /**
   * @brief Toggles the watchdog signal.
   */
  static void toggle_watchdog();
  /**
   * @brief Closes the watchdog signal for SDC.
   */
  static void close_watchdog_sdc();
  /**
   * @brief Turns on the yellow ASSI LED.
   */
  void turn_on_yellow();
  /**
   * @brief Turns on the blue ASSI LED.
   */
  void turn_on_blue();
};

inline void DigitalSender::open_sdc() { digitalWrite(CLOSE_SDC, LOW); }

inline void DigitalSender::close_sdc() { DEBUG_PRINT("CLOSING SDC"); }

inline void DigitalSender::activate_ebs() {
  digitalWrite(EBS_VALVE_REAR_PIN, HIGH);
  digitalWrite(EBS_VALVE_FRONT_PIN, HIGH);
}

inline void DigitalSender::deactivate_ebs() {
  digitalWrite(EBS_VALVE_REAR_PIN, LOW);
  digitalWrite(EBS_VALVE_FRONT_PIN, LOW);
}

inline void DigitalSender::disable_ebs_actuator_REAR() { digitalWrite(EBS_VALVE_REAR_PIN, LOW); }

inline void DigitalSender::enable_ebs_actuator_REAR() { digitalWrite(EBS_VALVE_REAR_PIN, HIGH); }

inline void DigitalSender::disable_ebs_actuator_FRONT() { digitalWrite(EBS_VALVE_FRONT_PIN, LOW); }

inline void DigitalSender::enable_ebs_actuator_FRONT() { digitalWrite(EBS_VALVE_FRONT_PIN, HIGH); }

inline void DigitalSender::turn_off_assi() {
  analogWrite(ASSI_YELLOW_PIN, LOW);
  analogWrite(ASSI_BLUE_PIN, LOW);
}

inline void DigitalSender::turn_on_yellow() { analogWrite(ASSI_YELLOW_PIN, 1023); }

inline void DigitalSender::turn_on_blue() { analogWrite(ASSI_BLUE_PIN, 1023); }

inline void DigitalSender::blink_led(const int pin) {
  static bool blink_state = false;
  blink_state = !blink_state;
  analogWrite(pin, blink_state * 1023);
}

inline void DigitalSender::turn_on_brake_light() { digitalWrite(BRAKE_LIGHT, HIGH); }

inline void DigitalSender::turn_off_brake_light() { digitalWrite(BRAKE_LIGHT, LOW); }

inline void DigitalSender::bspd_error() { digitalWrite(SDC_BSPD_OUT, HIGH); }

inline void DigitalSender::no_bspd_error() { digitalWrite(SDC_BSPD_OUT, LOW); }

inline void DigitalSender::toggle_watchdog() {
  static volatile bool wd_state = false;
  wd_state = !wd_state;
  digitalWrite(WD_ALIVE, wd_state);
  // DEBUG_PRINT("Toggling watchdog: " + String(wd_state ? "ON" : "OFF"));
}

inline void DigitalSender::close_watchdog_sdc() { digitalWrite(WD_SDC_CLOSE, HIGH); }