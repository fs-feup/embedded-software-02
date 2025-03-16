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
      ASSI_BLUE_PIN, ASSI_YELLOW_PIN, EBS_VALVE_1_PIN, EBS_VALVE_2_PIN, SDC_BSPD_OUT,
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
   * @brief Starts the watchdog signal.
   */
  static void start_watchdog();
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

inline void DigitalSender::close_sdc() { digitalWrite(CLOSE_SDC, HIGH); }

inline void DigitalSender::activate_ebs() {
  digitalWrite(EBS_VALVE_1_PIN, HIGH);
  digitalWrite(EBS_VALVE_2_PIN, HIGH);
}

inline void DigitalSender::deactivate_ebs() {
  digitalWrite(EBS_VALVE_1_PIN, LOW);
  digitalWrite(EBS_VALVE_2_PIN, LOW);
}

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
  static bool wd_state = false;
  wd_state = !wd_state;
  digitalWrite(WD_ALIVE, wd_state);  // TODO wd routine review
}

inline void DigitalSender::start_watchdog() { digitalWrite(WD_ALIVE, HIGH); }

inline void DigitalSender::close_watchdog_sdc() { digitalWrite(WD_SDC_CLOSE, HIGH); }