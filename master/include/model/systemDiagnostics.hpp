#pragma once

#include <cstdlib>

#include "Arduino.h"
#include "debugUtils.hpp"
#include "embedded/hardwareSettings.hpp"
#include "metro.h"

struct R2DLogics {
  Metro readyTimestamp{READY_TIMEOUT_MS};

  /// Timestamp from when EBS is released on r2d,
  /// used to tolerate a small delay before entering driving state
  Metro releaseEbsTimestamp{RELEASE_EBS_TIMEOUT_MS};

  /// Timestamp from when EBS is activate on entering ready state,
  /// used to tolerate a small delay in which pneumatic line pressure is low
  Metro engageEbsTimestamp{ENGAGE_EBS_TIMEOUT_MS};
  bool r2d{false};

  /**
   * @brief resets timestamps for ready
   */
  void enter_ready_state() {
    readyTimestamp.reset();
    engageEbsTimestamp.reset();
    r2d = false;
  }

  /**
   * @brief resets timestamps for driving
   */
  void reset_ebs_timestamp() { releaseEbsTimestamp.reset(); }

  /**
   * @brief Processes the go signal.
   *
   * This function is responsible for processing the go signal.
   * It performs the necessary actions based on the received signal.
   *
   */

  void process_go_signal() {
    //if 5 seconds have passed all good, VVVRRRUUUMMMMM 
    if (readyTimestamp.check()) {
      r2d = true;
      return;
    }
    r2d = false;
    return;
  }
};

struct FailureDetection {
  Metro pc_alive_timestamp_{COMPONENT_TIMESTAMP_TIMEOUT};
  Metro steer_alive_timestamp_{COMPONENT_TIMESTAMP_TIMEOUT};
  Metro inversor_alive_timestamp_{COMPONENT_TIMESTAMP_TIMEOUT};
  Metro res_signal_loss_timestamp_{RES_TIMESTAMP_TIMEOUT};
  Metro dc_voltage_drop_timestamp_{DC_VOLTAGE_TIMEOUT};  // timer to check if dc voltage drops below
                                                         // threshold for more than 150ms
  Metro dc_voltage_hold_timestamp_{
      DC_VOLTAGE_HOLD};  // timer for ts on, only after enough voltage for 1 sec
  bool steer_dead_{true};
  bool pc_dead_{true};
  bool inversor_dead_{true};
  bool res_dead_{true};
  bool emergency_signal_{false};
  bool ts_on_{false};
  double radio_quality_{0};
  unsigned dc_voltage_{0};

  [[nodiscard]] bool has_any_component_timed_out() {  // no discard makes return value non ignorable

    steer_dead_ = steer_alive_timestamp_.checkWithoutReset();
    pc_dead_ = pc_alive_timestamp_.checkWithoutReset();
    inversor_dead_ = inversor_alive_timestamp_.checkWithoutReset();
    res_dead_ = res_signal_loss_timestamp_.checkWithoutReset();
    if (steer_dead_ || pc_dead_ || inversor_dead_ || res_dead_) {
      DEBUG_PRINT("=== System Component Status Check ===");
    }
    if (steer_dead_) {
      DEBUG_PRINT("Steering System: " + String(steer_dead_ ? "DEAD" : "ALIVE"));
    }
    if (pc_dead_) {
      DEBUG_PRINT("PC Connection: " + String(pc_dead_ ? "DEAD" : "ALIVE"));
    }
    if (inversor_dead_) {
      DEBUG_PRINT("Inverter Status: " + String(inversor_dead_ ? "DEAD" : "ALIVE"));
    }
    if (res_dead_) {
      DEBUG_PRINT("RES Signal: " + String(res_dead_ ? "DEAD" : "ALIVE"));
    }

    return steer_dead_ || pc_dead_ || inversor_dead_ || res_dead_;
  }
};
