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
  /// used to tolerate a small delay in which hydraulic line pressure is low
  Metro engageEbsTimestamp{ENGAGE_EBS_TIMEOUT_MS};
  bool r2d{false};

  /**
   * @brief resets timestamps for ready
   */
  void refresh_r2d_vars() {
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
    // if 5 seconds have passed all good, VVVRRRUUUMMMMM
    if (readyTimestamp.check()) {
      r2d = true;
      return;
    }
    r2d = false;
    return;
  }
};

struct NonUnitaryFailureDetection {
  VolatileMetro pc_alive_timestamp_{COMPONENT_TIMESTAMP_TIMEOUT};
  VolatileMetro steer_alive_timestamp_{COMPONENT_TIMESTAMP_TIMEOUT};
  VolatileMetro inversor_alive_timestamp_{COMPONENT_TIMESTAMP_TIMEOUT};
  VolatileMetro bms_alive_timestamp_{COMPONENT_TIMESTAMP_TIMEOUT};
  VolatileMetro res_signal_loss_timestamp_{RES_TIMESTAMP_TIMEOUT};
  VolatileMetro dc_voltage_drop_timestamp_{
      DC_VOLTAGE_TIMEOUT};  // timer to check if dc voltage drops below
                            // threshold for more than 150ms
  VolatileMetro dc_voltage_hold_timestamp_{
      DC_VOLTAGE_HOLD};  // timer for ts on, only after enough voltage for 1 sec

  NonUnitaryFailureDetection& operator=(const NonUnitaryFailureDetection& other) {
    if (this != &other) {
      pc_alive_timestamp_ = other.pc_alive_timestamp_;
      steer_alive_timestamp_ = other.steer_alive_timestamp_;
      inversor_alive_timestamp_ = other.inversor_alive_timestamp_;
      res_signal_loss_timestamp_ = other.res_signal_loss_timestamp_;
      dc_voltage_drop_timestamp_ = other.dc_voltage_drop_timestamp_;
      dc_voltage_hold_timestamp_ = other.dc_voltage_hold_timestamp_;
      bms_alive_timestamp_ = other.bms_alive_timestamp_;
    }
    return *this;
  }
};

struct FailureDetection {
  bool steer_dead_{false};
  bool pc_dead_{false};
  bool inversor_dead_{false};
  bool res_dead_{false};
  bool bms_dead_{false};

  volatile bool emergency_signal_{false};
  volatile bool ts_on_{false};
  volatile double radio_quality_{0};
  volatile unsigned dc_voltage_{0};

  // Reference to the timestamp struct
  NonUnitaryFailureDetection& timestamps_;

  // init ref
  FailureDetection(NonUnitaryFailureDetection& timestamps) : timestamps_(timestamps) {}

  [[nodiscard]] bool has_any_component_timed_out() {
    steer_dead_ = timestamps_.steer_alive_timestamp_.checkWithoutReset();
    pc_dead_ = timestamps_.pc_alive_timestamp_.checkWithoutReset();
    inversor_dead_ = timestamps_.inversor_alive_timestamp_.checkWithoutReset();
    res_dead_ = timestamps_.res_signal_loss_timestamp_.checkWithoutReset();
    bms_dead_ = timestamps_.bms_alive_timestamp_.checkWithoutReset();

    if (steer_dead_ || pc_dead_ || inversor_dead_ || res_dead_ || bms_dead_) {
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

    return steer_dead_ || pc_dead_ || inversor_dead_ || res_dead_ || bms_dead_;
  }
};
