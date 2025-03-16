#pragma once

#include <cstdlib>

#include "debugUtils.hpp"
#include "embedded/digitalSender.hpp"
#include "embedded/hardwareSettings.hpp"
#include "model/systemData.hpp"

// Also known as Orchestrator
/**
 * @brief The CheckupManager class handles various checkup operations.
 */
class CheckupManager {
private:
  SystemData *_system_data_;  ///< Pointer to the system data object containing system status and
                              ///< sensor information.
  uint8_t pressure_test_phase_{0};
  Metro _watchdog_toggle_timer_{WATCHDOG_TOGGLE_DURATION};  ///< Timer for watchdog toggle sequence
  Metro _watchdog_test_timer_{WATCHDOG_TEST_DURATION};      ///< Timer for watchdog verification
  bool _watchdog_toggle_in_timer_{false};  ///< Flag to indicate if the watchdog toggle is in
                                           ///< progress in the interrupt timer.

  /**
   * @brief Checks if the pneumatic pressure is high enough, meaning EBS worked.
   * @return True if the pressure is high enough, false otherwise.
   */
  bool check_pressure_high() const;

  /**
   * @brief Checks if the pneumatic pressure is low enough, meaning EBS retracted.
   * @return True if the pressure is low enough, false otherwise.
   */
  bool check_pressure_low() const;

  /**
   * @brief Checks if the vehicle has failed to build pneumatic pressure in the limit time
   * (definido em engage ebs timestamp).
   */
  bool failed_to_build_pneumatic_pressure_in_engage_time() const;
  /**
   * @brief Checks if the vehicle has failed to build pneumatic pressure in the limit time
   * (aqui há alguma redundância, no sentido de já ter sido feito check da linha antes).
   */
  bool failed_to_build_pneumatic_pressure_in_release_time() const;
  /**
   * @brief Checks if the vehicle has failed to build hydraulic pressure in the limit time
   * (definido em engage ebs otv, quando entramos em ready ebs é ativado entao a pressão tem de
   * aumentar).
   */
  bool failed_to_build_hydraulic_pressure_in_time() const;
  /**
   * @brief Checks if the vehicle has failed to reduce hydraulic pressure in the limit time
   * (definido no release ebs, entrando em driving ebs é released).
   */
  bool failed_to_reduce_hydraulic_pressure_in_time() const;

  /**
   * @brief Handles the EBS checkup.
   */
  void handle_ebs_check();

public:
  Metro _ebs_sound_timestamp_{EBS_BUZZER_TIMEOUT};  ///< Timer for the EBS buzzer sound check.

  /**
   * @brief The CheckupState enum represents the different states of
   * the initial checkup process.
   * The checkup process is a sequence of checks that the vehicle must pass
   * before it can transition to ready state.
   */
  enum class CheckupState {
    WAIT_FOR_ASMS,
    START_WATCHDOG,
    TOGGLING_WATCHDOG,
    STOP_TOGGLING_WATCHDOG,
    CHECK_WATCHDOG,
    CLOSE_SDC,
    WAIT_FOR_ASATS,
    WAIT_FOR_TS,
    EBS_CHECKS, /** activate and deactivate ebs and -> check pressures are correct */
    CHECK_TIMESTAMPS,
    CHECKUP_COMPLETE
  };

  /**
   * This is for easier debugging in case initial checkup fails
   */
  enum class CheckupError { WAITING_FOR_RESPONSE, ERROR, SUCCESS };

  CheckupState checkup_state_{
      CheckupState::WAIT_FOR_ASMS};  ///< Current state of the checkup process.

  /**
   * @brief Constructor for the CheckupManager class.
   * @param system_data Pointer to the system data object.
   */
  explicit CheckupManager(SystemData *system_data) : _system_data_(system_data) {};

  /**
   * @brief Resets the checkup state to the initial state
   */
  void reset_checkup_state();

  /**
   * @brief Performs a manual driving checkup.
   */
  [[nodiscard]] bool should_stay_manual_driving() const;

  /**
   * @brief Performs an off checkup.
   */
  bool should_stay_off();

  /**
   * @brief Performs an initial checkup.
   */
  CheckupError initial_checkup_sequence();

  /**
   * @brief Performs a last re-check for off to ready transition.
   */
  [[nodiscard]] bool should_go_ready_from_off() const;

  /**
   * @brief Performs a ready to drive checkup.
   */
  [[nodiscard]] bool should_stay_ready() const;

  /**
   * @brief Performs an emergency checkup.
   */
  [[nodiscard]] bool should_enter_emergency(State current_state) const;

  bool should_enter_emergency_in_ready_state() const;
  bool should_enter_emergency_in_driving_state() const;

  [[nodiscard]] bool should_stay_driving() const;

  /**
   * @brief Performs a mission finished checkup.
   */
  [[nodiscard]] bool should_stay_mission_finished() const;

  /**
   * @brief Checks if the emergency sequence is complete and the vehicle can
   * transition to AS_OFF.
   */
  [[nodiscard]] bool emergency_sequence_complete() const;

  /**
   * @brief Checks if the RES has been triggered.
   *
   * This function checks whether the RES has been triggered or not.
   *
   */
  [[nodiscard]] bool res_triggered() const;

  [[nodiscard]] bool should_toggle_watchdog();
};

inline void CheckupManager::reset_checkup_state() {
  checkup_state_ = CheckupState::WAIT_FOR_ASMS;
  _system_data_->mission_finished_ = false;
}

inline bool CheckupManager::should_stay_manual_driving() const {
  if (_system_data_->mission_ != Mission::MANUAL ||
      _system_data_->hardware_data_.pneumatic_line_pressure_ != 0 ||
      _system_data_->hardware_data_.asms_on_) {
    return false;
  }

  return true;
}

inline bool CheckupManager::should_stay_off() {
  CheckupError init_sequence_state = initial_checkup_sequence();

  if (init_sequence_state != CheckupError::SUCCESS) {
    return true;
  }
  return false;
}

inline CheckupManager::CheckupError CheckupManager::initial_checkup_sequence() {
  switch (checkup_state_) {  // TODO
    case CheckupState::WAIT_FOR_ASMS:
      if (_system_data_->hardware_data_.asms_on_) {
        checkup_state_ = CheckupState::START_WATCHDOG;
        DEBUG_PRINT("ASMS activated, starting watchdog check");
      }
      break;
    case CheckupState::START_WATCHDOG:
      if (_system_data_->hardware_data_.wd_ready_) {
        checkup_state_ = CheckupState::TOGGLING_WATCHDOG;
        _watchdog_toggle_timer_.reset();
        DEBUG_PRINT("Watchdog ready, starting toggle sequence");
        break;
      }
      DigitalSender::start_watchdog();
      break;

    case CheckupState::TOGGLING_WATCHDOG:
      // Fail immediately if WD_READY goes low during toggling
      if (!_system_data_->hardware_data_.wd_ready_) {
        DEBUG_PRINT("Watchdog error: WD_READY went low during toggling phase");
        return CheckupError::ERROR;  // TODO: return specific error
      }

      DigitalSender::toggle_watchdog();

      // Toggle for the specified duration
      if (_watchdog_toggle_timer_.checkWithoutReset()) {
        checkup_state_ = CheckupState::STOP_TOGGLING_WATCHDOG;
        _watchdog_test_timer_.reset();
        DEBUG_PRINT("Watchdog toggle complete, stopping watchdog");
      }
      break;

    case CheckupState::STOP_TOGGLING_WATCHDOG:
      // Just reset the timer and transition to the check state
      _watchdog_test_timer_.reset();
      checkup_state_ = CheckupState::CHECK_WATCHDOG;
      DEBUG_PRINT("Stopping watchdog toggle, beginning verification");
      break;

    case CheckupState::CHECK_WATCHDOG:
      // Check if WD_READY goes low during this state
      if (!_system_data_->hardware_data_.wd_ready_) {
        _watchdog_toggle_in_timer_ = true;
        DigitalSender::close_watchdog_sdc();
        checkup_state_ = CheckupState::WAIT_FOR_ASATS;
        DEBUG_PRINT("Watchdog no longer ready - verification successful");
      }
      if (_watchdog_test_timer_.checkWithoutReset()) {
        DEBUG_PRINT("Watchdog test failed - WD_READY did not go low during verification time");
        return CheckupError::ERROR;
      }

      break;
    case CheckupState::WAIT_FOR_ASATS:

      if (_system_data_->hardware_data_.asats_pressed_) {
        _system_data_->failure_detection_.emergency_signal_ = false;
        checkup_state_ = CheckupState::CLOSE_SDC;
      }
      break;
    case CheckupState::CLOSE_SDC:
      DigitalSender::close_sdc();
      checkup_state_ = CheckupState::WAIT_FOR_TS;

      break;
    case CheckupState::WAIT_FOR_TS:
      if (_system_data_->failure_detection_.ts_on_) {
        DEBUG_PRINT("TS activated");

        checkup_state_ = CheckupState::EBS_CHECKS;
      }
      break;
    case CheckupState::EBS_CHECKS:
      handle_ebs_check();
      break;
    case CheckupState::CHECK_TIMESTAMPS: {
      // Check if all components have responded and no emergency signal has been sent
      if (_system_data_->failure_detection_.has_any_component_timed_out() ||
          _system_data_->failure_detection_.emergency_signal_) {
        return CheckupError::ERROR;
      }
      checkup_state_ = CheckupState::CHECKUP_COMPLETE;
      DEBUG_PRINT("Checkup complete and returning success");
      return CheckupError::SUCCESS;
    }
    default:
      break;
  }
  return CheckupError::WAITING_FOR_RESPONSE;
}

inline void CheckupManager::handle_ebs_check() {
  switch (pressure_test_phase_) {
    case 0:  // activate
      DigitalSender::activate_ebs();
      pressure_test_phase_++;
      break;
    case 1:  // check and deactivate
      if (check_pressure_high()) {
        DigitalSender::deactivate_ebs();
        pressure_test_phase_++;
      }
      break;
    case 2:  // check and activate
      if (check_pressure_low()) {
        pressure_test_phase_ = 0;
        DigitalSender::activate_ebs();  // might not be necessary
        checkup_state_ = CheckupState::CHECK_TIMESTAMPS;
      }
      break;
  }
}

inline bool CheckupManager::should_go_ready_from_off() const {
  if (!_system_data_->hardware_data_.asms_on_ || !_system_data_->failure_detection_.ts_on_ ||
      !_system_data_->hardware_data_.asats_pressed_ ||
      _system_data_->hardware_data_.bspd_sdc_open_) {
    return false;
  }
  _system_data_->r2d_logics_.enter_ready_state();
  return true;
}

inline bool CheckupManager::should_stay_ready() const {
  if (!_system_data_->r2d_logics_.r2d) {
    return true;
  }
  _system_data_->r2d_logics_.enter_driving_state();
  return false;
}

inline bool CheckupManager::should_enter_emergency(State current_state) const {
  if (current_state == State::AS_READY) {
    return should_enter_emergency_in_ready_state();
  } else if (current_state == State::AS_DRIVING) {
    return should_enter_emergency_in_driving_state();
  }
  return false;
}

bool CheckupManager::should_enter_emergency_in_ready_state() const {
  return _system_data_->failure_detection_.emergency_signal_ ||
         failed_to_build_pneumatic_pressure_in_engage_time() ||
         _system_data_->failure_detection_.has_any_component_timed_out() ||
         !_system_data_->hardware_data_.asms_on_ || !_system_data_->failure_detection_.ts_on_ ||
         failed_to_build_hydraulic_pressure_in_time() ||
         _system_data_->hardware_data_.bspd_sdc_open_;
}

bool CheckupManager::should_enter_emergency_in_driving_state() const {
  return _system_data_->failure_detection_.has_any_component_timed_out() ||
         _system_data_->failure_detection_.emergency_signal_ ||
         _system_data_->hardware_data_.bspd_sdc_open_ ||
         failed_to_build_pneumatic_pressure_in_release_time() ||
         failed_to_reduce_hydraulic_pressure_in_time() || !_system_data_->hardware_data_.asms_on_ ||
         !_system_data_->failure_detection_.ts_on_;
}

inline bool CheckupManager::check_pressure_high() const {
  return _system_data_->hardware_data_._hydraulic_line_pressure >= HYDRAULIC_BRAKE_THRESHOLD &&
         _system_data_->hardware_data_.pneumatic_line_pressure_;
}

inline bool CheckupManager::check_pressure_low() const {
  return _system_data_->hardware_data_._hydraulic_line_pressure < HYDRAULIC_BRAKE_THRESHOLD &&
         _system_data_->hardware_data_.pneumatic_line_pressure_;
}

bool CheckupManager::failed_to_build_pneumatic_pressure_in_engage_time() const {
  return _system_data_->hardware_data_.pneumatic_line_pressure_ == 0 &&
         _system_data_->r2d_logics_.engageEbsTimestamp.checkWithoutReset();
}
bool CheckupManager::failed_to_build_pneumatic_pressure_in_release_time() const {
  return _system_data_->hardware_data_.pneumatic_line_pressure_ == 0 &&
         _system_data_->r2d_logics_.releaseEbsTimestamp.checkWithoutReset();
}
bool CheckupManager::failed_to_build_hydraulic_pressure_in_time() const {
  return _system_data_->hardware_data_._hydraulic_line_pressure < HYDRAULIC_BRAKE_THRESHOLD &&
         _system_data_->r2d_logics_.engageEbsTimestamp.checkWithoutReset();
}

bool CheckupManager::failed_to_reduce_hydraulic_pressure_in_time() const {
  return _system_data_->hardware_data_._hydraulic_line_pressure >= HYDRAULIC_BRAKE_THRESHOLD &&
         _system_data_->r2d_logics_.releaseEbsTimestamp.checkWithoutReset();
}

inline bool CheckupManager::should_stay_driving() const {
  if (abs(_system_data_->hardware_data_._left_wheel_rpm) < 0.1 &&
      abs(_system_data_->hardware_data_._right_wheel_rpm) < 0.1 &&
      _system_data_->mission_finished_) {
    return false;
  }
  return true;
}

inline bool CheckupManager::should_stay_mission_finished() const {
  if (_system_data_->hardware_data_.asms_on_) {
    return true;
  }
  return false;
}

inline bool CheckupManager::emergency_sequence_complete() const {
  if (!_system_data_->hardware_data_.asms_on_ && _ebs_sound_timestamp_.checkWithoutReset()) {
    return true;
  }
  return false;
}

inline bool CheckupManager::res_triggered() const {
  if (_system_data_->failure_detection_.emergency_signal_) {
    return true;
  }
  return false;
}

inline bool CheckupManager::should_toggle_watchdog() { return _watchdog_toggle_in_timer_; }
