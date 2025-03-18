#pragma once

#include <embedded/digitalSender.hpp>
#include <logic/checkupManager.hpp>
#include <model/structure.hpp>

#include "TeensyTimerTool.h"
using namespace TeensyTimerTool;

/**
 * @brief The ASState class manages and transitions between different states of the vehicle system.
 *
 * The ASState class uses the CheckupManager to perform system checks and determine the appropriate
 * state transitions based on the vehicle's status and operational conditions. It interacts with
 * hardware components via the OutputCoordinator and Communicator interfaces to update the system
 * state and handle transitions.
 */
class ASState {
private:
  PeriodicTimer emergency_timer_;
  volatile bool timer_has_started = false;
  OutputCoordinator *
      _output_coordinator_;  ///< Pointer to the OutputCoordinator object for hardware interactions.
  Communicator
      *_communicator_;  ///< Pointer to the Communicator object for communication operations.

  inline static ASState *instance = nullptr;

public:
  CheckupManager
      _checkup_manager_;        ///< CheckupManager object for handling various checkup operations.
  State state_{State::AS_OFF};  ///< Current state of the vehicle system, initialized to OFF.

  /**
   * @brief Constructor for the ASState class.
   * @param system_data Pointer to the SystemData object containing system status and sensor
   * information.
   * @param communicator Pointer to the Communicator object.
   * @param output_coordinator Pointer to the OutputCoordinator object.
   */
  explicit ASState(SystemData *system_data, Communicator *communicator,
                   OutputCoordinator *output_coordinator)
      : _output_coordinator_(output_coordinator),
        _communicator_(communicator),
        _checkup_manager_(system_data) {
    instance = this;
  }

  /**
   * @brief Calculates the state of the vehicle.
   */
  void calculate_state();
  void timer_started() { timer_has_started = true; }
};

inline void ASState::calculate_state() {
  switch (state_) {
    case State::AS_MANUAL:
      if (_checkup_manager_.should_stay_manual_driving()) {
        break;
      }

      DEBUG_PRINT("Entering OFF state from MANUAL");
      _output_coordinator_->enter_off_state();

      state_ = State::AS_OFF;
      break;

    case State::AS_OFF:

      if (!timer_has_started) {
        emergency_timer_.begin(
            [] {
              instance->timer_started();
              if (instance->_checkup_manager_.should_enter_emergency(instance->state_)) {
                instance->_output_coordinator_->enter_emergency_state();
                instance->_checkup_manager_._ebs_sound_timestamp_.reset();
                instance->state_ = State::AS_EMERGENCY;
              }
            },
            150'000);
      }
      // If manual driving checkup fails, the car can't be in OFF state, so it goes back to MANUAL
      if (_checkup_manager_.should_stay_manual_driving()) {
        DEBUG_PRINT("Entering MANUAL state from OFF");
        _output_coordinator_->enter_manual_state();
        state_ = State::AS_MANUAL;
        break;
      }

      if (_checkup_manager_.should_stay_off()) break;
      if (!_checkup_manager_.should_go_ready_from_off()) break;  // recheck all states

      DEBUG_PRINT("Entering READY state from OFF");
      _output_coordinator_->enter_ready_state();
      state_ = State::AS_READY;
      DEBUG_PRINT("READY state entered...");
      break;

    case State::AS_READY:

      if (_checkup_manager_.should_stay_ready()) {
        break;
      }
      _output_coordinator_->enter_driving_state();
      state_ = State::AS_DRIVING;
      break;

    case State::AS_DRIVING:
      _output_coordinator_->blink_driving_led();
      if (_checkup_manager_.should_stay_driving()) break;
      _output_coordinator_->enter_finish_state();
      state_ = State::AS_FINISHED;
      break;

    case State::AS_FINISHED:
      if (_checkup_manager_.res_triggered()) {
        DEBUG_PRINT("Entering EMERGENCY state from FINISHED");

        _output_coordinator_->enter_emergency_state();
        _checkup_manager_._ebs_sound_timestamp_.reset();
        state_ = State::AS_EMERGENCY;
        break;
      }
      if (_checkup_manager_.should_stay_mission_finished()) break;

      DEBUG_PRINT("Entering OFF state from FINISHED");
      _output_coordinator_->enter_off_state();
      _checkup_manager_.reset_checkup_state();
      state_ = State::AS_OFF;
      break;

    case State::AS_EMERGENCY:
      _output_coordinator_->blink_emergency_led();

      if (_checkup_manager_.emergency_sequence_complete()) {
        DEBUG_PRINT("Entering OFF state from EMERGENCY");
        _output_coordinator_->enter_off_state();
        _checkup_manager_.reset_checkup_state();
        state_ = State::AS_OFF;
        break;
      }
      break;
    default:
      break;
  }
}
