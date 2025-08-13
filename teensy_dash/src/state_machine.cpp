#include "state_machine.hpp"

#include <io_settings.hpp>
elapsedMillis print_state_timer;
StateMachine::StateMachine(CanCommHandler& can_handler, LogicHandler& logic_handler,
                           IOManager& io_manager)
    : can_handler(can_handler), logic_handler(logic_handler), io_manager(io_manager) {}

void StateMachine::update() {
  int torque_from_apps = 0;
  switch (current_state_) {
    case State::IDLE:
      if (logic_handler.should_start_manual_driving()) {
        can_handler.reset_bamocar_init();
        current_state_ = State::INITIALIZING_DRIVING;
        DEBUG_PRINTLN("Starting manual driving");
        io_manager.play_r2d_sound();  // tapem os ouvidos!
      } else if (logic_handler.should_start_as_driving()) {
        can_handler.reset_bamocar_init();
        current_state_ = State::INITIALIZING_AS_DRIVING;
      } else {
        // Serial.println("chillin");
      }
      break;
    case State::INITIALIZING_DRIVING:
      if (transition_to_driving()) {
        DEBUG_PRINTLN("Transitioning to driving state");
        current_state_ = State::DRIVING;
      }
      break;  // wait for transition to finish
    case State::INITIALIZING_AS_DRIVING:
      if (transition_to_driving()) {
        current_state_ = State::AS_DRIVING;
      }
      break;  // wait for transition to finish
    case State::DRIVING:
      torque_from_apps = logic_handler.calculate_torque();
      if (logic_handler.should_go_idle()) {
        DEBUG_PRINTLN("Going idle from driving state");
        DEBUG_PRINTLN("Going idle from driving state");
        DEBUG_PRINTLN("Going idle from driving state");
        DEBUG_PRINTLN("Going idle from driving state");

        transition_to_idle();
        return;
      }
      if (torque_from_apps == config::apps::ERROR_PLAUSIBILITY) {
        DEBUG_PRINTLN("Torque implausible, sending 0 torque");
        DEBUG_PRINTLN("Torque implausible, sending 0 torque");
        DEBUG_PRINTLN("Torque implausible, sending 0 torque");
        torque_from_apps = 0;
      }
      if (torque_from_apps >= 0 && torque_from_apps <= config::bamocar::MAX) {
        can_handler.send_torque(
            torque_from_apps); /* VVVVVRRRRRRRRRRUUUUUUMMMMMMMMMMMMMMMMMMMMMMMm*/
      }
      break;
    case State::AS_DRIVING:

      if (logic_handler.just_entered_driving()) {
        DEBUG_PRINTLN("Going idle from AS driving state");
        DEBUG_PRINTLN("Going idle from AS driving state");
        DEBUG_PRINTLN("Going idle from AS driving state");
        DEBUG_PRINTLN("Going idle from AS driving state");

        io_manager.play_r2d_sound();
      }

      if (logic_handler.just_entered_emergency()) {
        DEBUG_PRINTLN("Going idle from AS driving state");
        DEBUG_PRINTLN("Going idle from AS driving state");
        DEBUG_PRINTLN("Going idle from AS driving state");
        DEBUG_PRINTLN("Going idle from AS driving state");

        transition_to_idle();
        io_manager.play_emergency_buzzer();
      }
      if (logic_handler.should_go_idle()) {
        DEBUG_PRINTLN("Going idle from AS driving state");
        DEBUG_PRINTLN("Going idle from AS driving state");
        DEBUG_PRINTLN("Going idle from AS driving state");
        DEBUG_PRINTLN("Going idle from AS driving state");
        transition_to_idle();
      }
      break;
    default:
      break;
  }
  if (print_state_timer >= 700) {
    // DEBUG_PRINTLN("Current state: " + String(static_cast<int>(current_state_)));
    // DEBUG_PRINTLN("Current torque: " + String(torque_from_apps));
    print_state_timer = 0;
  }
}

bool StateMachine::transition_to_driving() const {
  const bool done = can_handler.init_bamocar();
  return done;
}

State StateMachine::get_state() const { return this->current_state_; }

void StateMachine::transition_to_idle() {
  if (current_state_ == State::DRIVING || current_state_ == State::AS_DRIVING) {
    can_handler.stop_bamocar();
    can_handler.reset_bamocar_init();
    current_state_ = State::IDLE;
  }
}
