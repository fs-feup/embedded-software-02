#pragma once
#include <Arduino.h>

#include "can_comm_handler.hpp"
#include "data_struct.hpp"
#include "hw_io_manager.hpp"
#include "logic_handler.hpp"

enum class State { IDLE, DRIVING, AS_DRIVING };

class StateMachine {
public:
  StateMachine(CanCommHandler& can_handler, LogicHandler& logic_handler, IOManager& io_manager)
      : can_handler(can_handler), logic_handler(logic_handler), io_manager(io_manager) {}

  void update();

private:
  CanCommHandler& can_handler;
  LogicHandler& logic_handler;
  IOManager& io_manager;

  State current_state_;

  void handle_idle();

  void transition_to_driving();

  void transition_to_as_driving();

  void handle_driving();
};

void StateMachine::update() {
  switch (current_state_) {
    case State::IDLE:
      handle_idle();
      break;
    case State::DRIVING:
      handle_driving();
      break;
    case State::AS_DRIVING:
      handle_driving();
      break;
  }
}

void StateMachine::handle_idle() {
  if (logic_handler.should_start_manual_driving()) {
    transition_to_driving();
    current_state_ = State::DRIVING;
  } else if (logic_handler.should_start_as_driving()) {
    transition_to_as_driving();
    current_state_ = State::AS_DRIVING;
  }
}

void StateMachine::transition_to_driving() {
  io_manager.play_r2d_sound();
  can_handler.init_bamocar();
}

void StateMachine::transition_to_as_driving() {
  // delay?
  can_handler.init_bamocar();
}

void StateMachine::handle_driving() {
  if (logic_handler.should_go_idle()) {
    current_state_ = State::IDLE;
    can_handler.stop_bamocar();
  }
  // TODO(PedroRomao3): timer needed?
  if (current_state_ == State::DRIVING) {
    int torque_from_apps = logic_handler.calculate_torque();
    can_handler.send_torque(torque_from_apps);
  }
}