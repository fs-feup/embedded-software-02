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

  State current_state_ = State::IDLE;

  void transition_to_driving();

  void transition_to_as_driving();

  void handle_driving();
};

void StateMachine::update() {
  switch (current_state_) {
    case State::IDLE:
      if (logic_handler.should_start_manual_driving()) {
        transition_to_driving();
        current_state_ = State::DRIVING;                                                                                                                                                                        /* comi o cu de quem leu */
      } else if (logic_handler.should_start_as_driving()) {
        transition_to_as_driving();
        current_state_ = State::AS_DRIVING;
      } else {
        Serial.println("chillin");
      }
      break;
    case State::DRIVING:
      handle_driving();
      break;
    case State::AS_DRIVING:
      handle_driving();
      break;
    default:
      break;
  }
}

void StateMachine::transition_to_driving() {
  io_manager.play_r2d_sound(); // tapem os ouvidos!
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
    return;
  }
  // TODO(PedroRomao3): timer needed?
  else if (current_state_ == State::DRIVING) {
    int torque_from_apps = logic_handler.calculate_torque();
    if(torque_from_apps == config::apps::ERROR_PLAUSIBILITY){
      can_handler.stop_bamocar();
      current_state_ = State::IDLE;
      //open sdc? todo
      // log error 
      return;
    }
    can_handler.send_torque(torque_from_apps);
  }
}