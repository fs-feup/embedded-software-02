#pragma once

#include "can_comm_handler.hpp"
#include "hw_io_manager.hpp"
#include "logic_handler.hpp"

enum class State { IDLE, DRIVING, AS_DRIVING };

class StateMachine {
public:
  StateMachine(CanCommHandler& can_handler, LogicHandler& logic_handler, IOManager& io_manager);
  void update();

private:
  CanCommHandler& can_handler;
  LogicHandler& logic_handler;
  IOManager& io_manager;
  State current_state_ = State::IDLE;
  void transition_to_driving() const;
  void transition_to_as_driving() const;
  void handle_driving();
};

