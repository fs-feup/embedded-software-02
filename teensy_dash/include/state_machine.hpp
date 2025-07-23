#pragma once

#include "can_comm_handler.hpp"
#include "hw_io_manager.hpp"
#include "logic_handler.hpp"

class StateMachine {
public:
  StateMachine(CanCommHandler& can_handler, LogicHandler& logic_handler, IOManager& io_manager);
  void update();
  State get_state() const;

private:
  CanCommHandler& can_handler;
  LogicHandler& logic_handler;
  IOManager& io_manager;
  State current_state_ = State::IDLE;
  [[nodiscard]] bool transition_to_driving() const;
  void transition_to_idle();
  void handle_driving();
};
