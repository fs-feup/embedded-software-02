#pragma once
#include <Arduino.h>
#include "logic_handler.hpp"
#include "data_struct.hpp"
#include "can_comm_handler.hpp"
#include "hw_io_manager.hpp"



enum class State {
    IDLE,
    DRIVING,
    AS_DRIVING
};

class StateMachine {
public:
    StateMachine() : current_state_(State::IDLE) {}

    void transition_to(State new_state) {
        current_state_ = new_state;
    }

    void update() {
        switch (current_state_) {
            case State::IDLE:
                handle_idle();
                break;
            case State::DRIVING:
                handle_driving();
                break;
            case State::AS_DRIVING:
                handle_as_driving();
                break;
        }
    }

private:
    CanCommHandler& can_handler;
    LogicHandler& logic_handler;
    IOManager& io_manager;
    
    State current_state_;

    void handle_idle() {
    }

    void handle_driving() {
    }

    void handle_as_driving() {
    }
};