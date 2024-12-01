#pragma once
#include <Arduino.h>
#include "data_struct.hpp"

class LogicHandler {
public:
    LogicHandler();

    void initialize();
    void update();
    void reset();

private:
    SystemData& data;
};

