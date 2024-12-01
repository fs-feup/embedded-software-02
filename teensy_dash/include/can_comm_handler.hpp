#pragma once
#include <Arduino.h>
#include "data_struct.hpp"
#include <cstdint>

class CanCommHandler {
public:
    CanCommHandler();

    void initialize();
    void sendMessage(const uint8_t* data, uint8_t length);
    bool receiveMessage(uint8_t* buffer, uint8_t& length);

private:
    SystemData& data;
};

