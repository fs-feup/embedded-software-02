#pragma once
#include <deque>
#include "IOsettings.hpp"

struct RPMValues {
    float fr_rpm;
    float fl_rpm;
};
void insert_value_queue(int value, std::deque<int>& queue) {
    queue.push_front(value);

    if (queue.size() > APPS_SAMPLES) {
        queue.pop_back();
    }
}