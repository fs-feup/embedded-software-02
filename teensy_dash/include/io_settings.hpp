#pragma once
#include <cstdint>

namespace pins {
    namespace analog {
        constexpr uint8_t APPS_1 = 22;
        constexpr uint8_t APPS_2 = 20;
        constexpr uint8_t BRAKE_PRESSURE = 19;
        constexpr uint8_t ROTARY_SWITCH = 23;
    }

    namespace digital {
        constexpr uint8_t TS = 21;
        constexpr uint8_t BSPD = 7;
        constexpr uint8_t INERTIA = 8;
        constexpr uint8_t R2D = 4;
        constexpr uint8_t ATS = 5;
        constexpr uint8_t ATS_OUT = 9;
    }

    namespace spi {
        constexpr uint8_t CS = 10;
        constexpr uint8_t MOSI = 11;
        constexpr uint8_t MISO = 12;
        constexpr uint8_t SCK = 13;
    }

    namespace output {
        constexpr uint8_t RACE_LED = 14;
        constexpr uint8_t BUZZER = 2;
        constexpr uint8_t DISPLAY_MODE = 3;
        constexpr uint8_t TS_LED = 13;
        constexpr uint8_t BSPD_LED = 15;
        constexpr uint8_t INERTIA_LED = 16;
    }

    namespace encoder {
        constexpr uint8_t FRONT_RIGHT_WHEEL = 12;
        constexpr uint8_t FRONT_LEFT_WHEEL = 11;
    }
}

namespace config {
    namespace adc {
        constexpr int MAX_VALUE = 1024;
        constexpr int NUM_MODES = 8;
        constexpr int SEGMENT_SIZE = MAX_VALUE / NUM_MODES;
        constexpr int HYSTERESIS = 10;
    }

    namespace apps {
        constexpr uint16_t UPPER_BOUND_1 = 800;
        constexpr uint16_t LOWER_BOUND_1 = 240;
        constexpr uint16_t UPPER_BOUND_2 = 785;
        constexpr uint16_t LOWER_BOUND_2 = 220;
        
        constexpr uint16_t DEAD_THRESHOLD_1 = 780;
        constexpr uint16_t DEAD_THRESHOLD_2 = 240;
        
        constexpr uint16_t DEADZONE_EQUIVALENT_1 = 660;
        constexpr uint16_t DEADZONE_EQUIVALENT_2 = 360;
        
        constexpr uint16_t LINEAR_OFFSET = 124;
        constexpr uint16_t MAX = 770;
        constexpr uint16_t MIN = 270;
        
        constexpr float MAX_ERROR_PERCENT = 0.15f;
        constexpr uint16_t MAX_ERROR_ABS = UPPER_BOUND_1 * MAX_ERROR_PERCENT;
        
        constexpr uint8_t SAMPLES = 5;
        constexpr uint16_t BRAKE_BLOCK_THRESHOLD = 250;
        constexpr uint32_t IMPLAUSIBLE_TIMEOUT_MS = 200;
        constexpr uint32_t BRAKE_PLAUSIBILITY_TIMEOUT_MS = 500;
    }

    namespace brake {
        constexpr uint16_t BLOCK_THRESHOLD = 250;
    }

    namespace wheel {
        constexpr uint32_t LIMIT_RPM_INTERVAL = 500000;
        constexpr uint8_t PULSES_PER_ROTATION = 36;
    }

    namespace r2d {
        constexpr uint32_t TIMEOUT_MS = 15000;
    }

    namespace bamocar {
        constexpr uint16_t MAX = 32760;
        constexpr uint16_t MIN = 0;
    }
}