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
        constexpr int MAX_VALUE = 1023;
        constexpr int NEW_SCALE_MAX = 7;
        constexpr int HALF_JUMP = 73;
    }
    namespace buzzer {
        constexpr uint32_t BUZZER_FREQUENCY = 500;
    }

    namespace apps {
        constexpr uint16_t UPPER_BOUND_APPS_HIGHER = 800;
        constexpr uint16_t LOWER_BOUND_APPS_HIGHER = 240;
        constexpr uint16_t UPPER_BOUND_APPS_LOWER = 785;
        constexpr uint16_t LOWER_BOUND_APPS_LOWER = 220;
        
        constexpr uint16_t DEAD_THRESHOLD_APPS_HIGHER = 780;
        constexpr uint16_t DEAD_THRESHOLD_APPS_LOWER = 240;
        
        constexpr uint16_t APPS_HIGHER_DEADZONE_IN_APPS_LOWER_SCALE = 660;
        constexpr uint16_t APPS_LOWER_DEADZONE_IN_APPS_HIGHER_SCALE = 360;
        
        constexpr uint16_t LINEAR_OFFSET = 124;
        constexpr uint16_t MAX = 770;
        constexpr uint16_t MIN = 270;
        
        constexpr int8_t ERROR_PLAUSIBILITY = -4;

        constexpr uint8_t MAX_ERROR_PERCENT = 15;
        constexpr uint16_t MAX_ERROR_ABS = UPPER_BOUND_APPS_HIGHER * MAX_ERROR_PERCENT/100;
        
        constexpr uint8_t SAMPLES = 5;
        constexpr uint16_t BRAKE_BLOCK_THRESHOLD = 250;
        constexpr uint32_t IMPLAUSIBLE_TIMEOUT_MS = 200;
        constexpr uint32_t BRAKE_PLAUSIBILITY_TIMEOUT_MS = 500;
    }

    namespace brake {
        constexpr uint16_t BLOCK_THRESHOLD = 250;
        constexpr uint16_t PRESSURE_THRESHOLD = 250;
    }

    namespace wheel {
        constexpr uint32_t LIMIT_RPM_INTERVAL = 500'000;
        constexpr uint8_t PULSES_PER_ROTATION = 36;
    }

    namespace r2d {
        constexpr uint32_t TIMEOUT_MS = 15'000;
    }

    namespace bamocar {
        constexpr uint16_t MAX = 32'760;
        constexpr uint16_t MIN = 0;
    }
}