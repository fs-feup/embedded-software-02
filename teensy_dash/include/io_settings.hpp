#pragma once

namespace pins {
namespace analog {
constexpr uint8_t APPS_HIGHER = 20;
constexpr uint8_t APPS_LOWER = 22;
constexpr uint8_t BRAKE_PRESSURE = 19;
constexpr uint8_t ROTARY_SWITCH = 23;
}  // namespace analog

namespace digital {
constexpr uint8_t TS = 21;
constexpr uint8_t BSPD = 18;
constexpr uint8_t INERTIA = 7;
constexpr uint8_t R2D = 4;
constexpr uint8_t ATS = 5;
constexpr uint8_t ATS_OUT = 6;
}  // namespace digital

namespace spi {
constexpr uint8_t CS = 10;
constexpr uint8_t MOSI = 11;
constexpr uint8_t MISO = 12;
constexpr uint8_t SCK = 13;
}  // namespace spi

namespace output {
constexpr uint8_t RACE_LED = 17;
constexpr uint8_t BUZZER = 2;
constexpr uint8_t DISPLAY_MODE = 3;
constexpr uint8_t TS_LED = 16;
constexpr uint8_t BSPD_LED = 14;
constexpr uint8_t INERTIA_LED = 15;
}  // namespace output

namespace encoder {
constexpr uint8_t FRONT_RIGHT_WHEEL = 9;
constexpr uint8_t FRONT_LEFT_WHEEL = 8;
}  // namespace encoder
}  // namespace pins

namespace config {
namespace adc {
constexpr int MAX_VALUE = 1023;
constexpr int NEW_SCALE_MAX = 7;
constexpr int HALF_JUMP = 73;
}  // namespace adc
namespace buzzer {
constexpr uint32_t BUZZER_FREQUENCY = 500;
constexpr uint8_t EMERGENCY_DURATION = 1;
}  // namespace buzzer

namespace apps {
constexpr uint16_t UPPER_BOUND_APPS_HIGHER = 700;
constexpr uint16_t LOWER_BOUND_APPS_HIGHER = 200;
constexpr uint16_t UPPER_BOUND_APPS_LOWER = 610;
constexpr uint16_t LOWER_BOUND_APPS_LOWER = 40;

constexpr uint16_t DEAD_THRESHOLD_APPS_HIGHER = 780;
constexpr uint16_t APPS_LOWER_ZEROED = 5;
constexpr uint16_t DEADBAND = 40;

constexpr uint16_t APPS_HIGHER_WHEN_LOWER_ZEROES = 210;
constexpr uint16_t APPS_LOWER_DEADZONE_IN_APPS_HIGHER_SCALE = 360;

constexpr uint16_t LINEAR_OFFSET = 140;
constexpr uint16_t HIGHER_MAX = 677; // 677 for APPS Upper 515 APPS lower
constexpr uint16_t HIGHER_MIN = 210; // 210 for APPS Lower 80 APPS lower

constexpr uint16_t LOWER_MAX = 520; // 677 for APPS Upper 515 APPS lower
constexpr uint16_t LOWER_MIN = 85; // 210 for APPS Lower 80 APPS lower


constexpr uint16_t AVG_MIN = 145;
constexpr uint16_t AVG_MAX = 596;
constexpr uint16_t MIN_FOR_TORQUE = 0;
constexpr uint16_t MAX_FOR_TORQUE = LOWER_MAX - LOWER_MIN;

constexpr int ERROR_PLAUSIBILITY = -4;

constexpr uint8_t MAX_ERROR_PERCENT = 60;
constexpr uint16_t MAX_ERROR_ABS = UPPER_BOUND_APPS_HIGHER * MAX_ERROR_PERCENT / 100;

constexpr uint8_t SAMPLES = 5;
constexpr uint16_t BRAKE_BLOCK_THRESHOLD = 210;
constexpr uint32_t IMPLAUSIBLE_TIMEOUT_MS = 100; // Time to set implausibility flag back to false
constexpr uint32_t BRAKE_PLAUSIBILITY_TIMEOUT_MS = 500;
}  // namespace apps

namespace brake {
constexpr uint16_t BLOCK_THRESHOLD = 220;
constexpr uint16_t PRESSURE_THRESHOLD = 250;
}  // namespace brake

namespace wheel {
constexpr uint32_t LIMIT_RPM_INTERVAL = 500'000;
constexpr uint8_t PULSES_PER_ROTATION = 48;
}  // namespace wheel

namespace r2d {
constexpr uint32_t TIMEOUT_MS = 1'000;
}

namespace bamocar {
constexpr uint16_t MAX = 32'760;
constexpr uint16_t MIN = 0;
}  // namespace bamocar
}  // namespace config