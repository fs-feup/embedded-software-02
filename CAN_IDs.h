#ifndef CAN_IDS_H
#define CAN_IDS_H

#include <cstdint>


constexpr uint8_t MASTER_AS_STATE_CODE = 0x31;

constexpr uint8_t MASTER_AS_MISSION_CODE = 0x32;

constexpr uint8_t TEENSY_C1_RR_RPM_CODE = 0x11;

constexpr uint8_t TEENSY_C1_RL_RPM_CODE = 0x12;

constexpr uint8_t ALIVE_MESSAGE = 0x41;

/**
 * value of msg[0] for mission finished
*/
constexpr uint8_t MISSION_FINISHED_CODE = 0x42;

/**
 * value of msg[0] for emergency detected by AS CU
*/
constexpr uint8_t EMERGENCY_CODE = 0x43;

/**
 * Payload of the single byte message to reset the steering angle sensor
*/
constexpr uint8_t SET_ORIGIN_BOSCH_STEERING_ANGLE_RESET = 0x05;

/**
 * Payload of the single byte message to set the steering angle sensor origin
*/
constexpr uint8_t SET_ORIGIN_BOSCH_STEERING_ANGLE_SET = 0x03;

/**
 * Message code for Bamocar battery voltage
*/
constexpr uint8_t BAMOCAR_BATTERY_VOLTAGE_CODE = 0xEB;

/**
 * Message code for Bamocar motor speed
*/
constexpr uint8_t BAMOCAR_MOTOR_SPEED_CODE = 0x30;

/*
 * Message code that publishes commands to Bamocar
 */
constexpr uint8_t TORQUE_COMMAND_BAMO_BYTE = 0x90;

constexpr uint16_t MASTER_STATUS = 0x300;

constexpr uint16_t TEENSY_C1 = 0x123;

constexpr uint16_t HYDRAULIC_LINE = 0x90;

constexpr uint16_t ASMS = 0x91;

constexpr uint16_t SET_ORIGIN_CUBEM_ID = 0x55D;

constexpr uint16_t STEERING_COMMAND_CUBEM_ID = 0x45D;

constexpr uint16_t STEERING_CUBEM_ID = 0x295D;

constexpr uint16_t STEERING_BOSCH_ID = 0xA1;

constexpr uint16_t SET_ORIGIN_BOSCH_STEERING_ANGLE_ID = 0x725;


constexpr uint16_t AS_CU_NODE_ID = 0x400;

constexpr uint16_t BAMO_COMMAND_ID = 0x201;

/**
 * ID used for accelerations
 * from Bosch IMU
 */
constexpr uint16_t IMU_ACC = 0x175;

/**
 * ID used for gyroscope data
 * from Bosch IMU
 */
constexpr uint16_t IMU_GYRO = 0x179;

// -------------- SENSORS CONSTANTS --------------

/*
 * Quantization for the acceleration
 * m/s²/digit
 */
constexpr double QUANTIZATION_ACC = 0.0019616;

/*
 * Quantization for the gyro
 * rad/s/digit
 */
constexpr double QUANTIZATION_GYRO = 0.01;

/**
 * IMU Acceleration Ranges
 */
constexpr double IMU_ACC_MIN_RANGE = -48.9988064;
constexpr double IMU_ACC_MAX_RANGE = 48.9988064;

/**
 * IMU Gyroscope Ranges
 */
constexpr double IMU_GYRO_MIN_RANGE = -300.0;
constexpr double IMU_GYRO_MAX_RANGE = 300.0;

// -------------- SAFETY CONSTANTS --------------

/**
 * Checksum for steering angle sensor
*/
constexpr uint8_t BOSCH_SA_INITIAL_CRC = 0xFF;
constexpr uint8_t BOSCH_SA_CRC_POLYNOMIAL = 0x2F;

/**
 * Checksum for IMU
 */
// constexpr uint8_t CRC8_SAE_J1850_INITIAL_CRC = 0xFF;
// constexpr uint8_t CRC8_SAE_J1850_POLYNOMIAL = 0x1D;

/**
 * Limits for the throttle and steering angle
*/
// constexpr double THROTTLE_UPPER_LIMIT = 1.0; // Input Limits
// constexpr double THROTTLE_LOWER_LIMIT = -1.0; // Input Limits
// constexpr uint16_t BAMOCAR_MAX_RPM = 6500;
// constexpr uint16_t BAMOCAR_MAX_CURRENT = 73;
// constexpr uint16_t BAMOCAR_MAX_VOLTAGE = 600;
// constexpr uint16_t BAMOCAR_MAX_SCALE = 32767; // Max of the messages from the bamocar
// constexpr uint16_t MAX_ACCUMULATOR_CHARGING_CURRENT = 30; // In Amps, as per documentation

// constexpr double STEERING_UPPER_LIMIT = 0.392699; // Input Limits
// constexpr double STEERING_LOWER_LIMIT = -0.392699; // Input Limits
// constexpr uint8_t STEERING_UPPER_LIMIT_HEX_CHAR = 0x11; // Limit for buffer[1] value
// constexpr uint8_t STEERING_LOWER_LIMIT_HEX_CHAR = 0xEE; // Limit for buffer[1] value

constexpr uint16_t BMS_ID = 0x666;
constexpr uint16_t BAMO_RESPONSE_ID = 0x181;

constexpr uint16_t BMS_ID_CCL = 0x101;
constexpr uint16_t BMS_ID_ERR = 0x123;
constexpr uint32_t CH_ID = 0x02207446;
constexpr uint16_t TA_ID = 0x301;

// IDS
constexpr uint16_t MASTER_ID = 0x300;
constexpr uint16_t DASH_ID = 0x132;
constexpr uint16_t AS_CU_ID = 0x400;
constexpr uint16_t STEERING_ID = 0x295D;

// PC
constexpr uint8_t AS_CU_EMERGENCY_SIGNAL = 0x43;
constexpr uint8_t MISSION_FINISHED = 0x42;
constexpr uint8_t PC_ALIVE = 0x41;

// Sensors
constexpr uint8_t RIGHT_WHEEL_CODE = 0x11;
constexpr uint8_t LEFT_WHEEL_CODE = 0x12;
constexpr uint16_t HYDRAULIC_BRAKE_THRESHOLD = 165;

// Logging Status
constexpr uint16_t DRIVING_STATE = 0x500;
constexpr uint16_t DRIVING_CONTROL = 0x501;
constexpr uint16_t SYSTEM_STATUS = 0x502;

// RES
constexpr uint16_t NODE_ID = 0x011; // Competition Defined
constexpr uint16_t RES_STATE = (0x180 + NODE_ID);
constexpr uint16_t RES_READY = (0x700 + NODE_ID);
constexpr uint16_t RES_ACTIVATE = 0x000;

// Master State
constexpr uint8_t STATE_MSG = 0x31;
constexpr uint8_t MISSION_MSG = 0x32;
constexpr uint8_t LEFT_WHEEL_MSG = 0x33;

// Bamocar
constexpr uint8_t BTB_READY = 0xE2;
constexpr uint8_t VDC_BUS = 0xEB;
constexpr uint16_t DC_THRESHOLD = 1890; // same as 60 volts

constexpr float CAN_TIMEOUT_MS = 100;
#endif // CAN_IDS_H