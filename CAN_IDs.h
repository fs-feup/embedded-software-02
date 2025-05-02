#pragma once
#include <cstdint>

//-----------------------------------------------------------------------------
// System Constants
//-----------------------------------------------------------------------------
constexpr unsigned CAN_TIMEOUT_MS = 100;
constexpr uint8_t RPM_MSG_PERIOD_MS = 50;     // 20Hz
constexpr uint8_t APPS_MSG_PERIOD_MS = 50;    // 20Hz
constexpr uint8_t TORQUE_MSG_PERIOD_MS = 10;  // 100Hz
constexpr float WHEEL_PRECISION = 1e-2;

//-----------------------------------------------------------------------------
// CAN Message IDs
//-----------------------------------------------------------------------------
constexpr uint16_t MASTER_ID = 0x300;
constexpr uint16_t DASH_ID = 0x132;
constexpr uint16_t AS_CU_ID = 0x400;
constexpr uint16_t TA_ID = 0x301;
constexpr uint32_t CH_ID = 0x0220'7446;

//-----------------------------------------------------------------------------
// Status & Control Message Types
//-----------------------------------------------------------------------------
constexpr uint8_t PC_ALIVE = 0x41;
constexpr uint8_t MISSION_FINISHED = 0x42;
constexpr uint8_t AS_READY = 2;
constexpr uint8_t AS_OFF = 0;
constexpr uint8_t AS_DRIVING = 3;
constexpr uint8_t AS_EMERGENCY = 4;
constexpr uint8_t AS_FINISHED = 5;
constexpr uint8_t AS_CU_EMERGENCY_SIGNAL = 0x43;
constexpr uint8_t STATE_MSG = 0x31;
constexpr uint8_t MISSION_MSG = 0x32;
constexpr uint8_t LEFT_WHEEL_MSG = 0x33;
constexpr uint8_t DBG_LOG_MSG = 0x34;
constexpr uint8_t DBG_LOG_MSG_2 = 0x35;
constexpr uint8_t FR_RPM = 0x10;
constexpr uint8_t FL_RPM = 0x11;
constexpr uint8_t APPS_HIGHER = 0x20;
constexpr uint8_t APPS_LOWER = 0x21;

//-----------------------------------------------------------------------------
// Logging Status IDs
//-----------------------------------------------------------------------------
constexpr uint16_t DRIVING_STATE = 0x500;
constexpr uint16_t DRIVING_CONTROL = 0x501;
constexpr uint16_t SYSTEM_STATUS = 0x502;
constexpr uint16_t SOC_MSG = 0x200;

//-----------------------------------------------------------------------------
// Steering System
//-----------------------------------------------------------------------------
/** ID for steering angle messages from Bosch sensor */
constexpr uint16_t STEERING_BOSCH_ID = 0xA1;

/** ID for setting origin on Bosch steering angle sensor */
constexpr uint16_t SET_ORIGIN_BOSCH_STEERING_ANGLE_ID = 0x725;

/** ID used for steering commands to CUBE-M controller */
constexpr uint16_t STEERING_COMMAND_CUBEM_ID = 0x45D;

/** ID used for setting origin on CUBE-M controller */
constexpr uint16_t SET_ORIGIN_CUBEM_ID = 0x55D;

/** ID for steering angle messages (standardized) */
constexpr uint16_t STEERING_ID = 0x295D;

/** Payload to reset steering angle sensor to zero */
constexpr uint8_t SET_ORIGIN_BOSCH_STEERING_ANGLE_RESET = 0x05;

/** Payload to set steering angle sensor origin */
constexpr uint8_t SET_ORIGIN_BOSCH_STEERING_ANGLE_SET = 0x03;

//-----------------------------------------------------------------------------
// Bamocar Motor Controller
//-----------------------------------------------------------------------------
/** ID for sending commands to Bamocar */
constexpr uint16_t BAMO_COMMAND_ID = 0x201;

/** ID for receiving responses from Bamocar */
constexpr uint16_t BAMO_RESPONSE_ID = 0x181;

/** Command code for torque commands to Bamocar */
constexpr uint8_t TORQUE_COMMAND_BAMO_BYTE = 0x90;

/** Message code for battery voltage */
constexpr uint8_t BAMOCAR_BATTERY_VOLTAGE_CODE = 0xEB;

/** Message code for motor speed */
constexpr uint8_t BAMOCAR_MOTOR_SPEED_CODE = 0x30;

/** BTB ready status code */
constexpr uint8_t BTB_READY = 0xE2;

/** DC bus voltage threshold (60V) */
constexpr uint16_t DC_THRESHOLD = 1890;

/** Also used for ASMS_ON */
constexpr uint16_t ASMS = 0x91;
constexpr uint16_t ASMS_ON = 0x201;

//-----------------------------------------------------------------------------
// Sensors
//-----------------------------------------------------------------------------
/** IMU acceleration data ID */
constexpr uint16_t IMU_ACC = 0x175;

/** IMU gyroscope data ID */
constexpr uint16_t IMU_GYRO = 0x179;

/** Right wheel speed sensor code */
constexpr uint8_t RIGHT_WHEEL_CODE = 0x11;

/** Left wheel speed sensor code */
constexpr uint8_t LEFT_WHEEL_CODE = 0x12;

//-----------------------------------------------------------------------------
// Hydraulic System
//-----------------------------------------------------------------------------
constexpr uint16_t HYDRAULIC_LINE = 0x90;
constexpr uint16_t HYDRAULIC_BRAKE_THRESHOLD = 165;

//-----------------------------------------------------------------------------
// Battery Management System
//-----------------------------------------------------------------------------
constexpr uint16_t BMS_ID = 0x666;
constexpr uint16_t BMS_ID_CCL = 0x101;
constexpr uint16_t BMS_ID_ERR = 0x123;

//-----------------------------------------------------------------------------
// Rules Enforcement System (RES)
//-----------------------------------------------------------------------------
constexpr uint16_t NODE_ID = 0x011;  // Competition Defined
constexpr uint16_t RES_STATE = (0x180 + NODE_ID);
constexpr uint16_t RES_READY = (0x700 + NODE_ID);
constexpr uint16_t RES_ACTIVATE = 0x000;

// Bamocar
constexpr uint8_t BTB_READY_0 = 0xE2;
constexpr uint8_t BTB_READY_1 = 0x01;
constexpr uint8_t BTB_READY_2 = 0x00;
constexpr uint8_t BTB_READY_3 = 0x00;
constexpr uint8_t ENABLE_0 = 0xE8;
constexpr uint8_t ENABLE_1 = 0x01;
constexpr uint8_t ENABLE_2 = 0x00;
constexpr uint8_t ENABLE_3 = 0x00;
constexpr std::array<uint8_t, 3> BTB_READY_SEQUENCE = {BTB_READY_1, BTB_READY_2, BTB_READY_3};
constexpr std::array<uint8_t, 3> ENABLE_SEQUENCE = {ENABLE_1, ENABLE_2, ENABLE_3};
constexpr uint8_t DC_VOLTAGE = 0xEB;
constexpr uint8_t SPEED_ACTUAL = 0x30;
constexpr uint8_t SPEED_LIMIT = 0x34;
constexpr uint8_t SPEED_DELTAMA_ACC = 0x35;
constexpr uint8_t SPEED_DELTAMA_DECC = 0xED;
constexpr uint8_t DEVICE_I_MAX = 0xC4;
constexpr uint8_t DEVICE_I_CNT = 0xC5;
