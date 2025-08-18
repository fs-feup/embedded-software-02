#pragma once
#include <array>  // Added for std::array
#include <cstdint>

//-----------------------------------------------------------------------------
// System Constants
//-----------------------------------------------------------------------------

constexpr unsigned CAN_TIMEOUT_MS = 100;          // 100
constexpr uint8_t RPM_MSG_PERIOD_MS = 150;        // 150
constexpr uint8_t HYDRAULIC_MSG_PERIOD_MS = 165;  // 165
constexpr uint8_t APPS_MSG_PERIOD_MS = 250;       // 250
constexpr float WHEEL_PRECISION = 1e-2;           // 1e-2

//-----------------------------------------------------------------------------
// CAN Message IDs
//-----------------------------------------------------------------------------
constexpr uint16_t MASTER_ID = 0x300;      // 0x300
constexpr uint16_t DASH_ID = 0x132;        // 0x132
constexpr uint16_t AS_CU_ID = 0x400;       // 0x400
constexpr uint16_t TA_ID = 0x301;          // 0x301
constexpr uint32_t CH_ID = 0x0220'7446;    // 0x0220'7446
constexpr uint16_t HC_ID = 0x105;          // 0x105
constexpr uint16_t BMS_TX_ID = 0x101;  // 0x101
constexpr uint16_t DATA_LOGGER_SIGNALS_1 = 0x511;  // 0x511
constexpr uint16_t DATA_LOGGER_SIGNALS_2 = 0x512;  // 0x512
constexpr uint16_t DATA_LOGGER_SIGNALS_3 = 0x513;  // 0x513
constexpr uint16_t DATA_LOGGER_SIGNALS_4 = 0x514;  // 0x514
constexpr uint16_t DATA_LOGGER_SIGNALS_5 = 0x515;  // 0x515

//-----------------------------------------------------------------------------

// Status & Control Message Types
//-----------------------------------------------------------------------------
constexpr uint8_t PC_ALIVE = 0x41;          // 0x41
constexpr uint8_t MISSION_FINISHED = 0x42;  // 0x42

constexpr uint8_t AS_OFF = 0;        // 0
constexpr uint8_t AS_READY = 2;      // 2
constexpr uint8_t AS_DRIVING = 3;    // 3
constexpr uint8_t AS_FINISHED = 4;   // 4
constexpr uint8_t AS_EMERGENCY = 5;  // 5

constexpr uint8_t AS_CU_EMERGENCY_SIGNAL = 0x43;  // 0x43
constexpr uint8_t STATE_MSG = 0x31;               // 0x31
constexpr uint8_t MISSION_MSG = 0x32;             // 0x32
constexpr uint8_t LEFT_WHEEL_MSG = 0x33;          // 0x33
constexpr uint8_t DBG_LOG_MSG = 0x34;             // 0x34
constexpr uint8_t DBG_LOG_MSG_2 = 0x35;           // 0x35
constexpr uint8_t EBS_STATE_MSG = 0x36;           // 0x36
constexpr uint8_t EBS_REDUNDANCY_MSG = 0x37;      // 0x37

//-----------------------------------------------------------------------------
// Logging Status IDs
//-----------------------------------------------------------------------------
constexpr uint8_t DRIVING_STATE = 0x40;      // 0x40
constexpr uint16_t DRIVING_CONTROL = 0x501;  // 0x501
constexpr uint16_t SYSTEM_STATUS = 0x502;    // 0x502
constexpr uint16_t SOC_MSG = 0x60;           // 0x60

//-----------------------------------------------------------------------------
// Steering System
//-----------------------------------------------------------------------------
/** ID for steering angle messages from Bosch sensor */
constexpr uint16_t STEERING_BOSCH_ID = 0xA1;  // 0xA1

/** ID for setting origin on Bosch steering angle sensor */
constexpr uint16_t SET_ORIGIN_BOSCH_STEERING_ANGLE_ID = 0x725;  // 0x725

/** ID used for steering commands to CUBE-M controller */
constexpr uint16_t STEERING_COMMAND_CUBEM_ID = 0x45D;  // 0x45D

/** ID used for setting origin on CUBE-M controller */
constexpr uint16_t SET_ORIGIN_CUBEM_ID = 0x55D;  // 0x55D

/** ID for steering angle messages (standardized) */
constexpr uint16_t STEERING_ID = 0x295D;  // 0x295D

constexpr uint8_t FR_RPM = 0x10;       // 0x10
constexpr uint8_t FL_RPM = 0x11;       // 0x11
constexpr uint8_t APPS_HIGHER = 0x20;  // 0x20
constexpr uint8_t APPS_LOWER = 0x21;   // 0x21

/** Payload to reset steering angle sensor to zero */
constexpr uint8_t SET_ORIGIN_BOSCH_STEERING_ANGLE_RESET = 0x05;  // 0x05

/** Payload to set steering angle sensor origin */
constexpr uint8_t SET_ORIGIN_BOSCH_STEERING_ANGLE_SET = 0x03;  // 0x03

//-----------------------------------------------------------------------------
// Bamocar Motor Controller
//-----------------------------------------------------------------------------
/** ID for sending commands to Bamocar */
constexpr uint16_t BAMO_COMMAND_ID = 0x201;  // 0x201

/** ID for receiving responses from Bamocar */
constexpr uint16_t BAMO_RESPONSE_ID = 0x181;  // 0x181

/** Command code for torque commands to Bamocar */
constexpr uint8_t TORQUE_COMMAND_BAMO_BYTE = 0x90;  // 0x90

/** Message code for battery voltage */
constexpr uint8_t BAMOCAR_BATTERY_VOLTAGE_CODE = 0xEB;  // 0xEB

/** Message code for motor speed */
constexpr uint8_t BAMOCAR_MOTOR_SPEED_CODE = 0x30;  // 0x30

/** BTB ready status code */
constexpr uint8_t BTB_READY = 0xE2;  // 0xE2

/** DC bus voltage threshold (60V) */
constexpr uint16_t DC_THRESHOLD = 1890;  // 1890

/** Also used for ASMS_ON */
constexpr uint16_t ASMS = 0x91;  // 0x91

//-----------------------------------------------------------------------------
// Sensors
//-----------------------------------------------------------------------------
/** IMU acceleration data ID */
constexpr uint16_t IMU_ACC = 0x175;  // 0x175

/** IMU gyroscope data ID */
constexpr uint16_t IMU_GYRO = 0x179;  // 0x179

/** Right wheel speed sensor code */
constexpr uint8_t RIGHT_WHEEL_CODE = 0x11;  // 0x11

/** Left wheel speed sensor code */
constexpr uint8_t LEFT_WHEEL_CODE = 0x12;  // 0x12

//-----------------------------------------------------------------------------
// Hydraulic System
//-----------------------------------------------------------------------------
constexpr uint16_t HYDRAULIC_LINE = 0x90;            // 0x90
constexpr uint16_t HYDRAULIC_BRAKE_THRESHOLD = 165; 

//-----------------------------------------------------------------------------
// Battery Management System
//-----------------------------------------------------------------------------
constexpr uint16_t BMS_ID = 0x101;                   // 0x666
constexpr uint16_t BMS_ID_CCL = 0x101;               // 0x101
constexpr uint16_t BMS_ID_ERR = 0x123;               // 0x123
constexpr uint32_t BMS_THERMISTOR_ID = 0x1839'F380;  // 0x1839'F380

constexpr uint16_t BMS_DUMP_ROW_0 = 0x6B6;
constexpr uint32_t BMS_DUMP_ROW_1 = 0x22'F028;
constexpr uint32_t BMS_DUMP_ROW_2 = 0x22'F029;

//-----------------------------------------------------------------------------
// Remote Emergency System (RES)
//-----------------------------------------------------------------------------
constexpr uint16_t NODE_ID = 0x011;                // 0x011
constexpr uint16_t RES_STATE = (0x180 + NODE_ID);  //(0x180 + NODE_ID)
constexpr uint16_t RES_READY = (0x700 + NODE_ID);  //(0x700 + NODE_ID)

constexpr uint16_t RES_ACTIVATE = 0x000;  // 0x000

// Bamocar
constexpr uint8_t BTB_READY_0 = 0xE2;  // 0xE2
constexpr uint8_t BTB_READY_1 = 0x01;  // 0x01
constexpr uint8_t BTB_READY_2 = 0x00;  // 0x00
constexpr uint8_t BTB_READY_3 = 0x00;  // 0x00
constexpr uint8_t ENABLE_0 = 0xE8;     // 0xE8
constexpr uint8_t ENABLE_1 = 0x01;     // 0x01
constexpr uint8_t ENABLE_2 = 0x00;     // 0x00
constexpr uint8_t ENABLE_3 = 0x00;     // 0x00
constexpr std::array<uint8_t, 3> BTB_READY_SEQUENCE = {
    BTB_READY_1, BTB_READY_2, BTB_READY_3};  // {BTB_READY_1, BTB_READY_2, BTB_READY_3}
constexpr std::array<uint8_t, 3> ENABLE_SEQUENCE = {ENABLE_1, ENABLE_2,
                                                    ENABLE_3};  // {ENABLE_1, ENABLE_2, ENABLE_3}
constexpr uint8_t DC_VOLTAGE = 0xEB;                            // 0xEB
constexpr uint8_t SPEED_ACTUAL = 0x30;                          // 0x30
constexpr uint8_t CURRENT_ACTUAL = 0x20;                        // 0x20
constexpr uint8_t LOGICMAP_ERRORS = 0x8F;                       // 0x8F
constexpr uint8_t MOTOR_TEMPERATURE = 0x49;                     // 0x49

constexpr uint8_t SPEED_LIMIT = 0x34;         // 0x34
constexpr uint8_t SPEED_DELTAMA_ACC = 0x35;   // 0x35
constexpr uint8_t SPEED_DELTAMA_DECC = 0xED;  // 0xED
constexpr uint8_t DEVICE_I_MAX = 0xC4;        // 0xC4
constexpr uint8_t DEVICE_I_CNT = 0xC5;        // 0xC5

//-----------------------------------------------------------------------------
// CELLS
//-----------------------------------------------------------------------------

constexpr uint32_t CELL_TEMPS_BASE_ID = 0x110;  // 0x110
constexpr uint32_t ALL_TEMPS_ID = 0x280;        // 0x280
constexpr uint8_t NTC_SENSOR_COUNT = 18;        // 18
constexpr uint8_t NUM_BOARDS = 6;               // 6 boards

// -----------------------------------------------------------------------------
// DISPLAY WIDGET IDS
// -----------------------------------------------------------------------------
constexpr uint16_t WIDGET_FORM_CMD = 0x9999;
constexpr uint16_t WIDGET_THROTTLE = 0x0001;
constexpr uint16_t WIDGET_BRAKE = 0x0002;
constexpr uint16_t WIDGET_SPEED = 0x0003;
constexpr uint16_t WIDGET_INVERTER_MODE = 0x0004;
constexpr uint16_t WIDGET_CELLS_MIN = 0x0005;
constexpr uint16_t WIDGET_CELLS_MAX = 0x0006;
constexpr uint16_t WIDGET_LV_SOC = 0x0007;
constexpr uint16_t WIDGET_VOLTAGE = 0x0008;
constexpr uint16_t WIDGET_CURRENT = 0x0009;
constexpr uint16_t WIDGET_CH_STATUS = 0x000A;
constexpr uint16_t WIDGET_SDC_BUTTON = 0x000B;
constexpr uint16_t WIDGET_INVERTER_ERRORS = 0x000C;
constexpr uint16_t WIDGET_AUTONOMOUS_MISSION = 0x000E;
constexpr uint16_t WIDGET_INVERTER_WARNINGS = 0x000D;
constexpr uint16_t WIDGET_ALL_TEMPS = 0x0010;  // Base ID for all temperature widgets
constexpr uint16_t WIDGET_HV_SOC = 0x0011;
constexpr uint16_t WIDGET_BMS_DUMP_0 = 0xBB00;
constexpr uint16_t WIDGET_BMS_DUMP_1 = 0xBB01;
constexpr uint16_t WIDGET_BMS_DUMP_2 = 0xBB02;
constexpr uint16_t WIDGET_BMS_DUMP_3 = 0xBB03;