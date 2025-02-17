#ifndef _CAN_H_
#define _CAN_H_

#include <Arduino.h>

#define BMS_ID_CCL 0x101
#define SET_DATA_RESPONSE 0x01
#define SET_VOLTAGE_RESPONSE 0x02
#define SET_CURRENT_RESPONSE 0x03
#define READ_DATA_RESPONSE 0x03
#define REQUEST_CURRENT_ID 0x02200000
#define RESPONSE_CHARGER__CURRENT_ID 0x02204000
#define CURRENT_VOLTAGE_RESPONSE 0x00
#define CURRENT_CURRENT_RESPONSE 0x2f
#define CHARGER_SAFETY_BIT_MASK 0x04
#define BMS_ID_ERR 0x123
#define CHARGER_ID 0x02207446
#define TA_ID 0x301
#define BMS_ID 0x639

#define SHUTDOWN_PIN 32
#define CH_SAFETY_PIN 36
#define LATCHING_ERROR_PIN 34

#define MAX_VOLTAGE 456000
#define MAX_CURRENT 18000

struct PARAMETERS {
    uint32_t setVoltage = 0;
    uint32_t currVoltage = 0;
    uint32_t allowedCurrent = 0;
    uint32_t setCurrent = 0;
    uint32_t currCurrent = 0;
    uint32_t ccl = 0;
    int16_t temp[60];
};

#endif  // _CAN_H_