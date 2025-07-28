#pragma once

#include <Arduino.h>
#include <FlexCAN_T4.h>

#include <string>

#include "../../CAN_IDs.h"
#include "../utils.hpp"
#include "comm/utils.hpp"
#include "debugUtils.hpp"
#include "enum_utils.hpp"
#include "model/systemData.hpp"
#include "utils.hpp"

/**
 * @brief Array of standard CAN message codes to be used for FIFO filtering
 * Each Code struct contains a key and a corresponding message ID.
 */
inline std::array<Code, 7> fifoCodes = {{{0, DASH_ID},
                                         {1, BAMO_RESPONSE_ID},
                                         {2, AS_CU_EMERGENCY_SIGNAL},
                                         {3, MISSION_FINISHED},
                                         {4, AS_CU_ID},
                                         {5, RES_STATE},
                                         {6, RES_READY}}};

/**
 * @brief Array of extended CAN message codes to be used for FIFO filtering
 * Contains the key and corresponding message ID for extended messages.
 */
inline std::array<Code, 1> fifoExtendedCodes = {{
    {7, STEERING_ID},
}};

/**
 * @brief Class that contains definitions of typical messages to send via CAN
 * It serves only as an example of the usage of the strategy pattern,
 * where the communicator is the strategy
 */
class Communicator {
private:
  // Static FlexCAN_T4 object for CAN2 interface with RX and TX buffer sizes specified
  inline static FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;

public:
  // Pointer to SystemData instance for storing system-related data
  inline static SystemData *_systemData = nullptr;

  /**
   * @brief Constructor for the Communicator class
   * Initializes the Communicator with the given system data instance.
   * @param system_data Pointer to the SystemData instance.
   */
  explicit Communicator(SystemData *systemdata);

  /**
   * @brief Initializes the CAN bus
   */
  void init();

  /**
   * @brief Parses the message received from the CAN bus
   */
  static void parse_message(const CAN_message_t &msg);

  /**
   * @brief Sends a message to the CAN bus
   * @param len Length of the message
   * @param buffer Buffer containing the message
   * @param id ID of the message
   * @return 0 if successful
   */
  template <std::size_t N>
  static int send_message(unsigned len, const std::array<uint8_t, N> &buffer, unsigned id);

  /**
   * @brief Callback for message from AS CU
   */
  static void pc_callback(const uint8_t *buf);

  /**
   * @brief Callback RES default callback
   */
  static void res_state_callback(const uint8_t *buf);

  /**
   * @brief Callback for RES activation
   */
  static void res_ready_callback();

  /**
   * @brief Callback from inversor, for alive signal and data
   */
  static void bamocar_callback(const uint8_t *buf);

  /**
   * @brief Callback for steering actuator information
   */
  static void steering_callback();

  /**
   * @brief Callback for dashboard
   */
  static void dash_callback(const uint8_t *buf);

  /**
   * @brief Publish AS State to CAN
   */
  static int publish_state(int state_id);

  /**
   * @brief Publish EBS States to CAN
   */
  static int publish_ebs_states(uint8_t ebs_state, bool is_redundancy = false);

  /**
   * @brief Publish AS Mission to CAN
   */
  static int publish_mission(int mission_id);
  /**
   * @brief Publish SOC to CAN
   */
  static int publish_soc(uint8_t soc);
  /**
   * @brief Publish ASMS state to CAN
   */
  static int publish_asms_on(bool asms_on);
  /**
   * @brief Publish AS Mission to CAN
   */
  static int publish_debug_morning_log(const SystemData &system_data, uint8_t sate,
                                       uint8_t state_checkup);

  /**
   * @brief Publish rl wheel rpm to CAN
   */
  static int publish_rpm();
};

inline Communicator::Communicator(SystemData *system_data) { _systemData = system_data; }

void Communicator::init() {
  can3.begin();
  can3.setBaudRate(1'000'000);
  can3.setRFFN(RFFN_32);
  can3.enableFIFO();
  can3.enableFIFOInterrupt();

  can3.setFIFOFilter(REJECT_ALL);
  for (auto &fifoCode : fifoCodes) can3.setFIFOFilter(fifoCode.key, fifoCode.code, STD);

  for (auto &fifoExtendedCode : fifoExtendedCodes)
    can3.setFIFOFilter(fifoExtendedCode.key, fifoExtendedCode.code, EXT);

  can3.onReceive(FIFO, parse_message);

  can3.mailboxStatus();
}
inline void Communicator::res_state_callback(const uint8_t *buf) {
  bool emg_stop1 = buf[0] & 0x01;
  bool emg_stop2 = buf[3] >> 7 & 0x01;
  bool go_switch = (buf[0] >> 1) & 0x01;
  bool go_button = (buf[0] >> 2) & 0x01;
  // DEBUG_PRINT("RES GO: " + String(go_switch) + "   EMG 1: " + String(emg_stop1) + "   EMG 2: " +
  // String(emg_stop2));

  if (go_button || go_switch)
    _systemData->r2d_logics_.process_go_signal();
  else if (!(emg_stop1 || emg_stop2)) {  // If both are false
    // DEBUG_PRINT("Received Emergency from RES");
    _systemData->failure_detection_.emergency_signal_ = true;
  }

  _systemData->failure_detection_.radio_quality_ = buf[6];
  bool signal_loss = (buf[7] >> 6) & 0x01;
  if (!signal_loss) {
    _systemData->updatable_timestamps_.res_signal_loss_timestamp_
        .reset();  // making sure we dont receive only signal loss for the defined time interval
                   // DEBUG_PRINT("SIGNAL OKAY");
  } else {
    // Too many will violate the disconnection time limit
    // DEBUG_PRINT("SIGNAL LOSS");
  }
}

inline void Communicator::res_ready_callback() {
  // If res sends boot message, activate it
  std::array<uint8_t, 2> msg = {0x01, NODE_ID};  // 0x00 in byte 2 for all nodes

  send_message(2, msg, RES_ACTIVATE);
}

inline void Communicator::bamocar_callback(const uint8_t *buf) {
  _systemData->updatable_timestamps_.inversor_alive_timestamp_.reset();

  if (buf[0] == BTB_READY) {
    if (buf[1] == false) {
      _systemData->failure_detection_.ts_on_ = false;
    }
  } else if (buf[0] == BAMOCAR_BATTERY_VOLTAGE_CODE) {
    unsigned dc_voltage = (buf[2] << 8) | buf[1];
    _systemData->failure_detection_.dc_voltage_ = dc_voltage;

    // Voltage hysteresis:
    if (dc_voltage < DC_THRESHOLD) {
      // When voltage drops/is below threshold:
      // Reset hold timer and check if voltage has been below threshold long enough
      _systemData->updatable_timestamps_.dc_voltage_hold_timestamp_.reset();
      if (_systemData->updatable_timestamps_.dc_voltage_drop_timestamp_.checkWithoutReset()) {
        _systemData->failure_detection_.ts_on_ = false;
      }
    } else {
      // When voltage is above threshold:
      // Reset drop timer and check if voltage has been above threshold long enough
      _systemData->updatable_timestamps_.dc_voltage_drop_timestamp_.reset();
      if (_systemData->updatable_timestamps_.dc_voltage_hold_timestamp_.checkWithoutReset()) {
        _systemData->failure_detection_.ts_on_ = true;
      }
    }
  }
}

inline void Communicator::pc_callback(const uint8_t *buf) {
  // DEBUG_PRINT("PC alive signal received");
  if (buf[0] == PC_ALIVE) {
    _systemData->updatable_timestamps_.pc_alive_timestamp_.reset();
  } else if (buf[0] == MISSION_FINISHED) {
    _systemData->mission_finished_ = true;
  } else if (buf[0] == AS_CU_EMERGENCY_SIGNAL) {
    DEBUG_PRINT("Received Emergency from AS CU");
    _systemData->failure_detection_.emergency_signal_ = true;
  }
}

inline void Communicator::steering_callback() {
  _systemData->updatable_timestamps_.steer_alive_timestamp_.reset();
}

inline void Communicator::dash_callback(const uint8_t *buf) {
  if (buf[0] == HYDRAULIC_LINE) {
    _systemData->hardware_data_.hydraulic_line_front_pressure = (buf[2] << 8) | buf[1];
  }
}

inline void Communicator::parse_message(const CAN_message_t &msg) {
  switch (msg.id) {
    case AS_CU_ID:
      pc_callback(msg.buf);
    case RES_STATE:
      res_state_callback(msg.buf);
      break;
    case RES_READY:
      res_ready_callback();
      break;
    case BAMO_RESPONSE_ID:
      bamocar_callback(msg.buf);
      break;
    case STEERING_ID:
      steering_callback();
      break;
    case DASH_ID:
      dash_callback(msg.buf);
      break;
    default:
      break;
  }
}

inline int Communicator::publish_state(const int state_id) {
  const std::array<uint8_t, 2> msg = {STATE_MSG, static_cast<uint8_t>(state_id)};
  send_message(2, msg, MASTER_ID);
  return 0;
}

inline int Communicator::publish_mission(int mission_id) {
  const std::array<uint8_t, 2> msg = {MISSION_MSG, static_cast<uint8_t>(mission_id)};

  send_message(2, msg, MASTER_ID);
  return 0;
}
inline int Communicator::publish_debug_morning_log(const SystemData &system_data, uint8_t state,
                                                   uint8_t state_checkup) {
  send_message(8, create_debug_message_1(system_data, state, state_checkup), MASTER_ID);
  send_message(8, create_debug_message_2(system_data), MASTER_ID);
  return 0;
}

inline int Communicator::publish_soc(uint8_t soc) {
  const std::array<uint8_t, 2> msg = {SOC_MSG, soc};
  send_message(2, msg, MASTER_ID);
  return 0;
}

inline int Communicator::publish_asms_on(bool asms_on) {
  const std::array<uint8_t, 2> msg = {ASMS, asms_on};
  send_message(2, msg, MASTER_ID);
  return 0;
}

inline int Communicator::publish_rpm() {
  std::array<uint8_t, 5> rl_rpm_msg = {0};
  std::array<uint8_t, 5> rr_rpm_msg = {0};

  char rr_rpm_byte[4];
  char rl_rpm_byte[4];
  rpm_2_byte(_systemData->hardware_data_._left_wheel_rpm, rl_rpm_byte);
  rpm_2_byte(_systemData->hardware_data_._right_wheel_rpm, rr_rpm_byte);

  rl_rpm_msg[0] = LEFT_WHEEL_CODE;
  for (int i = 0; i < 4; i++) {
    rl_rpm_msg[i + 1] = rl_rpm_byte[i];
  }

  rr_rpm_msg[0] = RIGHT_WHEEL_CODE;
  for (int i = 0; i < 4; i++) {
    rr_rpm_msg[i + 1] = rr_rpm_byte[i];
  }
  send_message(5, rl_rpm_msg, MASTER_ID);
  send_message(5, rr_rpm_msg, MASTER_ID);

  return 0;
}

inline int Communicator::publish_ebs_states(uint8_t ebs_state, bool is_redundancy) {
  // EBS state is a 4-bit value, we need to send it as a byte
  // std::array<uint8_t, 2> msg = {EBS_REDUNDANCY_MSG, ebs_state & 0x0F};
  // send_message(2, msg, MASTER_ID);
  // std::array<uint8_t, 2> msg = {EBS_STATE_MSG, ebs_state & 0x0F};
  // send_message(2, msg, MASTER_ID);
  if (is_redundancy) {
    std::array<uint8_t, 2> msg = {EBS_REDUNDANCY_MSG, ebs_state & 0x0F};
    send_message(2, msg, MASTER_ID);
  }else {
    std::array<uint8_t, 2> msg = {EBS_STATE_MSG, ebs_state & 0x0F};
    send_message(2, msg, MASTER_ID);
  }

  return 0;
}

template <std::size_t N>
inline int Communicator::send_message(const unsigned len, const std::array<uint8_t, N> &buffer,
                                      const unsigned id) {
  CAN_message_t can_message;
  can_message.id = id;
  can_message.len = len;
  for (unsigned i = 0; i < len; i++) {
    can_message.buf[i] = buffer[i];
  }
  can3.write(can_message);

  return 0;
}
