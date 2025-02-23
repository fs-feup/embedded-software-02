#include "../include/temp_header.hpp"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

const u_int8_t pinNTC_Temp[N_NTC] = {A4,  A5,  A6,  A7, A8, A9,  A2,  A3,  A10,
                                     A11, A12, A13, A0, A1, A17, A16, A15, A14};

float CELL_TEMP[N_NTC];

static BoardData board_temps[TOTAL_BOARDS];

float read_ntc_temperature(const int analog_value) {
  float temperature = TEMPERATURE_DEFAULT_C;

  if (analog_value >= 0 && analog_value <= 1023) {
    const float voltage_divider = static_cast<float>(analog_value) * (V_REF / 1023.0f);
    const float resistor_value = (RESISTOR_PULLUP * voltage_divider) / (VDD - voltage_divider);
    const float temp_kelvin = 1.0f / ((1.0f / TEMPERATURE_DEFAULT_K) +
                                      (log(resistor_value / RESISTOR_NTC_REFERNCE) / NTC_BETA));
    temperature = temp_kelvin - 273.15f;
  }

  return temperature;
}

void read_Temperatures() {
  float sum_temp = 0.0;
  float min_temp = 100.0f;
  float max_temp = -100.0f;

  for (int i = 0; i < N_NTC; i++) {
    CELL_TEMP[i] = read_ntc_temperature(analogRead(pinNTC_Temp[i]));
    min_temp = min(min_temp, CELL_TEMP[i]);
    max_temp = max(max_temp, CELL_TEMP[i]);
    sum_temp += CELL_TEMP[i];
  }
  board_temps[BOARD_ID].temp_data.min_temp = safeTemperatureCast(min_temp);
  board_temps[BOARD_ID].temp_data.max_temp = safeTemperatureCast(max_temp);
  board_temps[BOARD_ID].temp_data.avg_temp = safeTemperatureCast(sum_temp / N_NTC);
  board_temps[BOARD_ID].valid = true;
}

void check_Temperatures() {
  bool error_flag = false;
  for (const float& temp : CELL_TEMP) {
    if (temp > MAXIMUM_TEMPERATURE) {
      error_flag = true;
      break;
    }
  }
  if (error_flag) {
    digitalWrite(ERROR_SIGNAL, HIGH);
  } else {
    digitalWrite(ERROR_SIGNAL, LOW);
  }
}

int8_t safeTemperatureCast(const float temp) {
  int8_t result = 0;

  if (temp > 127.0f) {
    result = MAX_INT8_T;
  } else if (temp < -128.0f) {
    result = MIN_INT8_T;
  } else {
    result = static_cast<int8_t>(round(temp));
  }

  return result;
}

void send_CAN_max_min_avg_Temperatures() {
  CAN_message_t msg;
  msg.id = MASTER_CELL_ID;
  msg.len = 4;

  msg.buf[0] = BOARD_ID;
  msg.buf[1] = board_temps[BOARD_ID].temp_data.min_temp;
  msg.buf[2] = board_temps[BOARD_ID].temp_data.max_temp;
  msg.buf[3] = board_temps[BOARD_ID].temp_data.avg_temp;
  can1.write(msg);

  Serial.print("CAN MSG - ID: 0x109 | Min: ");
  Serial.print(static_cast<int8_t>(msg.buf[1]));
  Serial.print("°C, Max: ");
  Serial.print(static_cast<int8_t>(msg.buf[2]));
  Serial.print("°C, Avg: ");
  Serial.println(static_cast<int8_t>(msg.buf[3]));
}
void send_to_BMS(const TemperatureData& global_data) {
  CAN_message_t msg;
  msg.id = BMS_THERMISTOR_ID;
  msg.len = 8;
  msg.buf[0] = THERMISTOR_MODULE_NUMBER;
  msg.buf[1] = global_data.min_temp;
  msg.buf[2] = global_data.max_temp;
  msg.buf[3] = global_data.avg_temp;
  msg.buf[4] = NUMBER_OF_THERMISTORS;
  msg.buf[5] = HIGHEST_THERMISTOR_ID;
  msg.buf[6] = LOWEST_THERMISTOR_ID;
  msg.buf[7] = msg.buf[1] + msg.buf[2] + msg.buf[3] + msg.buf[4] + msg.buf[5] + msg.buf[6] +
               CHECKSUM_CONSTANT + MSG_LENGTH;
  can1.write(msg);
  // According to documentation we might need to send message to another id as well, although last
  // year only this one was used and worked fine
}

void send_CAN_all_cell_temperatures() {
  CAN_message_t msg;

  for (u_int8_t msgIndex = 0; msgIndex < 3; msgIndex++) {
    msg.id = BOARD_ID;  // TODO
    msg.len = 8;

    msg.buf[0] = BOARD_ID;
    msg.buf[1] = msgIndex;

    for (int i = 0; i < CELLS_PER_MESSAGE; i++) {
      int cellIndex = (msgIndex * CELLS_PER_MESSAGE) + i;
      if (cellIndex < N_NTC) {
        msg.buf[i + 2] = safeTemperatureCast(CELL_TEMP[cellIndex]);
      } else {
        msg.buf[i + 2] = 0;
      }
    }

    can1.write(msg);

    Serial.print("CAN MSG - ID: 0x");
    Serial.print(msg.id, HEX);
    Serial.print(" | Board: ");
    Serial.print(BOARD_ID);
    Serial.print(" | Cells ");
    Serial.print(msgIndex * CELLS_PER_MESSAGE + 1);
    Serial.print("-");
    Serial.println(min((msgIndex + 1) * CELLS_PER_MESSAGE, N_NTC));

    delay(100);
  }
}

void show_Temperatures() {
  Serial.print("\o{33}[2J");
  Serial.print("\o{33}[H");
  Serial.println("----------- Temperaturas -----------");
  for (int i = 0; i < N_NTC; i++) {
    Serial.print("CELL ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(CELL_TEMP[i]);
    Serial.println("°C");
  }
}

void code_reset() {
  pinMode(ERROR_SIGNAL, OUTPUT);
  digitalWrite(ERROR_SIGNAL, LOW);
}

void can_sniffer(const CAN_message_t& msg) {
  if (msg.id == MASTER_CELL_ID && msg.len == 4) {
    uint8_t board = msg.buf[0];
    if (board < TOTAL_BOARDS) {
      board_temps[board].temp_data.min_temp = static_cast<int8_t>(msg.buf[1]);
      board_temps[board].temp_data.max_temp = static_cast<int8_t>(msg.buf[2]);
      board_temps[board].temp_data.avg_temp = static_cast<int8_t>(msg.buf[3]);
      board_temps[board].valid = true;
    }
  }
}

void calculate_global_stats(TemperatureData& global_data) {
  int8_t sum = 0;
  u_int8_t valid_count = 0;
  global_data.min_temp = MAX_INT8_T;
  global_data.max_temp = MIN_INT8_T;

  for (const auto& board : board_temps) {
    if (board.valid) {
      global_data.min_temp = min(global_data.min_temp, board.temp_data.min_temp);
      global_data.max_temp = max(global_data.max_temp, board.temp_data.max_temp);
      sum += board.temp_data.avg_temp;
      valid_count++;
    }
  }
  if (valid_count > 0) {
    global_data.avg_temp = sum / valid_count;
  } else {
    global_data.avg_temp = 0;
  }
}

void setup() {
  Serial.begin(9600);
  code_reset();
  can1.begin();
  can1.setBaudRate(CAN_BAUD_RATE);
  if (THIS_IS_MASTER) {
    can1.enableFIFO();
    can1.enableFIFOInterrupt();
    can1.setFIFOFilter(REJECT_ALL);
    can1.setFIFOFilter(0, MASTER_CELL_ID, STD);
    can1.onReceive(can_sniffer);
  }
}

void loop() {
  read_Temperatures();
  check_Temperatures();

  if (!THIS_IS_MASTER) {
    send_CAN_max_min_avg_Temperatures();
  } else {
    TemperatureData global_data;
    calculate_global_stats(global_data);
    send_to_BMS(global_data);
  }
  show_Temperatures();
  delay(DELAY_INTERVAL);
}
