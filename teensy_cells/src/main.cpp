#include "../include/temp_header.hpp"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

u_int8_t pinNTC_Temp[N_NTC] = {A4,  A5,  A6,  A7, A8, A9,  A2,  A3,  A10,
                          A11, A12, A13, A0, A1, A17, A16, A15, A14};


float CELL_TEMP[N_NTC];

static BoardData board_temps[TOTAL_BOARDS];

float read_ntc_temperature(int analog_value) {
  if (analog_value < 0 || analog_value > 1023) {
    return TEMPERATURE_DEFAULT_C;
  }
  float voltage_divider = static_cast<float>(analog_value) * (V_REF / 1023.0f);

  float resistor_value = (RESISTOR_PULLUP * voltage_divider) / (VDD - voltage_divider);
  float temp_kelvin = 1.0f / ((1.0f / TEMPERATURE_DEFAULT_K) +
                              (log(resistor_value / RESISTOR_NTC_REFERNCE) / NTC_BETA));
  return temp_kelvin - 273.15f;  // return in celcius
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
  board_temps[BOARD_ID].min_temp = safeTemperatureCast(min_temp);
  board_temps[BOARD_ID].max_temp = safeTemperatureCast(max_temp);
  board_temps[BOARD_ID].avg_temp = safeTemperatureCast(sum_temp / N_NTC);
  board_temps[BOARD_ID].valid = true;
}

void check_Temperatures() {
  bool error_flag = false;
  for (int i = 0; i < N_NTC; i++) {
    if (CELL_TEMP[i] > MAXIMUM_TEMPERATURE) {
      error_flag = true;
    }
  }
  if (error_flag) {
    digitalWrite(ERROR_SIGNAL, HIGH);
  } else {
    digitalWrite(ERROR_SIGNAL, LOW);
  }
}

int8_t safeTemperatureCast(float temp) {
  if (temp > 127.0f) return MAX_INT8_T;
  if (temp < -128.0f) return MIN_INT8_T;
  return static_cast<int8_t>(round(temp));
}

void send_CAN_max_min_avg_Temperatures() {
  CAN_message_t msg;
  msg.id = MASTER_CELL_ID;
  msg.len = 4;

  msg.buf[0] = BOARD_ID;
  msg.buf[1] = board_temps[BOARD_ID].min_temp;
  msg.buf[2] = board_temps[BOARD_ID].max_temp;
  msg.buf[3] = board_temps[BOARD_ID].avg_temp;
  can1.write(msg);

  Serial.print("CAN MSG - ID: 0x109 | Min: ");
  Serial.print(static_cast<int8_t>(msg.buf[1]));
  Serial.print("°C, Max: ");
  Serial.print(static_cast<int8_t>(msg.buf[2]));
  Serial.print("°C, Avg: ");
  Serial.println(static_cast<int8_t>(msg.buf[3]));
}
void send_to_BMS(int8_t global_min, int8_t global_max, int8_t global_avg) {
  CAN_message_t msg;
  msg.id = BMS_THERMISTOR_ID;
  msg.len = 8;
  msg.buf[0] = THERMISTOR_MODULE_NUMBER;
  msg.buf[1] = global_min;
  msg.buf[2] = global_max;
  msg.buf[3] = global_avg;
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
      msg.buf[i + 2] = cellIndex < N_NTC ? safeTemperatureCast(CELL_TEMP[cellIndex]) : 0;
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
  Serial.print("\033[2J");
  Serial.print("\033[H");
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
      board_temps[board].min_temp = static_cast<int8_t>(msg.buf[1]);
      board_temps[board].max_temp = static_cast<int8_t>(msg.buf[2]);
      board_temps[board].avg_temp = static_cast<int8_t>(msg.buf[3]);
      board_temps[board].valid = true;
    }
  }
}

void calculate_global_stats(int8_t& global_min, int8_t& global_max, int8_t& global_avg) {
  int8_t sum = 0;
  u_int8_t valid_count = 0;
  global_min = MAX_INT8_T;
  global_max = MIN_INT8_T;

  for (const auto& board : board_temps) {
    if (board.valid) {
      global_min = min(global_min, board.min_temp);
      global_max = max(global_max, board.max_temp);
      sum += board.avg_temp;
      valid_count++;
    }
  }
  global_avg = valid_count > 0 ? sum / valid_count : 0;
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
    int8_t global_min;
    int8_t global_max;
    int8_t global_avg;
    calculate_global_stats(global_min, global_max, global_avg);
    send_to_BMS(global_min, global_max, global_avg);
  }
  show_Temperatures();
  delay(DELAY_INTERVAL);
}
