#include "../include/tijoloes_quentes.hpp"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

const u_int8_t pin_ntc_temp[NTC_SENSOR_COUNT] = {A4,  A5,  A6,  A7, A8, A9,  A2,  A3,  A10,
                                                 A11, A12, A13, A0, A1, A17, A16, A15, A14};

float cell_temps[NTC_SENSOR_COUNT];

unsigned long last_reading_time = 0;
uint8_t error_count = 0;
uint8_t no_error_iterations = 0;
static BoardData board_temps[TOTAL_BOARDS];
#if !THIS_IS_MASTER
unsigned long last_master_message_time = 0;
bool master_has_communicated = false;

// Check if master communication has timed out (for non-master boards)
bool check_master_timeout() {
  const unsigned long current_time = millis();

  // Allow 2 seconds on startup before considering it a timeout
  if (!master_has_communicated) {
    if (current_time > 2000) {
      Serial.println("Timeout: No data ever received from master");
      return true;
    }
    return false;
  }

  if (current_time - last_master_message_time > MAX_TEMP_DELAY_MS) {
    Serial.print("Timeout: No data from master for ");
    Serial.print((current_time - last_master_message_time));
    Serial.println("ms");
    return true;
  }

  return false;
}

void can_receive_from_master(const CAN_message_t& msg) {
  if (msg.id == MASTER_CELL_ID) {
    last_master_message_time = millis();
    master_has_communicated = true;
  }
}

#endif


#if THIS_IS_MASTER
void send_master_heartbeat() {
  CAN_message_t msg;
  msg.id = MASTER_CELL_ID;
  msg.len = 0;  
  
  if (send_can_message(msg)) {
    Serial.println("Sent master heartbeat");
  }
}
#endif


float read_ntc_temperature(const int analog_value) {
  float temperature = TEMPERATURE_DEFAULT_C;

  if (analog_value < ANALOG_MIN || analog_value > ANALOG_MAX) {
    Serial.print("Invalid analog value: ");
    Serial.println(analog_value);
    return temperature;
  }
  // todo: document this
  const float voltage_divider = static_cast<float>(analog_value) * (V_REF / 1023.0f);
  const float resistor_value = (RESISTOR_PULLUP * voltage_divider) / (VDD - voltage_divider);
  const float temp_kelvin = 1.0f / ((1.0f / TEMPERATURE_DEFAULT_K) +
                                    (log(resistor_value / RESISTOR_NTC_REFERNCE) / NTC_BETA));
  temperature = temp_kelvin - KELVIN_OFFSET;

  return temperature;
}

void read_check_temperatures() {
  float sum_temp = 0.0;
  float min_temp = TEMPERATURE_MIN_C;
  float max_temp = TEMPERATURE_MAX_C;
  bool error = false;

  for (int i = 0; i < NTC_SENSOR_COUNT; i++) {
    cell_temps[i] = read_ntc_temperature(analogRead(pin_ntc_temp[i]));
    min_temp = min(min_temp, cell_temps[i]);
    max_temp = max(max_temp, cell_temps[i]);
    sum_temp += cell_temps[i];

    if (cell_temps[i] > MAXIMUM_TEMPERATURE && !error) {
      error_count++;
      error = true;
    }

    board_temps[BOARD_ID].temp_data.min_temp = safe_temperature_cast(min_temp);
    board_temps[BOARD_ID].temp_data.max_temp = safe_temperature_cast(max_temp);
    board_temps[BOARD_ID].temp_data.avg_temp = safe_temperature_cast(sum_temp / NTC_SENSOR_COUNT);
    board_temps[BOARD_ID].has_reported = true;

    digitalWrite(ERROR_SIGNAL, error_count >= MAX_NUM_ERRORS ? HIGH : LOW);
  }
  if (!error) {
    no_error_iterations++;
  }
}

bool check_temperature_timeouts() {
  const unsigned long current_time = millis();
  bool timeout_detected = false;

  for (uint8_t board_id = 1; board_id < TOTAL_BOARDS; board_id++) {
    const BoardData& board = board_temps[board_id];

    if (!board.has_reported) {
      // Allow 2 seconds on startup before considering it a timeout
      if (current_time > 2000) {
        Serial.print("Timeout: No data ever received from board ");
        Serial.println(board_id);
        timeout_detected = true;
      }
      continue;
    }

    if (current_time - board.last_update_ms > MAX_TEMP_DELAY_MS) {
      Serial.print("Timeout: Stale data from board ");
      Serial.print(board_id);
      Serial.print(", last update was ");
      Serial.print((current_time - board.last_update_ms));
      Serial.println("ms ago");
      timeout_detected = true;
    }
  }

  return timeout_detected;
}

int8_t safe_temperature_cast(const float temp) {
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

bool send_can_message(CAN_message_t& msg) {
  for (uint8_t attempt = 0; attempt < MAX_RETRIES; attempt++) {
    if (can1.write(msg)) {
      return true;
    }
    delay(5);  // Short delay before retry
  }
  Serial.println("CAN send failed after retries");
  return false;
}

void send_can_max_min_avg_temperatures() {
  CAN_message_t msg;
  msg.id = CELL_TEMPS_BASE_ID + BOARD_ID;
  msg.len = 4;

  msg.buf[0] = BOARD_ID;
  msg.buf[1] = board_temps[BOARD_ID].temp_data.min_temp;
  msg.buf[2] = board_temps[BOARD_ID].temp_data.max_temp;
  msg.buf[3] = board_temps[BOARD_ID].temp_data.avg_temp;

  if (send_can_message(msg)) {
    Serial.print("CAN MSG - ID: 0x109 | Min: ");
    Serial.print(static_cast<int8_t>(msg.buf[1]));
    Serial.print("°C, Max: ");
    Serial.print(static_cast<int8_t>(msg.buf[2]));
    Serial.print("°C, Avg: ");
    Serial.println(static_cast<int8_t>(msg.buf[3]));
  }
}
void send_to_bms(const TemperatureData& global_data) {
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

void send_can_all_cell_temperatures() {
  CAN_message_t msg;

  for (u_int8_t msgIndex = 0; msgIndex < 3; msgIndex++) {
    msg.id = BOARD_ID;  // TODO
    msg.len = 8;

    msg.buf[0] = BOARD_ID;
    msg.buf[1] = msgIndex;

    for (int i = 0; i < CELLS_PER_MESSAGE; i++) {
      int cellIndex = (msgIndex * CELLS_PER_MESSAGE) + i;
      if (cellIndex < NTC_SENSOR_COUNT) {
        msg.buf[i + 2] = safe_temperature_cast(cell_temps[cellIndex]);
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
    Serial.println(min((msgIndex + 1) * CELLS_PER_MESSAGE, NTC_SENSOR_COUNT));

    delay(100);
  }
}

void show_temperatures() {
  Serial.write("\033[2J");  // Clear screen
  Serial.write("\033[H");   // Home cursor
  Serial.println("----------- Temperaturas -----------");
  for (int i = 0; i < NTC_SENSOR_COUNT; i++) {
    Serial.print("CELL ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(cell_temps[i]);
    Serial.println("°C");
  }
}

void code_reset() {
  pinMode(ERROR_SIGNAL, OUTPUT);
  digitalWrite(ERROR_SIGNAL, LOW);
}

void can_snifflas(const CAN_message_t& msg) {
  if (msg.id >= CELL_TEMPS_BASE_ID && msg.id < CELL_TEMPS_BASE_ID + TOTAL_BOARDS && msg.len == 4) {
    uint8_t board_from_id = msg.id - CELL_TEMPS_BASE_ID;
    uint8_t board_from_buf = msg.buf[0];

    if (board_from_id != board_from_buf) {
      Serial.print("Warning: Board ID mismatch - ID from message: ");
      Serial.print(board_from_id);
      Serial.print(", ID from payload: ");
      Serial.println(board_from_buf);
      return;
    }

    if (board_from_id < TOTAL_BOARDS) {
      board_temps[board_from_id].temp_data.min_temp = static_cast<int8_t>(msg.buf[1]);
      board_temps[board_from_id].temp_data.max_temp = static_cast<int8_t>(msg.buf[2]);
      board_temps[board_from_id].temp_data.avg_temp = static_cast<int8_t>(msg.buf[3]);
      board_temps[board_from_id].has_reported = true;
      board_temps[board_from_id].last_update_ms = millis();
    } else {
      Serial.print("Error: Invalid board ID: ");
      Serial.println(board_from_id);
    }
  }
}

void calculate_global_stats(TemperatureData& global_data) {
  int16_t sum = 0;
  u_int8_t valid_count = 0;
  global_data.min_temp = MAX_INT8_T;
  global_data.max_temp = MIN_INT8_T;

  for (const auto& board : board_temps) {
    if (board.has_reported) {
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

#if THIS_IS_MASTER
  unsigned long current_time = millis();
  for (uint8_t i = 0; i < TOTAL_BOARDS; i++) {
    if (i != BOARD_ID) {
      board_temps[i].last_update_ms = current_time;
    }
  }

  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  can1.setFIFOFilter(REJECT_ALL);

  for (uint8_t i = 0; i < min(8, TOTAL_BOARDS); i++) {  // FlexCAN typically supports 8 filters
    can1.setFIFOFilter(i, CELL_TEMPS_BASE_ID + i, STD);
  }

  can1.onReceive(can_snifflas);
  Serial.println("CAN filters configured for all board IDs");
#else

  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  can1.setFIFOFilter(REJECT_ALL);
  can1.setFIFOFilter(0, MASTER_CELL_ID, STD);
  can1.onReceive(can_receive_from_master);
  Serial.println("CAN filter configured for master messages");
#endif
}

void loop() {
  unsigned long current_time = millis();

  if (current_time - last_reading_time > TEMP_SENSOR_READ_INTERVAL) {
    last_reading_time = current_time;
    read_check_temperatures();

#if !THIS_IS_MASTER
    bool master_timeout = check_master_timeout();
    if (master_timeout) {
      Serial.println("EMERGENCY SHUTDOWN: Master data timeout detected!");
      digitalWrite(ERROR_SIGNAL, HIGH);
    }
    send_can_max_min_avg_temperatures();
#endif

#if THIS_IS_MASTER
    bool timeout_detected = check_temperature_timeouts();

    if (timeout_detected) {
      Serial.println("EMERGENCY SHUTDOWN: Temperature data timeout detected!");
      digitalWrite(ERROR_SIGNAL, HIGH);
    }

    TemperatureData global_data;
    calculate_global_stats(global_data);
    send_to_bms(global_data);
    send_master_heartbeat();
#endif
    show_temperatures();
  }
  if (no_error_iterations >= NO_ERROR_RESET_THRESHOLD) {
    error_count = 0;
    no_error_iterations = 0;
  }
}
