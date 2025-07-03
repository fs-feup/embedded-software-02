#include "../include/tijoloes_quentes.hpp"

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can1;  // todo

const u_int8_t pin_ntc_temp[NTC_SENSOR_COUNT] = {A4,  A5,  A6,  A7, A8, A9,  A2,  A3,  A10,
                                                 A11, A12, A13, A0, A1, A17, A16, A15, A14};//after A12 is A13 in teensy 1

float cell_temps[NTC_SENSOR_COUNT];
CAN_error_t error;

const unsigned long SETUP_TIMEOUT = 1000;  // 5 seconds for CAN set up detection

bool baud_1M = true;
bool bitrate_switched = false;
uint32_t current_bitrate = CAN_DRIVING_BAUD_RATE;  // Start with driving
unsigned long last_reading_time = 0;
volatile unsigned long last_message_received_time = 0;
uint8_t error_count = 0;
uint8_t no_error_iterations = 0;
static BoardData board_temps[TOTAL_BOARDS];

#if !THIS_IS_MASTER
volatile unsigned long last_master_message_time = 0;
volatile bool master_has_communicated = false;

// Check if master communication has timed out (for non-master boards)
bool check_master_timeout() {
  const unsigned long current_time = millis();

  // Allow 2 seconds on startup before considering it a timeout
  if (!master_has_communicated) {
    if (current_time > 10000) {
      DEBUG_PRINTLN("Timeout: No data ever received from master");
      return true;
    }
    return false;
  }

  if (current_time - last_master_message_time > MAX_TEMP_DELAY_MS) {
    // DEBUG_PRINT("Timeout: No data from master for ");
    // DEBUG_PRINT((current_time - last_master_message_time));
    // DEBUG_PRINTLN("ms");
    return true;
  }

  return false;
}


void can_receive_from_master(const CAN_message_t& msg) {
  if (msg.id == MASTER_CELL_ID) {
    last_master_message_time = millis();
    master_has_communicated = true;
  }
  last_message_received_time = millis();
}

#endif
void debug_helper() {
  DEBUG_PRINTLN("----------- DEBUG HELPER -----------");

  // --- General Info ---
  DEBUG_PRINT("THIS_IS_MASTER: ");
  DEBUG_PRINTLN(THIS_IS_MASTER ? "true" : "false");
  DEBUG_PRINT("BOARD_ID: ");
  DEBUG_PRINTLN(BOARD_ID);
  DEBUG_PRINT("Current millis(): ");
  DEBUG_PRINTLN(millis());

  // --- Error and State Info ---
  DEBUG_PRINT("error_count: ");
  DEBUG_PRINTLN(error_count);
  DEBUG_PRINT("no_error_iterations: ");
  DEBUG_PRINTLN(no_error_iterations);

  // --- CAN Info ---
  DEBUG_PRINT("Last CAN message received time: ");
  DEBUG_PRINTLN(last_message_received_time);
  // Note: Actual current baud rate isn't stored directly in a global variable after setup.
  // We can print the configured rates.
  DEBUG_PRINT("CAN_BAUD_RATE_1M ?: ");
  DEBUG_PRINTLN(baud_1M ? "ye" : "no");

  // --- Board Specific Temperature Data (for the current board) ---
  DEBUG_PRINTLN("--- Current Board Temperature Data ---");
  DEBUG_PRINT("Board ID [");
  DEBUG_PRINT(BOARD_ID);
  DEBUG_PRINT("] Min Temp: ");
  DEBUG_PRINTLN(board_temps[BOARD_ID].temp_data.min_temp);
  DEBUG_PRINT("Board ID [");
  DEBUG_PRINT(BOARD_ID);
  DEBUG_PRINT("] Max Temp: ");
  DEBUG_PRINTLN(board_temps[BOARD_ID].temp_data.max_temp);
  DEBUG_PRINT("Board ID [");
  DEBUG_PRINT(BOARD_ID);
  DEBUG_PRINT("] Avg Temp: ");
  DEBUG_PRINTLN(board_temps[BOARD_ID].temp_data.avg_temp);
  DEBUG_PRINT("Board ID:");
  DEBUG_PRINTLN(BOARD_ID);

#if THIS_IS_MASTER
  // --- Master Specific Info ---
  DEBUG_PRINTLN("--- Master Specific Debug Info ---");
  DEBUG_PRINTLN("Global Temperature Stats (as sent to BMS):");
  // To print global_data, you might need to make it accessible here or recalculate it.
  // For simplicity, let's print the contents of board_temps which master uses to calculate global
  // stats.
  DEBUG_PRINTLN("Stored Temperatures from other boards:");
  for (uint8_t i = 0; i < TOTAL_BOARDS; i++) {
    if (i == BOARD_ID && THIS_IS_MASTER) {  // Master's own data is already printed above
      // Or if you want to show it in this loop specifically for master:
      // DEBUG_PRINT("Board (Master) ["); DEBUG_PRINT(i); DEBUG_PRINT("] Own Data - Min: ");
      // DEBUG_PRINT(board_temps[i].temp_data.min_temp); DEBUG_PRINT(", Max: ");
      // DEBUG_PRINT(board_temps[i].temp_data.max_temp); DEBUG_PRINT(", Avg: ");
      // DEBUG_PRINT(board_temps[i].temp_data.avg_temp); DEBUG_PRINT(", Last Update: ");
      // DEBUG_PRINTLN(board_temps[i].last_update_ms);
      continue;
    }
    DEBUG_PRINT("Board [");
    DEBUG_PRINT(i);
    DEBUG_PRINT("] Min: ");
    DEBUG_PRINT(board_temps[i].temp_data.min_temp);
    DEBUG_PRINT(", Max: ");
    DEBUG_PRINT(board_temps[i].temp_data.max_temp);
    DEBUG_PRINT(", Avg: ");
    DEBUG_PRINT(board_temps[i].temp_data.avg_temp);
    DEBUG_PRINT(", HasComm: ");
    DEBUG_PRINT(board_temps[i].has_communicated ? "Y" : "N");
    DEBUG_PRINT(", LastUpdate: ");
    DEBUG_PRINTLN(board_temps[i].last_update_ms);
  }
#else
  // --- Slave Specific Info ---
  DEBUG_PRINTLN("--- Slave Specific Debug Info ---");
  DEBUG_PRINT("Last message received from master at: ");
  DEBUG_PRINTLN(last_master_message_time);
  DEBUG_PRINT("Master has communicated: ");
  DEBUG_PRINTLN(master_has_communicated ? "Yes" : "No");
#endif

  DEBUG_PRINTLN("--------- END DEBUG HELPER ---------");
  DEBUG_PRINTLN();  // Add a blank line for readability
}



#if THIS_IS_MASTER
void send_master_heartbeat() {
  CAN_message_t msg;
  msg.id = MASTER_CELL_ID;
  msg.len = 0;

  if (send_can_message(msg)) {
    // DEBUG_PRINTLN("Sent master heartbeat");
  }
}
#endif

float read_ntc_temperature(const int analog_value) {
  float temperature = TEMPERATURE_DEFAULT_C;

  if (analog_value < ANALOG_MIN || analog_value > ANALOG_MAX) {
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
  float min_temp = TEMPERATURE_MAX_C;
  float max_temp = TEMPERATURE_MIN_C;
  bool error = false;
  for (int i = 0; i < NTC_SENSOR_COUNT; i++) {
    cell_temps[i] = read_ntc_temperature(analogRead(pin_ntc_temp[i]));
    DEBUG_PRINT("Cell ");
    DEBUG_PRINT(i+1);
    DEBUG_PRINT(": ");
    DEBUG_PRINTLN(analogRead(pin_ntc_temp[i]));
    min_temp = min(min_temp, cell_temps[i]);
    max_temp = max(max_temp, cell_temps[i]);
    sum_temp += cell_temps[i];
    if (cell_temps[i] > MAXIMUM_TEMPERATURE && !error) {
      error_count++;
      error = true;
    }

    board_temps[BOARD_ID].has_communicated = true;

    digitalWrite(ERROR_SIGNAL, error_count >= MAX_NUM_ERRORS ? HIGH : LOW);
  }
  board_temps[BOARD_ID].temp_data.min_temp = safe_temperature_cast(min_temp);
  board_temps[BOARD_ID].temp_data.max_temp = safe_temperature_cast(max_temp);
  board_temps[BOARD_ID].temp_data.avg_temp = safe_temperature_cast(sum_temp / NTC_SENSOR_COUNT);
  if (!error) {
    no_error_iterations++;
  }
}

bool check_temperature_timeouts() {
  const unsigned long current_time = millis();
  bool timeout_detected = false;

  for (uint8_t board_id = 1; board_id < TOTAL_BOARDS; board_id++) {
    const BoardData& board = board_temps[board_id];

    if (!board.has_communicated) {
      // Allow 2 seconds on startup before considering it a timeout
      if (current_time > 15000) {
        // DEBUG_PRINT("Timeout: No data ever received from board ");
        // DEBUG_PRINTLN(board_id);
        timeout_detected = true;
      }
      continue;
    }

    if (current_time - board.last_update_ms > MAX_TEMP_DELAY_MS) {
      // DEBUG_PRINT("Timeout: Stale data from board ");
      // DEBUG_PRINT(board_id);
      // DEBUG_PRINT(", last update was ");
      // DEBUG_PRINT((current_time - board.last_update_ms));
      // DEBUG_PRINTLN("ms ago");
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
    DEBUG_PRINTLN("CAN send failed, retrying...");
    delay(5);  // Short delay before retry
  }
  DEBUG_PRINTLN("CAN send failed after retries");
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
  // print min max avg

  // DEBUG_PRINT("CAN MSG - ID: 0x109 | Min: ");
  // DEBUG_PRINT(static_cast<int8_t>(msg.buf[1]));
  // DEBUG_PRINT("°C, Max: ");
  // DEBUG_PRINT(static_cast<int8_t>(msg.buf[2]));
  // DEBUG_PRINT("°C, Avg: ");
  // DEBUG_PRINTLN(static_cast<int8_t>(msg.buf[3]));

  if (send_can_message(msg)) {
    // DEBUG_PRINTLN("Sent CAN message with min, max, and avg temperatures");
  } else {
    DEBUG_PRINTLN("Failed to send CAN message");
  }
}
void send_to_bms(const TemperatureData& global_data) {
  // print

  CAN_message_t msg;
  msg.id = BMS_THERMISTOR_ID;
  msg.flags.extended = true;
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

void show_temperatures() {
  DEBUG_WRITE("\033[2J");  // Clear screen
  DEBUG_WRITE("\033[H");   // Home cursor
  DEBUG_PRINTLN("----------- Temperaturas -----------");
  for (int i = 0; i < NTC_SENSOR_COUNT; i++) {
    DEBUG_PRINT("CELL ");
    DEBUG_PRINT(i + 1);
    DEBUG_PRINT(": ");
    DEBUG_PRINT(cell_temps[i]);
    DEBUG_PRINTLN("°C");
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
      DEBUG_PRINT("Warning: Board ID mismatch - ID from message: ");
      DEBUG_PRINT(board_from_id);
      DEBUG_PRINT(", ID from payload: ");
      DEBUG_PRINTLN(board_from_buf);
      return;
    }

    if (board_from_id < TOTAL_BOARDS) {
      board_temps[board_from_id].temp_data.min_temp = static_cast<int8_t>(msg.buf[1]);
      board_temps[board_from_id].temp_data.max_temp = static_cast<int8_t>(msg.buf[2]);
      board_temps[board_from_id].temp_data.avg_temp = static_cast<int8_t>(msg.buf[3]);
      board_temps[board_from_id].has_communicated = true;
      board_temps[board_from_id].last_update_ms = millis();
    } else {
      DEBUG_PRINT("Error: Invalid board ID: ");
      DEBUG_PRINTLN(board_from_id);
    }
  }
  last_message_received_time = millis();
}

void calculate_global_stats(TemperatureData& global_data) {
  int16_t sum = 0;
  u_int8_t valid_count = 0;
  global_data.min_temp = MAX_INT8_T;
  global_data.max_temp = MIN_INT8_T;

  for (uint8_t i = 0; i < TOTAL_BOARDS; i++) {
    const auto& board = board_temps[i];
    if (board.has_communicated) {
      //print board id and the 3 values
      DEBUG_PRINT("Board ID: ");
      DEBUG_PRINT(i); // Use the array index instead of board.id
      DEBUG_PRINT(" Min: ");
      DEBUG_PRINT(board.temp_data.min_temp);
      DEBUG_PRINT(" Max: ");
      DEBUG_PRINT(board.temp_data.max_temp);
      DEBUG_PRINT(" Avg: ");
      DEBUG_PRINTLN(board.temp_data.avg_temp);
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

void initialize_can(uint32_t baudRate) {
  DEBUG_PRINT("Initializing CAN at ");
  DEBUG_PRINT(baudRate);
  DEBUG_PRINTLN(" baud...");

  can1.begin();
  can1.setBaudRate(baudRate);
  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  can1.setFIFOFilter(REJECT_ALL);

#if THIS_IS_MASTER

  for (uint8_t i = 0; i < TOTAL_BOARDS; i++) {  // FlexCAN typically supports 8 filters
    can1.setFIFOFilter(i, CELL_TEMPS_BASE_ID + 1 + i, STD);
  }
  can1.setFIFOFilter(TOTAL_BOARDS, BMS_ID_CCL, STD);
  can1.setFIFOFilter(TOTAL_BOARDS + 1, MASTER_ID, STD);  // Set filter for master messages

  can1.onReceive(can_snifflas);
  DEBUG_PRINTLN("CAN filters configured for all board IDs");
#else

  can1.setFIFOFilter(0, MASTER_CELL_ID, STD);
  can1.setFIFOFilter(1, BMS_ID_CCL, STD);
  can1.setFIFOFilter(2, MASTER_ID, STD);  // Set filter for master messages

  can1.onReceive(can_receive_from_master);
  DEBUG_PRINTLN("CAN filter configured for master messages");
#endif

  can1.mailboxStatus();
  DEBUG_PRINTLN("CAN Initialized/Re-initialized.");
  last_message_received_time = millis();  // Reset message timer for detection logic
}

void setup() {
  Serial.begin(115200);

  code_reset();

#if THIS_IS_MASTER
  unsigned long current_time = millis();
  for (uint8_t i = 0; i < TOTAL_BOARDS; i++) {
    if (i != BOARD_ID) {
      board_temps[i].last_update_ms = current_time;
    }
  }
#endif
  for (int i = 0; i < NTC_SENSOR_COUNT; i++) {
    pinMode(pin_ntc_temp[i], INPUT);
  }
  delay(100);
  initialize_can(CAN_DRIVING_BAUD_RATE);

  // delay(1000);  // Wait a bit before re-initializing
  // can1.disableFIFOInterrupt();  // Disable FIFO interrupts before re-initializing
  // can1.disableFIFO();  // Disable FIFO before re-initializing
  // can1.reset();  // Reset before re-initializing, data sheet 44.8.1
  // delay(100);  // Short delay to ensure reset is complete
}
void loop() {
  unsigned long current_time = millis();

  if (!bitrate_switched && (current_time - last_message_received_time > 1000)) {
    if (current_bitrate == CAN_DRIVING_BAUD_RATE) {
      DEBUG_PRINTLN("Timeout: No CAN message for 1s. Switching from 1Mbps to 125kbps permanently.");
      current_bitrate = CAN_CHARGING_BAUD_RATE;
      can1.setBaudRate(current_bitrate);  // Dynamically change the bitrate
      bitrate_switched = true;            // Set the flag to prevent further switching
    }
  }

  if (current_time - last_reading_time > TEMP_SENSOR_READ_INTERVAL) {
    last_reading_time = current_time;
    read_check_temperatures();

#if !THIS_IS_MASTER
    if (check_master_timeout()) {
      // DEBUG_PRINTLN("EMERGENCY SHUTDOWN: Master data timeout detected!");
      digitalWrite(ERROR_SIGNAL, HIGH);
    }
    send_can_max_min_avg_temperatures();
    // debug_helper();
#endif

#if THIS_IS_MASTER
    bool timeout_detected = check_temperature_timeouts();

    if (timeout_detected) {
      // DEBUG_PRINTLN("EMERGENCY SHUTDOWN: Temperature data timeout detected!");
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
  delay(2 * BOARD_ID);
}