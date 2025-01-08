#include <FlexCAN_T4.h>

#include "Arduino.h"

#define DELAY_VALUE 500
#define N_NTC 18
#define N_BYTES 1023
#define VDD 5.0
#define RESISTOR_PULLUP 10000.0
#define RESISTOR_NTC_REFERNCE 10000.0  // Resistência a 25°C do NTC
#define TEMPERATURE_DEFAULT_C 25.0
#define TEMPERATURE_DEFAULT_K 298.15
#define NTC_BETA 3971.0
#define MAXIMUM_TEMPERATURE 60.0

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

int pinNTC_Temp[N_NTC] = {A4,  A5,  A6,  A7, A8, A9,  A2,  A3,  A10,
                          A11, A12, A13, A0, A1, A17, A16, A15, A14};
int ERROR_SIGNAL = 35;

float CELL_TEMP[N_NTC];
float MIN_TEMP;
float MAX_TEMP;

uint32_t CAN_IDS[9] = {0x100, 0x101, 0x102, 0x103, 0x104, 0x105, 0x106, 0x107, 0x108};

void read_Temperatures() {
  int ANALOG_SIGNAL;
  float VOLTAGE_DIVIDER;
  float RESISTOR_VALUE;

  MIN_TEMP = 100;
  MAX_TEMP = -100;

  for (int i = 0; i < N_NTC; i++) {
    ANALOG_SIGNAL = analogRead(pinNTC_Temp[i]);
    VOLTAGE_DIVIDER = ANALOG_SIGNAL * (VDD / 1023.0);
    RESISTOR_VALUE = (RESISTOR_PULLUP * VOLTAGE_DIVIDER) / (VDD - VOLTAGE_DIVIDER);
    CELL_TEMP[i] = 1 / ((1 / TEMPERATURE_DEFAULT_K) +
                        (log(RESISTOR_VALUE / RESISTOR_NTC_REFERNCE) /
                         NTC_BETA));       // fórmula para obter a temperatura do NTC em K
    CELL_TEMP[i] = CELL_TEMP[i] - 273.15;  // conversão para Celsius
    MIN_TEMP = min(MIN_TEMP, CELL_TEMP[i]);
    MAX_TEMP = max(MAX_TEMP, CELL_TEMP[i]);
  }
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

void send_CAN_Temperatures() {
  CAN_message_t msg;

  for (int i = 0; i < 9; i++) {
    msg.id = CAN_IDS[i];
    msg.len = 8;
    float temp1 = CELL_TEMP[i * 2];
    float temp2 = CELL_TEMP[i * 2 + 1];
    memcpy(&msg.buf[0], &temp1, sizeof(temp1));
    memcpy(&msg.buf[4], &temp2, sizeof(temp2));
    can1.write(msg);

    Serial.print("Enviando via CAN - ID: 0x");
    Serial.print(CAN_IDS[i], HEX);
    Serial.print(" | Temp1: ");
    Serial.print(temp1);
    Serial.print("°C, Temp2: ");
    Serial.println(temp2);
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

void setup() {
  Serial.begin(9600);
  code_reset();
  can1.begin();
  can1.setBaudRate(500000);
}

void loop() {
  read_Temperatures();
  check_Temperatures();
  send_CAN_Temperatures();
  show_Temperatures();
  delay(DELAY_VALUE);
}

// o que falta adicionar é uma
//     "master" de modo a receber os IDs todos de CAN e depois mandar um ID especifico de CAN para a
//     BMS(temperatura máxima e minima das celulas),
//     depois amanha falo contigo