#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <avr/interrupt.h>

#include <cstdint>
#include <deque>

#include "data_struct.hpp"
#include "io_settings.hpp"
#include "utils.hpp"

class DisplaySPI {
private:
  static volatile uint8_t receivedByte;
  static volatile bool dataReceived;
  static volatile uint8_t responseByte;

  int slaveSelectPin;

public:
  DisplaySPI(int csPin = pins::spi::CS);
  ~DisplaySPI();

  void begin();
  void update();
  static void handleSPIInterrupt();
};

volatile uint8_t DisplaySPI::receivedByte = 0;
volatile bool DisplaySPI::dataReceived = false;
volatile uint8_t DisplaySPI::responseByte = 0;

DisplaySPI* activeSPIDisplay = nullptr;

ISR(SPI_STC_vect) { DisplaySPI::handleSPIInterrupt(); }

DisplaySPI::DisplaySPI(int csPin) : slaveSelectPin(csPin) { activeSPIDisplay = this; }

DisplaySPI::~DisplaySPI() {
  if (activeSPIDisplay == this) {
    activeSPIDisplay = nullptr;
  }
}

void DisplaySPI::begin() {
  pinMode(slaveSelectPin, INPUT_PULLUP);
  pinMode(pins::spi::MISO, OUTPUT);
  pinMode(pins::spi::MOSI, INPUT);
  pinMode(pins::spi::SCK, INPUT);

  SPI.begin();
  SPCR |= _BV(SPE);
  SPCR &= ~_BV(MSTR);
  SPCR |= _BV(SPIE);

  Serial.println("Teensy SPI Slave Initialized");
}

void DisplaySPI::update() {
  if (dataReceived) {
    Serial.print("Received Command: 0x");
    Serial.print(receivedByte, HEX);
    Serial.print(", Sent Response: 0x");
    Serial.println(responseByte, HEX);
    dataReceived = false;
  }
}

void DisplaySPI::handleSPIInterrupt() {
  receivedByte = SPDR;
  dataReceived = true;

  switch (receivedByte) {
    case 0x01:
      responseByte = 0xAB;
      break;
    case 0x02:
      responseByte = 0x55;
      break;
    default:
      responseByte = 0xFF;
      break;
  }

  SPDR = responseByte;
}