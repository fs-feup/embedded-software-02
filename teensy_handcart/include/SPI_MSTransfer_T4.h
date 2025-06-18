#pragma once

#include "Arduino.h"
#include "circular_buffer.h"
#include <functional>
#include <SPI.h>

#if defined(__IMXRT1062__)
// 48.5.1.1 LPSPI memory map
#define SLAVE_CR spiAddr[4] // multiply index by 4 for address in data sheet (convert to HEX)
#define SLAVE_FCR spiAddr[22]
#define SLAVE_FSR spiAddr[23]
#define SLAVE_IER spiAddr[6]
#define SLAVE_CFGR0 spiAddr[8]
#define SLAVE_CFGR1 spiAddr[9]
#define SLAVE_TDR(x) spiAddr[25] = (x)
#define SLAVE_RDR spiAddr[29]
#define SLAVE_SR spiAddr[5]
#define SLAVE_TCR_REFRESH spiAddr[24] = (2UL << 27) | LPSPI_TCR_FRAMESZ(16 - 1) // Prescale Divide by 4 | Frame Size 16 bits

#define SPI_WAIT_STATE \
    while ( !(SLAVE_SR & (1UL << 9)) ) { /* FCF: Frame Complete Flag, set when PCS deasserts */ \
      if ( !(SLAVE_FSR & 0x1F0000) ) continue; /* wait for received data */ \
      if ( (SLAVE_SR & (1UL << 8)) ) { /* WCF set */

#define SPI_ENDWAIT_STATE \
      } \
    }

#define SPI_ISR_EXIT \
    SLAVE_SR = 0x3F00; \
    asm volatile ("dsb"); \
    return;
#endif

#define SPI_MST_QUEUE_SLOTS 20
#define SPI_MST_DATA_BUFFER_MAX 20

struct AsyncMST {
  uint16_t packetID = 0;
  uint16_t slaveID = 0;
};

using slave_handler_ptr = void(*)(uint16_t* buffer, uint16_t length, AsyncMST info);
using detectPtr = std::function<void(AsyncMST info)>;

typedef void (*_SPI_ptr)();

#define SPI_MSTransfer_T4_CLASS template<SPIClass* port = nullptr>
#define SPI_MSTransfer_T4_FUNC template<SPIClass* port>
#define SPI_MSTransfer_T4_OPT SPI_MSTransfer_T4<port>

extern SPIClass SPI;

class SPI_MSTransfer_T4_Base {
  public:
    virtual void SPI_MSTransfer_SLAVE_ISR();
};

static SPI_MSTransfer_T4_Base* LPSPI4 = nullptr;


// The size (2nd template argument) must always be a power of 2
// This example doesn't work bcs it's not a compile-time constant:
// (uint32_t)pow(2, ceil(log(SPI_MST_QUEUE_SLOTS) / log(2)))
inline Circular_Buffer<uint16_t, SPI_MST_QUEUE_SLOTS, SPI_MST_DATA_BUFFER_MAX> smtqueue;

SPI_MSTransfer_T4_CLASS class SPI_MSTransfer_T4 : public SPI_MSTransfer_T4_Base {
  public:
    SPI_MSTransfer_T4();
    void begin() const;
    uint16_t transfer16(const uint16_t *buffer, uint16_t length, uint16_t widgetID, uint16_t packetID);

  private:
    volatile uint32_t *spiAddr;
    void SPI_MSTransfer_SLAVE_ISR() override;
    uint32_t nvic_irq = 0;
};

#include "SPI_MSTransfer_T4.tpp"
