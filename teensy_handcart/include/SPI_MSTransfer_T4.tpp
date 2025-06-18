#pragma once
#include <SPI_MSTransfer_T4.h>
#include "Arduino.h"
#include "SPI.h"

extern void __attribute__((weak)) lpspi4_slave_isr() {
    LPSPI4->SPI_MSTransfer_SLAVE_ISR();
}

SPI_MSTransfer_T4_FUNC
SPI_MSTransfer_T4_OPT::SPI_MSTransfer_T4() {
#if defined(__IMXRT1062__)
    if (port == &SPI) {
        LPSPI4 = this;
        constexpr uint8_t port_num = 3;
        spiAddr = reinterpret_cast<volatile uint32_t *>(0x40394000 + (0x4000 * port_num));
        // 403A_0000h -> section 48.5.1.1
        CCM_CCGR1 |= (3UL << 6); // lpspi4_clk_enable
        nvic_irq = 32 + port_num; // LPSPI interrupt request line to the core
        _VectorsRam[16 + nvic_irq] = lpspi4_slave_isr;

        /* Alternate pins not broken out on Teensy 4.0/4.1 for LPSPI4 */
        const auto spireg = reinterpret_cast<volatile uint32_t *>(0x401F84EC + (port_num * 0x10)); // section 11.6.323
        /*
        TODO yves: could directly use macros instead of spireg
        IOMUXC_LPSPI4_PCS0_SELECT_INPUT
        IOMUXC_LPSPI4_SCK_SELECT_INPUT
        IOMUXC_LPSPI4_SDI_SELECT_INPUT
        IOMUXC_LPSPI4_SDO_SELECT_INPUT
        */
        // Selecting Pads Involved in Daisy Chain.
        spireg[0] = 0; /* PCS0_SELECT_INPUT */ // pin 10
        spireg[1] = 0; /* SCK_SELECT_INPUT */ // pin 13
        spireg[2] = 0; /* SDI_SELECT_INPUT */ // pin 12
        spireg[3] = 0; /* SDO_SELECT_INPUT */ // pin 11

        // Sets the mux mode to 0b0011 (LPSPI4) for each pad
        IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03 = 0x3; /* LPSPI4 SCK (CLK) */
        IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_01 = 0x3; /* LPSPI4 SDI (MISO) */
        IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_02 = 0x3; /* LPSPI4 SDO (MOSI) */
        IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_00 = 0x3; /* LPSPI4 PCS0 (CS) */

        // already explained this in master (200mhz, pke enabled, drive strength field R0/3)
        IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_01 = IOMUXC_PAD_DSE(3) | IOMUXC_PAD_SPEED(3) | IOMUXC_PAD_PKE;
        /* LPSPI4 SDI (MISO) */
        IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_02 = IOMUXC_PAD_DSE(3) | IOMUXC_PAD_SPEED(3) | IOMUXC_PAD_PKE;
        /* LPSPI4 SDO (MOSI) */
        IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_03 = IOMUXC_PAD_DSE(3) | IOMUXC_PAD_SPEED(3) | IOMUXC_PAD_PKE;
        /* LPSPI4 SCK (CLK) */
        IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_00 = IOMUXC_PAD_DSE(3) | IOMUXC_PAD_SPEED(3) | IOMUXC_PAD_PKE;
        /* LPSPI4 PCS0 (CS) */
    }
#endif
}

SPI_MSTransfer_T4_FUNC
void SPI_MSTransfer_T4_OPT::begin() const {
#if defined(__IMXRT1062__)
    SLAVE_CR = LPSPI_CR_RST; /* Reset Module */
    SLAVE_CR = 0; /* Disable Module */
    SLAVE_FCR = 0;
    SLAVE_IER = 0x1; /* RX Interrupt */
    // todo yves: this is weird bcs bit 0 zero corresponds to TDIE (Transmit Data Interrupt Enable)
    SLAVE_CFGR0 = 0;
    SLAVE_CFGR1 = (LPSPI_CFGR1_OUTCFG & 0xFCFFFFFF) | (3UL << 24);

    SLAVE_SR = 0x3F00; /* Clear status register */
    SLAVE_TCR_REFRESH;
    SLAVE_TDR(0x0); /* dummy data, must populate initial TX slot */
    SLAVE_CR |= LPSPI_CR_MEN | LPSPI_CR_DBGEN | LPSPI_CR_DOZEN; /* Enable Module, Debug Mode, Doze Mode */
#endif

    NVIC_ENABLE_IRQ(nvic_irq);
}

SPI_MSTransfer_T4_FUNC
void SPI_MSTransfer_T4_OPT::SPI_MSTransfer_SLAVE_ISR() {
    static uint16_t header;

    // Blocking loop; Wait for FEED command
    SPI_WAIT_STATE
        header = SLAVE_RDR;
        header = (header << 1) | (header >> 15); // Apply circular left shift of 1
        if ((header != 0xFEED)) {
            SLAVE_TDR(header);
            continue;
        } else
            SLAVE_TDR(0xCC00);

        break;
    SPI_ENDWAIT_STATE

    if (!smtqueue.size()) // No slave queue
    {
        SPI_WAIT_STATE
            SLAVE_RDR;
            SLAVE_TDR(0x6900);
        SPI_ENDWAIT_STATE
    } else // Access Slave Queue
    {
        uint16_t buf[smtqueue.length_front()] = {0}, pos = 0, command = 0;
        smtqueue.peek_front(buf, sizeof(buf) >> 1);
        SPI_WAIT_STATE
            command = SLAVE_RDR;
            command = (command << 1) | (command >> 15);
            SLAVE_TDR(0x6900 | smtqueue.size());
            if (command == 0xF00D) {
                SPI_WAIT_STATE
                    command = SLAVE_RDR;
                    command = (command << 1) | (command >> 15);
                    if (pos >= (sizeof(buf) >> 1))
                        pos = 0;
                    SLAVE_TDR(buf[pos]);
                    pos++;
                    if (command == 0xCE0A) {
                        smtqueue.pop_front();
                        SPI_WAIT_STATE
                            command = SLAVE_RDR;
                            command = (command << 1) | (command >> 15);
                            SLAVE_TDR(0xD632);
                        SPI_ENDWAIT_STATE
                    }
                SPI_ENDWAIT_STATE
            }
        SPI_ENDWAIT_STATE
    } /* end of 0xFEED CMD */
    SPI_ISR_EXIT
}

SPI_MSTransfer_T4_FUNC
uint16_t SPI_MSTransfer_T4_OPT::transfer16(const uint16_t *buffer, const uint16_t length, const uint16_t widgetID,
                                           const uint16_t packetID) {
    uint16_t data[5 + length], checksum = 0, data_pos = 0;

    // Build the packet first
    // Header
    data[data_pos] = 0xDA7A;
    checksum ^= data[data_pos];
    data_pos++;

    // Packet Length
    data[data_pos] = length + 5;
    checksum ^= data[data_pos];
    data_pos++;

    // Widget ID
    data[data_pos] = widgetID;
    checksum ^= data[data_pos];
    data_pos++;

    // Packet ID
    data[data_pos] = packetID;
    checksum ^= data[data_pos];
    data_pos++;

    // Actual Data
    for (uint16_t i = 0; i < length; i++) {
        data[data_pos] = buffer[i];
        checksum ^= data[data_pos];
        data_pos++;
    }
    data[data_pos] = checksum;

    // Try to replace existing packet with same widgetID (position 2 in the packet)
    if (smtqueue.replace(data, length + 5, 2, -1, -1)) {
        Serial.printf("Replaced existing packet for widget %04X\n", widgetID);
        return packetID;
    }

    // If no existing packet found, try to add new one
    if (smtqueue.size() == smtqueue.capacity()) {
        Serial.println("Queue is full, cannot transfer data.");
        return 0;
    }

    smtqueue.push_back(data, length + 5);
    Serial.printf("Added new packet for widget %04X\n", widgetID);
    return packetID;
}
