/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_cc3000
 * @{
 * @file        cc3000_spi.h
 * @brief       Implementation of the CC3000 SPI API
 *
 * @author      Fabian Nack <nack@inf.fu-berlin.de>
 * @}
 */

#ifndef __SPI_H__
#define __SPI_H__

#ifdef  __cplusplus
extern "C" {
#endif

#define CC3000_FIRST_WRITE_SIZE     (4)
#define CC3000_SPI_HEADER_SIZE      (5)

#define CC3000_FIRST_WRITE_WAIT     HWTIMER_TICKS(50)

typedef void (*gcSpiHandleRx)(void *p);

typedef enum
{
  eSPI_STATE_POWERUP = 0,
  eSPI_STATE_INITIALIZED,
  eSPI_STATE_IDLE,
  eSPI_STATE_FIRST_WRITE,
  eSPI_STATE_WRITE,
  eSPI_STATE_READ_IRQ
} cc_3000_states_t;

typedef struct {
    gcSpiHandleRx SPIRxHandler;
    unsigned short usTxPacketLength;
    unsigned short usRxPacketLength;
    cc_3000_states_t ulSpiState;
    char *pTxPacket;
    char *pRxPacket;
} spi_information_t;

extern unsigned char wlan_tx_buffer[];

void init_CC3000_SPI(void);

extern void SpiOpen(gcSpiHandleRx pfRxHandler);
extern void SpiClose(void);
extern long SpiWrite(unsigned char *pUserBuffer, unsigned short usLength);
extern void SpiResumeSpi(void);

void WriteWlanPin(unsigned char val);
void WlanInterruptEnable(void);
void WlanInterruptDisable(void);
long ReadWlanInterruptPin(void);

#ifdef  __cplusplus
}
#endif // __cplusplus

#endif
