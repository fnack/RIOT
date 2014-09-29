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
#define CC3000_MIN_DATA_LENGTH      (5)

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

typedef struct {
    long lNumNetworks; //       4 Bytes: number of networks found
    long lNetScanStatus; //     4 Bytes: The status of the scan:
                         //         0 - agged results,
                         //         1 - results valid,
                         //         2 - no results
                         //     - 56 bytes: Result entry, where the bytes are arranged as
                         // follows:
    _Bool bValid :1; //         1 bit:  is result valid = 1 or not Valid = 0
    uint8_t sRSSI :7; //        7 bits:     RSSI value;
    uint8_t sSecMode :2; //     2 bits:     securityMode of the AP: 0 - Open, 1 - WEP, 2 WPA, 3 WPA2
    uint8_t lNameLength :6; //  6 bits:     SSID name length
    short sTime; //             2 bytes: the time at which the entry has entered into scans result table
    char strSSID[32]; //        32 bytes:SSID name
    char strBSSID[6]; //        6 bytes: BSSID
} scanresults;

typedef struct {
    unsigned char bValid :1; //     1 byte:     is result valid = 1 or not Valid = 0
    unsigned int uiRSSI :7; //      1 byte:     RSSI value;
    unsigned int uiSecMode; //      1 byte:     securityMode of the AP: 0 - Open, 1 - WEP, 2 WPA, 3 WPA2
    unsigned int uiNameLength; //   1 byte:     SSID name length
    char strSSID[32]; //            32 bytes:   SSID name
} network_t;

extern unsigned char wlan_tx_buffer[];

void cc3000_init(void);

unsigned long cc3000_scan_networks(void/*network_t nets[]*/);
void cc3000_get_ipconfig(void);

extern void SpiOpen(gcSpiHandleRx pfRxHandler);
extern void SpiClose(void);
extern long SpiWrite(unsigned char *pUserBuffer, unsigned short usLength);
extern void SpiResumeSpi(void);

#ifdef  __cplusplus
}
#endif // __cplusplus

#endif
