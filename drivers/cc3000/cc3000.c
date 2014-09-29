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
 * @file        cc3000.c
 * @brief       Implementation of the CC3000 SPI API
 *
 * @author      Fabian Nack <nack@inf.fu-berlin.de>
 * @}
 */

#include <stdio.h>
#include <string.h>
#include "cc3000/cc3000_common.h"
#include "cc3000/socket.h"
#include "cc3000/cc3000_spi.h"
#include "cc3000/hci.h"
#include "cc3000/netapp.h"
#include "cc3000/wlan.h"
#include "hwtimer.h"
#include "board.h"
#include "vtimer.h"

#include "periph/spi.h"
#include "periph/gpio.h"

// SPI CMD
#define READ                (0x03)
#define WRITE               (0x01)

unsigned char wlan_rx_buffer[CC3000_RX_BUFFER_SIZE];
unsigned char wlan_tx_buffer[CC3000_TX_BUFFER_SIZE];

static void wlan_irq_handler(void *args);
static void wlan_cb(long event, char *payload, uint8_t length);
static void wlan_write_enable_pin(unsigned char val);
static void wlan_irq_enable(void);
static void wlan_irq_disable(void);
static long wlan_read_irq_pin(void);

spi_information_t sSpiInformation;

void cc3000_init(void)
{
    /* Configure CS pin */
    gpio_init_out(CC3000_CS, GPIO_NOPULL);
    /* Chip select to high (not active) */
    gpio_set(CC3000_CS);

    /* Configure CC3000 WLAN pin */
	gpio_init_out(CC3000_WLAN_EN, GPIO_NOPULL);
    /* Disable CC3000 WLAN for now */
	gpio_clear(CC3000_WLAN_EN);

	/* Initialize CC3000 SPI */
	spi_poweron(CC3000_SPI);
	spi_init_master(CC3000_SPI, SPI_CONF_SECOND_RISING, SPI_SPEED_10MHZ);

	/* Configure interrupt for CC3000 */
	gpio_init_int(CC3000_SPI_IRQ, GPIO_PULLUP, GPIO_FALLING, wlan_irq_handler, 0);
	gpio_irq_disable(CC3000_SPI_IRQ);

	/* Initialize CC3000 driver from Texas Instruments */
    wlan_init(wlan_cb, NULL, NULL, NULL, wlan_read_irq_pin,
              wlan_irq_enable, wlan_irq_disable, wlan_write_enable_pin);
}

static void wlan_cb(long event, char *payload, uint8_t length)
{
    switch (event) {
        case HCI_EVNT_WLAN_BASE:
            printf("HCI_EVNT_WLAN_BASE\r\n");
            break;
        case HCI_EVNT_WLAN_CONNECT:
            printf("HCI_EVNT_WLAN_CONNECT\r\n");
            break;
        case HCI_EVNT_WLAN_DISCONNECT:
            printf("HCI_EVNT_WLAN_DISCONNECT\r\n");
            break;
        case HCI_EVNT_WLAN_IOCTL_ADD_PROFILE:
            printf("HCI_EVNT_WLAN_IOCTL_ADD_PROFILE\r\n");
            break;
        case HCI_EVNT_SOCKET:
            printf("HCI_EVNT_SOCKET\r\n");
            break;
        case HCI_EVNT_BIND:
            printf("HCI_EVNT_BIND\r\n");
            break;
        case HCI_EVNT_RECV:
            printf("HCI_EVNT_RECV\r\n");
            break;
        case HCI_EVNT_ACCEPT:
            printf("HCI_EVNT_ACCEPT\r\n");
            break;
        case HCI_EVNT_LISTEN:
            printf("HCI_EVNT_LISTEN\r\n");
            break;
        case HCI_EVNT_CONNECT:
            printf("HCI_EVNT_CONNECT\r\n");
            break;
        case HCI_EVNT_SELECT:
            printf("HCI_EVNT_SELECT\r\n");
            break;
        case HCI_EVNT_CLOSE_SOCKET:
            printf("HCI_EVNT_CLOSE_SOCKET\r\n");
            break;
        case HCI_EVNT_RECVFROM:
            printf("HCI_EVNT_RECVFROM\r\n");
            break;
        case HCI_EVNT_SETSOCKOPT:
            printf("HCI_EVNT_SETSOCKOPT\r\n");
            break;
        case HCI_EVNT_GETSOCKOPT:
            printf("HCI_EVNT_GETSOCKOPT\r\n");
            break;
        case HCI_EVNT_BSD_GETHOSTBYNAME:
            printf("HCI_EVNT_BSD_GETHOSTBYNAME\r\n");
            break;
        case HCI_EVNT_SEND:
            printf("HCI_EVNT_SEND\r\n");
            break;
        case HCI_EVNT_WRITE:
            printf("HCI_EVNT_WRITE\r\n");
            break;
        case HCI_EVNT_SENDTO:
            printf("HCI_EVNT_SENDTO\r\n");
            break;
        case HCI_EVNT_PATCHES_REQ:
            printf("HCI_EVNT_PATCHES_REQ\r\n");
            break;
        case HCI_EVNT_UNSOL_BASE:
            printf("HCI_EVNT_UNSOL_BASE\r\n");
            break;
        case HCI_EVNT_WLAN_UNSOL_BASE:
            printf("HCI_EVNT_WLAN_UNSOL_BASE\r\n");
            break;
        case HCI_EVNT_DATA_UNSOL_FREE_BUFF:
            printf("HCI_EVNT_DATA_UNSOL_FREE_BUFF\r\n");
            break;
        case HCI_EVNT_NVMEM_CREATE_ENTRY:
            printf("HCI_EVNT_NVMEM_CREATE_ENTRY\r\n");
            break;
        case HCI_EVNT_NVMEM_SWAP_ENTRY:
            printf("HCI_EVNT_NVMEM_SWAP_ENTRY\r\n");
            break;
        case HCI_EVNT_NVMEM_READ:
            printf("HCI_EVNT_NVMEM_READ\r\n");
            break;
        case HCI_EVNT_NVMEM_WRITE:
            printf("HCI_EVNT_NVMEM_WRITE\r\n");
            break;
        case HCI_EVNT_READ_SP_VERSION:
            printf("HCI_EVNT_READ_SP_VERSION\r\n");
            break;
        case HCI_EVNT_INPROGRESS:
            printf("HCI_EVNT_INPROGRESS\r\n");
            break;
        case HCI_EVNT_WLAN_ASYNC_SIMPLE_CONFIG_DONE:
            printf("HCI_EVNT_WLAN_ASYNC_SIMPLE_CONFIG_DONE\r\n");
    //        CC3000_SmartConfig_finished = true;
            break;
        case HCI_EVNT_WLAN_UNSOL_CONNECT:
            printf("HCI_EVNT_WLAN_UNSOL_CONNECT\r\n");
    //        LED_GR_ON;
    //        CC3000_is_Connected = true;
            break;
        case HCI_EVNT_WLAN_UNSOL_DISCONNECT:
            printf("HCI_EVNT_WLAN_UNSOL_DISCONNECT\r\n");
    //        LED_GR_OFF;
    //        LED_RT_ON;
    //        CC3000_is_Connected = false;
    //        CC3000_DHCP_done = false;
    //        memset(&ipConfig, 0, sizeof(ipConfig));
            break;
        case HCI_EVNT_WLAN_UNSOL_DHCP:
            printf("HCI_EVNT_WLAN_UNSOL_DHCP\r\n");
//
//            ipConfig.cIP[0] = pcData[3];
//            ipConfig.cIP[1] = pcData[2];
//            ipConfig.cIP[2] = pcData[1];
//            ipConfig.cIP[3] = pcData[0];
//            ipConfig.cSubnet[0] = pcData[7];
//            ipConfig.cSubnet[1] = pcData[6];
//            ipConfig.cSubnet[2] = pcData[5];
//            ipConfig.cSubnet[3] = pcData[4];
//            ipConfig.cGateway[0] = pcData[11];
//            ipConfig.cGateway[1] = pcData[10];
//            ipConfig.cGateway[2] = pcData[9];
//            ipConfig.cGateway[3] = pcData[8];
//            ipConfig.cDHCP[0] = pcData[15];
//            ipConfig.cDHCP[1] = pcData[14];
//            ipConfig.cDHCP[2] = pcData[13];
//            ipConfig.cDHCP[3] = pcData[12];
//            ipConfig.cDNS[0] = pcData[19];
//            ipConfig.cDNS[1] = pcData[18];
//            ipConfig.cDNS[2] = pcData[17];
//            ipConfig.cDNS[3] = pcData[16];
//            CC3000_printIPconfig();
//            if (ipConfig.cIP[0] != 0) {
//                // verbunden und IP erhalten
//                CC3000_DHCP_done = 1;
//
//            } else {
//                CC3000_DHCP_done = false;
//            }
            break;
        case HCI_EVNT_WLAN_UNSOL_INIT:
            printf("HCI_EVNT_WLAN_UNSOL_INIT\r\n");
            break;
        case HCI_EVNT_WLAN_TX_COMPLETE:
            printf("HCI_EVNT_WLAN_TX_COMPLETE\r\n");
            break;
        case HCI_EVNT_WLAN_ASYNC_PING_REPORT:
            printf("HCI_EVNT_WLAN_ASYNC_PING_REPORT\r\n");
            break;
        case HCI_EVNT_WLAN_KEEPALIVE:
            printf("HCI_EVNT_WLAN_KEEPALIVE\r\n");
            break;
        case HCI_EVENT_CC3000_CAN_SHUT_DOWN:
            printf("HCI_EVENT_CC3000_CAN_SHUT_DOWN\r\n");
            break;
        case HCI_EVNT_BSD_TCP_CLOSE_WAIT:
            printf("HCI_EVNT_BSD_TCP_CLOSE_WAIT\r\n");
    //        CC3000_SOCKET_WAIT_DISCONNECT = (unsigned long) *pcData;
    //        handle_HTTP_Client = 99;
            break;
        default:
            printf("Unknown event %ld: \n", event);
            break;
    }
}

//*****************************************************************************
//!  CC3000_ScanNetworks(network_t nets[])
//!
//!  @return    -1 on error else Number of visible Networks
//!             Networks are saved in parameter nets
//!
//!  @brief     Saves all visible Networks inside of the provided Parameter nets
//!             Total Amount of Networks are NETWORKS_ARRAY_MAX
//*****************************************************************************
unsigned long cc3000_scan_networks(void/*network_t nets[]*/) {
    uint32_t scan_intervals[16];
    memset(scan_intervals, 2000, 16);
    unsigned int position = 0;
    scanresults pScanRes;

    long result = wlan_ioctl_set_scan_params(1, // Scan intervall im mSek 2600 200 pro Kanal 13 Kanäle
            100, // min Verweilzeit in mSek
            100, // max Verweilzeit in mSek
            5, // Anzahl Proben
            0x1ff, // Channelmask bitweise aufwärts bis 13
            -80, // RSSI Threshold
            0, // NRS Threshold
            205, // Tx Power
            scan_intervals // pointer Array 16 Kanäle Timeout in mSek
            );

    if (result) {
        printf("FATAL SCAN ERROR\n");
    }

    vtimer_usleep(10000 * 1000);
    result = wlan_ioctl_set_scan_params(0,
            100, // min Verweilzeit in mSek
            100, // max Verweilzeit in mSek
            5, // Anzahl Proben
            0x1ff, // Channelmask bitweise aufwärts bis 13
            -80, // RSSI Threshold
            0, // NRS Threshold
            205, // Tx Power
            scan_intervals // pointer Array 16 Kanäle Timeout in mSek
            );

    if (result) {
        printf("FATAL SCAN ERROR\n");
    }

    wlan_ioctl_get_scan_results(1, (uint8_t*) &pScanRes);

    while (pScanRes.lNumNetworks > 0) {
        if ((pScanRes.lNetScanStatus == 1) & (pScanRes.lNumNetworks > 0)) {
//            nets[position].bValid = pScanRes->bValid;
//            strcpy(nets[position].strSSID, pScanRes->strSSID);
//            nets[position].uiRSSI = pScanRes->sRSSI;
//            nets[position].uiSecMode = pScanRes->sSecMode;
            position++;
            printf("+--------------------------------+\n");
            printf("Nr: %2i\nRSSI: %4i\nSSID: %s\nSecMode %1i\n",
                    position, pScanRes.sRSSI, pScanRes.strSSID,
                    pScanRes.sSecMode);
        }
        wlan_ioctl_get_scan_results(1, (uint8_t*) &pScanRes);
    }
    return position;
}

void cc3000_get_ipconfig(void)
{
    tNetappIpconfigRetArgs config;
    netapp_ipconfig(&config);
    printf("+--------------------------------+\n");
    printf("IP: %"PRIu8":%"PRIu8":%"PRIu8":%"PRIu8"\n", config.aucIP[3],config.aucIP[2],
                                                        config.aucIP[1],config.aucIP[0]);
}

void SpiOpen(gcSpiHandleRx pfRxHandler)
{
	sSpiInformation.ulSpiState = eSPI_STATE_POWERUP;

	sSpiInformation.SPIRxHandler = pfRxHandler;
    sSpiInformation.pRxPacket = (char *)wlan_rx_buffer;
	sSpiInformation.pTxPacket = (char *)wlan_tx_buffer;
	sSpiInformation.usRxPacketLength = 0;
	sSpiInformation.usTxPacketLength = 0;

    tSLInformation.WlanInterruptEnable();
}

void SpiClose(void)
{
	if (sSpiInformation.pRxPacket) {
		sSpiInformation.pRxPacket = 0;
	}
	tSLInformation.WlanInterruptDisable();
	tSLInformation.WriteWlanPin(WLAN_DISABLE);
}

void SpiResumeSpi(void)
{
    tSLInformation.WlanInterruptEnable();
}

long SpiWrite(unsigned char *pUserBuffer, unsigned short usLength)
{
    /** Padding **/
    size_t tx_len = (usLength & 0x01) ? usLength : usLength + 1;

	pUserBuffer[0] = WRITE;
	pUserBuffer[1] = ((tx_len & 0xFF00) >> 8);
	pUserBuffer[2] = (tx_len & 0x00FF);
	pUserBuffer[3] = 0;
	pUserBuffer[4] = 0;

	/** Add spi header size **/
	usLength = tx_len + CC3000_SPI_HEADER_SIZE;
    char output[usLength];

    sSpiInformation.pTxPacket = (char *)pUserBuffer;
    sSpiInformation.usTxPacketLength = usLength;

	/** If CC3000 was just powered up, we need to wait for initialization **/
    if (sSpiInformation.ulSpiState == eSPI_STATE_POWERUP) {
        while (sSpiInformation.ulSpiState != eSPI_STATE_INITIALIZED);
    }

    printf("Sending with length %d:\n", usLength);
    for (int i=0; i<usLength; i++) {
        printf("%x ", pUserBuffer[i]);
    }
    printf("\n");


    switch (sSpiInformation.ulSpiState) {
        case eSPI_STATE_INITIALIZED:
            /** First write procedure needed **/
            sSpiInformation.ulSpiState = eSPI_STATE_FIRST_WRITE;
            gpio_clear(CC3000_CS);
            hwtimer_wait(CC3000_FIRST_WRITE_WAIT);
            spi_transfer_bytes(CC3000_SPI, (char *)pUserBuffer, output, CC3000_FIRST_WRITE_SIZE);
            hwtimer_wait(CC3000_FIRST_WRITE_WAIT);
            spi_transfer_bytes(CC3000_SPI, (char *)pUserBuffer + CC3000_FIRST_WRITE_SIZE, output + CC3000_FIRST_WRITE_SIZE, usLength - CC3000_FIRST_WRITE_SIZE);

            printf("Received answer:\n");
            for (int i=0; i<usLength; i++) {
                printf("%x ", output[i]);
            }
            printf("\n");
            sSpiInformation.ulSpiState = eSPI_STATE_IDLE;
            gpio_set(CC3000_CS);
            break;
        case eSPI_STATE_IDLE:
        default:
            /** Generic write procedure **/
            tSLInformation.WlanInterruptDisable();
            sSpiInformation.ulSpiState = eSPI_STATE_WRITE;
            gpio_clear(CC3000_CS);
            tSLInformation.WlanInterruptEnable();
            if (!tSLInformation.ReadWlanInterruptPin()) {
                spi_transfer_bytes(CC3000_SPI, (char *)pUserBuffer, 0, usLength);
                sSpiInformation.ulSpiState = eSPI_STATE_IDLE;
                gpio_set(CC3000_CS);
            }
            break;
    }

    /** Blocking behavior **/
    while (eSPI_STATE_IDLE != sSpiInformation.ulSpiState);

	return 0;
}

static void wlan_write_enable_pin(unsigned char val)
{
    if (val) {
        gpio_set(CC3000_WLAN_EN);
    }
    else {
        gpio_clear(CC3000_WLAN_EN);
    }
}

static void wlan_irq_enable(void)
{
    gpio_irq_enable(CC3000_SPI_IRQ);
}

static void wlan_irq_disable(void)
{
    gpio_irq_disable(CC3000_SPI_IRQ);
}

static long wlan_read_irq_pin(void)
{
    return gpio_read(CC3000_SPI_IRQ);
}

static void wlan_irq_handler(void *args)
{
    uint16_t data_left = 0;
    uint16_t data_complete = 0;

    /** No action, if interrupt pin was cleared before **/
    if (!tSLInformation.ReadWlanInterruptPin()) {
        switch (sSpiInformation.ulSpiState) {
            case eSPI_STATE_POWERUP:
                sSpiInformation.ulSpiState = eSPI_STATE_INITIALIZED;
                break;
            case eSPI_STATE_IDLE:
                sSpiInformation.ulSpiState = eSPI_STATE_READ_IRQ;
                gpio_clear(CC3000_CS);
                hwtimer_wait(CC3000_FIRST_WRITE_WAIT);
                spi_transfer_byte(CC3000_SPI, READ, sSpiInformation.pRxPacket);
                spi_transfer_bytes(CC3000_SPI, 0, sSpiInformation.pRxPacket + 1, 9);
                data_left = ntohs(*((uint16_t *) &sSpiInformation.pRxPacket[3]));
                if (!(data_left & 0x01)) {
                    data_left++;
                }
                data_complete = CC3000_SPI_HEADER_SIZE + data_left;
                data_left -= CC3000_MIN_DATA_LENGTH;
                printf("data_left: %"PRIu16"\n", data_left);

                if (data_left > 0) {
                    spi_transfer_byte(CC3000_SPI, READ, sSpiInformation.pRxPacket + 10);
                    data_left--;
                    if (data_left > 0) {
                        spi_transfer_bytes(CC3000_SPI, 0, sSpiInformation.pRxPacket + 11, data_left);
                    }
                }

                printf("Read answer: \n");
                for (int i=0; i<data_complete; i++) {
                    printf("%x ", sSpiInformation.pRxPacket[i]);
                }
                printf("\n");
                tSLInformation.WlanInterruptDisable();
                gpio_set(CC3000_CS);
                sSpiInformation.ulSpiState = eSPI_STATE_IDLE;
                sSpiInformation.SPIRxHandler(sSpiInformation.pRxPacket + 5);
                tSLInformation.WlanInterruptEnable();
                break;
            case eSPI_STATE_WRITE:
                break;
            default:
                break;
        }
    }
}
