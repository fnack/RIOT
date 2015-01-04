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

#include "cc3000/cc3000_common.h"
#include "cc3000/socket.h"
#include "cc3000/cc3000_spi.h"
#include "cc3000/hci.h"
#include "hwtimer.h"
#include "board.h"
#include "vtimer.h"

#include "periph/spi.h"
#include "periph/gpio.h"

// SPI CMD
#define READ                3
#define WRITE               1

unsigned char wlan_rx_buffer[CC3000_RX_BUFFER_SIZE];
unsigned char wlan_tx_buffer[CC3000_TX_BUFFER_SIZE];

spi_information_t sSpiInformation;

void cc3000_irq_handler(void *args)
{
    unsigned short data_to_recv = 0;
    char tSpiReadHeader[] = {READ, 0, 0, 0, 0};
    LED_GREEN_ON;

    /** No action, if interrupt pin was cleared before **/
    if (!tSLInformation.ReadWlanInterruptPin()) {
        switch (sSpiInformation.ulSpiState) {
            case eSPI_STATE_POWERUP:
                sSpiInformation.ulSpiState = eSPI_STATE_INITIALIZED;
                break;
            case eSPI_STATE_IDLE:
                sSpiInformation.ulSpiState = eSPI_STATE_READ_IRQ;
                gpio_clear(CC3000_CS);
                spi_transfer_bytes(CC3000_SPI, tSpiReadHeader, sSpiInformation.pRxPacket, CC3000_SPI_HEADER_SIZE);

                uint16_t *pnetlen = (uint16_t *) &sSpiInformation.pRxPacket[3];
                data_to_recv = ntohs(*pnetlen);
                if (!(data_to_recv & 1)) {
                    data_to_recv++;
                }
                // TODO: Check if buffer size is big enough
                spi_transfer_bytes(CC3000_SPI, 0, sSpiInformation.pRxPacket, data_to_recv);

                tSLInformation.WlanInterruptDisable();
                gpio_set(CC3000_CS);
                sSpiInformation.ulSpiState = eSPI_STATE_IDLE;
                sSpiInformation.SPIRxHandler(sSpiInformation.pRxPacket);
                tSLInformation.WlanInterruptEnable();
                break;
            case eSPI_STATE_WRITE:
                break;
            default:
                break;
        }
    }
}

void init_CC3000_SPI(void)
{
    /** configure CS pin **/
    gpio_init_out(CC3000_CS, GPIO_NOPULL);
    /** chip select to high (not active) **/
    gpio_set(CC3000_CS);

    /** configure CC3000 WLAN pin **/
	gpio_init_out(CC3000_WLAN_EN, GPIO_NOPULL);
    /** disable CC3000 WLAN for now **/
	gpio_clear(CC3000_WLAN_EN);

	/** initialize CC3000 SPI **/
	spi_init_master(CC3000_SPI, SPI_CONF_SECOND_RISING, SPI_SPEED_10MHZ);

	/** configure interrupt for CC3000 **/
	//gpio_init_int(CC3000_SPI_IRQ, GPIO_PULLUP, GPIO_FALLING, cc3000_irq_handler, 0);
}

void SpiOpen(gcSpiHandleRx pfRxHandler)
{
	sSpiInformation.ulSpiState = eSPI_STATE_POWERUP;

	sSpiInformation.SPIRxHandler = pfRxHandler;
    sSpiInformation.pRxPacket = (char *)wlan_rx_buffer;
	sSpiInformation.pTxPacket = (char *)wlan_tx_buffer;
	sSpiInformation.usRxPacketLength = 0;
	sSpiInformation.usTxPacketLength = 0;

    gpio_clear(CC3000_WLAN_EN);
    vtimer_usleep(1000000);
    WlanInterruptEnable();
    gpio_set(CC3000_WLAN_EN);

    while(gpio_read(CC3000_SPI_IRQ) != 0) {
        LED_RED_ON;
    }
    LED_RED_OFF;
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

    sSpiInformation.pTxPacket = (char *)pUserBuffer;
    sSpiInformation.usTxPacketLength = usLength;

	/** If CC3000 was just powered up, we need to wait for initialization **/
    if (sSpiInformation.ulSpiState == eSPI_STATE_POWERUP) {
        while (sSpiInformation.ulSpiState != eSPI_STATE_INITIALIZED);
    }

    switch (sSpiInformation.ulSpiState) {
        case eSPI_STATE_INITIALIZED:
            /** First write procedure needed **/
            sSpiInformation.ulSpiState = eSPI_STATE_FIRST_WRITE;
            gpio_clear(CC3000_CS);
            // TODO: Really no waiting for interrupt?
            hwtimer_wait(CC3000_FIRST_WRITE_WAIT);
            spi_transfer_bytes(CC3000_SPI, (char *)pUserBuffer, 0, CC3000_FIRST_WRITE_SIZE);
            hwtimer_wait(CC3000_FIRST_WRITE_WAIT);
            spi_transfer_bytes(CC3000_SPI, (char *)pUserBuffer + CC3000_FIRST_WRITE_SIZE, 0, usLength - CC3000_FIRST_WRITE_SIZE);
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

void WriteWlanPin(unsigned char val)
{
    if (val) {
        gpio_set(CC3000_WLAN_EN);
    }
    else {
        gpio_clear(CC3000_WLAN_EN);
    }
}

void WlanInterruptEnable(void)
{
    /** leaves room for optimization **/
    gpio_init_int(CC3000_SPI_IRQ, GPIO_PULLUP, GPIO_FALLING, &cc3000_irq_handler, 0);
    //gpio_init_in(CC3000_SPI_IRQ, GPIO_PULLUP);
}

void WlanInterruptDisable(void)
{
    gpio_irq_disable(CC3000_SPI_IRQ);
}

long ReadWlanInterruptPin(void)
{
    return gpio_read(CC3000_SPI_IRQ);
}
