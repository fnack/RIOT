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
#include "stm32f415xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include "misc.h"

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
//    /** configure CS pin **/
//    gpio_init_out(CC3000_CS, GPIO_NOPULL);
//    /** chip select to high (not active) **/
//    gpio_set(CC3000_CS);
//
//    /** configure CC3000 WLAN pin **/
//	gpio_init_out(CC3000_WLAN_EN, GPIO_NOPULL);
//    /** disable CC3000 WLAN for now **/
//	gpio_clear(CC3000_WLAN_EN);
//
//	/** initialize CC3000 SPI **/
//	spi_init_master(CC3000_SPI, SPI_CONF_SECOND_RISING, SPI_SPEED_10MHZ);
//
//	/** configure interrupt for CC3000 **/
//	//gpio_init_int(CC3000_SPI_IRQ, GPIO_PULLUP, GPIO_FALLING, cc3000_irq_handler, 0);

    unsigned long ulSpiIRQState;
    //=========================================================================
    // Clocksystem einschalten
    //=========================================================================
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;

    //=========================================================================
    // CC3000 PWR_EN Setup
    //=========================================================================

    // PWR_EN
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;      //TODO WLAN Enable
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);          //TODO WLAN Enable

    // WLAN PIN = 0 - CC3000 ausgeschaltet
    GPIO_ResetBits(GPIOC, GPIO_Pin_13);             //TODO WLAN Enable

    //=========================================================================
    // CC3000 GPIO Setup
    //=========================================================================

    // CS - PB12
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // CS PIN = 1 - SPI nicht aktiv
    GPIO_SetBits(GPIOC, GPIO_Pin_1);

    // CLK - PB10
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_SPI2);

    // MOSI - PC3
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource3, GPIO_AF_SPI2);

    // MISO - PC2
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource2, GPIO_AF_SPI2);

    //=========================================================================
    // CC3000 SPI2 Setup
    //=========================================================================
    // Taktquelle RCC_APB1Periph_SPI2 mit 42Mhz geteilt durch 4 = SPI-CLK 10,5MHz

    SPI_InitTypeDef SPI_InitStructure;

    // SPI2
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; //SPI_CPHA_1Edge
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; //SPI_BaudRatePrescaler_16
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_Init(SPI2, &SPI_InitStructure);

    // Enable SPI
    SPI_Cmd(SPI2, ENABLE);

    //=========================================================================
    // CC3000 IRQ Setup
    //=========================================================================

    // SPI_IRQ
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Interrupt Konfiguration
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource10);

    EXTI_InitTypeDef EXTI_SPIGPIO_InitStructure;
    EXTI_SPIGPIO_InitStructure.EXTI_Line = EXTI_Line10;
    EXTI_SPIGPIO_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_SPIGPIO_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_SPIGPIO_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_Init(&EXTI_SPIGPIO_InitStructure);

    NVIC_InitTypeDef NVIC_InitSPIGPIOStructure;

    NVIC_InitSPIGPIOStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitSPIGPIOStructure.NVIC_IRQChannel = EXTI15_10_IRQn; //EXTI15_10_IRQn;
    NVIC_InitSPIGPIOStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
    NVIC_InitSPIGPIOStructure.NVIC_IRQChannelSubPriority = 0x01;
    NVIC_Init(&NVIC_InitSPIGPIOStructure);

    //=============================================================================
    // Anfangssynchronisation mit dem CC3000 Modul
    //=============================================================================
    // Warten ... IRQ Status lesen ... und dann Modul einschalten
    tSLInformation.WriteWlanPin(WLAN_DISABLE);
    vtimer_usleep(100000);
    ulSpiIRQState = tSLInformation.ReadWlanInterruptPin();
    tSLInformation.WriteWlanPin(WLAN_ENABLE);

    // wenn IRQ 1 war
    if ( ulSpiIRQState != 0 )
        {

        // und warten auf IRQ-Flanke HL
        while(tSLInformation.ReadWlanInterruptPin() != 0) { ; }

        }

    // wenn IRQ 0 war
    else
        {

        //dann WLAN-EN HLH-Flanke mit kurzem Warten
        tSLInformation.WriteWlanPin(WLAN_DISABLE);
        vtimer_usleep(100000);
        tSLInformation.WriteWlanPin(WLAN_ENABLE);

        // und warten auf IRQ-Flanke LH
        while(tSLInformation.ReadWlanInterruptPin() != 0) { ; }

        // dann nochmal WLAN-EN HLH-Flanke mit kurzem Warten
        tSLInformation.WriteWlanPin(WLAN_DISABLE);
        vtimer_usleep(100000);
        tSLInformation.WriteWlanPin(WLAN_ENABLE);

        // und warten auf IRQ-Flanke HL
        while(tSLInformation.ReadWlanInterruptPin() == 0) { ; }

        }
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
