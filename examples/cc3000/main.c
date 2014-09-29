/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       Hello World application
 *
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 * @author      Ludwig Ortmann <ludwig.ortmann@fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>
#include "cc3000.h"
#include "cc3000/wlan.h"
#include "cc3000/socket.h"
#include "periph/gpio.h"
#include "vtimer.h"
#include "board.h"

char* ssid = "msbiotnw";
char* wLANkey = "msbiot123";
char* message = "Hello World from CC3000";

int main(void)
{
    sockaddr_in destin;
    destin.sin_family = AF_INET;
    destin.sin_port = htons(44444);
    destin.sin_addr.s_addr = htonl(0xC0A800FF);
    uint8_t ip[4];
    ip[0] = 192;
    ip[1] = 168;
    ip[2] = 0;
    ip[3] = 254;
    puts("Hello World!");

    vtimer_init();

    printf("You are running RIOT on a(n) %s board.\n", RIOT_BOARD);
    printf("This board features a(n) %s MCU.\n", RIOT_MCU);

    cc3000_init();
    wlan_start(0);

    wlan_set_event_mask(HCI_EVNT_WLAN_KEEPALIVE);
    cc3000_scan_networks();

    LED_GREEN_ON;
    vtimer_usleep(10000000);
    int result = wlan_connect(3, ssid, strlen(ssid), 0, wLANkey, strlen(wLANkey));
    printf("The returned result was %d\n", result);
    vtimer_usleep(10000000);
    LED_RED_ON;
    printf("WLAN Status is %d\n", wlan_ioctl_statusget());
    cc3000_get_ipconfig();
    netapp_ping_send((uint32_t*)&ip, 5, 10, 3000);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    printf("Socket handle is %d\n", sock);

    while (1) {
    int sended = sendto(sock, message, strlen(message), 0, (sockaddr*)&destin, sizeof(destin));
    printf("Sended bytes were %d\n", sended);
    vtimer_usleep(3000000);
    }

    return 0;
}
