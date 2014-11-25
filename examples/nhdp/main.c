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

#include "nhdp.h"
#include "netdev/default.h"
#include "netdev/base.h"
#include "nomac.h"
#include "cc110x.h"
#include "board.h"

#define STACKSZ_TEST (KERNEL_CONF_STACKSIZE_DEFAULT)

static netdev_t *dev;
static char test_stack[STACKSZ_TEST];
static kernel_pid_t nomac_pid;
static uint8_t address;
static uint8_t is_manet;
static uint8_t addr_type;
static size_t addr_size;
static uint16_t max_pl_size;

int main(void)
{
    puts("Hello World!");

    printf("You are running RIOT on a(n) %s board.\n", RIOT_BOARD);
    printf("This board features a(n) %s MCU.\n", RIOT_MCU);

    address = 1;
    addr_size = 1;
    is_manet = 1;
    addr_type = AF_CC110X;
    max_pl_size = 256;
    
    dev = NETDEV_DEFAULT;
    dev->driver->init(dev);
    cc110x_set_channel(1);
    cc110x_set_address(address);
    //cc110x_set_monitor(1);
    nomac_init_module();
    vtimer_init();
    
    nomac_pid = nomac_init(test_stack, STACKSZ_TEST, PRIORITY_MAIN - 1, "mac_test", dev);
    

    nhdp_init();
    nhdp_start();
    nhdp_register_if_default(nomac_pid, &address, addr_size, addr_type, max_pl_size);

    return 0;
}
