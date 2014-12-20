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
 * @brief       NHDP application
 *
 * @author      Fabian Nack <nack@inf.fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>
#include <string.h>

#include "thread.h"
#include "nhdp.h"
#include "netdev/default.h"
#include "netdev/base.h"
#include "nomac.h"
#include "posix_io.h"
#include "shell.h"
#include "shell_commands.h"
#include "board_uart0.h"

#define STACKSZ_NOMAC (KERNEL_CONF_STACKSIZE_DEFAULT)

static netdev_t *dev;
static char nomac_stack[STACKSZ_NOMAC];
static kernel_pid_t nomac_pid;
static kernel_pid_t nhdp_pid = KERNEL_PID_UNDEF;
static uint8_t channel = 1;

static void _nhdp_sh_init(int argc, char **argv)
{
    if (argc != 2) {
        printf("Usage: %s <addr>\n", argv[0]);
        return;
    }

    if (nhdp_pid != KERNEL_PID_UNDEF) {
        printf("[ERROR] Can't initialize twice");
        return;
    }

    /* Start NHDP */
    nhdp_init();
    nhdp_pid = nhdp_start();

    /* Register IF with given HW address */
    if (nhdp_pid != KERNEL_PID_UNDEF) {
        uint16_t address = atoi(argv[1]);
        printf("Started NHDP\n");
        dev->driver->set_option(dev, NETDEV_OPT_ADDRESS, &address, sizeof(uint8_t));
        nhdp_register_if_default(nomac_pid, (uint8_t*)&address, sizeof(uint16_t), AF_CC110X, 120);
        printf("Registered IF with addr: %"PRIu16"\n", address);
    }
}

static void _nhdp_sh_print(int argc, char **argv)
{
    print_packet();
}

const shell_command_t shell_commands[] = {
    {"nhdp_init", "Initialize NHDP with IF with given HW address", _nhdp_sh_init},
    {"print", "Print last packet", _nhdp_sh_print},
    {NULL, NULL, NULL}
};

int main(void)
{
    puts("NHDP Link Layer example");

    nomac_init_module();
    vtimer_init();

    /* start shell */
    posix_open(uart0_handler_pid, 0);
    
    dev = NETDEV_DEFAULT;
    nomac_pid = nomac_init(nomac_stack, sizeof(nomac_stack), PRIORITY_MAIN - 1, "mac_test", dev);
    dev->driver->set_option(dev, NETDEV_OPT_CHANNEL, &channel, sizeof(uint8_t));

    shell_t shell;
    shell_init(&shell, shell_commands, UART0_BUFSIZE, uart0_readc, uart0_putc);

    shell_run(&shell);
    return 0;
}
