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
#include "posix_io.h"
#include "shell.h"
#include "shell_commands.h"
#include "board_uart0.h"
#include "nhdp.h"

static ipv6_addr_t my_address, all_n_address;

static void _nhdp_sh_start(int argc, char **argv)
{
    (void) argc;
    (void) argv;
    int result;

    /* Completely set NHDP up */
    nhdp_init();
    nhdp_start();

    result = nhdp_register_if_default(1, (uint8_t*)my_address.uint8, 16, AF_INET6, 100);
    if (result == 0) {
        printf("[SUCCESS] NHDP was successfully started\n");
    }
    else if (result == -1) {
        printf("[ERROR] Insufficient memory for NHDP start\n");
    }
    else {
        printf("[ERROR] NHDP is already started\n");
    }
}

static void _nhdp_sh_print_packet(int argc, char **argv)
{
    (void) argc;
    (void) argv;

    nhdp_print_last_packet();
}

static void _nhdp_sh_print_state(int argc, char **argv)
{
    (void) argc;
    (void) argv;

    nhdp_print_state();
}

const shell_command_t shell_commands[] = {
    {"nhdp_start", "Start NHDP for defaulttransceiver interface", _nhdp_sh_start},
    {"nhdp_print_packet", "Print last send NHDP packet", _nhdp_sh_print_packet},
    {"nhdp_print_state", "Print NHDP's information base state", _nhdp_sh_print_state},
    {NULL, NULL, NULL}
};

int main(void)
{
    puts("NHDP example application");

    vtimer_init();

    /* Configure IPv6 address for defaulttransceiver */
    net_if_set_src_address_mode(0, NET_IF_TRANS_ADDR_M_SHORT);
    sixlowpan_lowpan_init_interface(0);
    ipv6_addr_set_all_nodes_addr(&all_n_address);
    ipv6_net_if_get_best_src_addr(&my_address, &all_n_address);

    /* start shell */
    posix_open(uart0_handler_pid, 0);

    shell_t shell;
    shell_init(&shell, shell_commands, UART0_BUFSIZE, uart0_readc, uart0_putc);

    shell_run(&shell);
    return 0;
}
