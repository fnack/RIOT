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

radio_address_t address;
ipv6_addr_t my_address;

#define STACKSZ_TEST (KERNEL_CONF_STACKSIZE_MAIN)

static kernel_pid_t nhdp_pid = KERNEL_PID_UNDEF;

static ipv6_addr_t *_get_next_hop(ipv6_addr_t *addr)
{
    return NULL;
}

static void _nhdp_sh_init(int argc, char **argv)
{
    if (argc == 2) {
        if (nhdp_pid == KERNEL_PID_UNDEF) {

            net_if_set_hardware_address(0, (uint16_t) atoi(argv[1]));
            sixlowpan_lowpan_init_interface(0);

            /* need link local prefix to query _our_ corresponding address  */
            ipv6_iface_set_routing_provider(_get_next_hop);

            ipv6_init_as_router();

            nhdp_init();
            nhdp_pid = nhdp_start();
        }
    }

}

static void _nhdp_sh_reg_if(int argc, char **argv)
{
    ipv6_addr_t dest;
    if (argc == 2) {
        if (!inet_pton(AF_INET6, argv[1], &dest)) {
            printf("Wrong IPv6 address\n");
            return;
        }

        if (nhdp_pid != KERNEL_PID_UNDEF) {
            nhdp_register_if_default(0, (uint8_t*)dest.uint8, 16, AF_INET6, 120);
        }
    }
}

static void _nhdp_sh_print(int argc, char **argv)
{
    print_packet();
}

const shell_command_t shell_commands[] = {
    {"nhdp_init", "Initialize NHDP", _nhdp_sh_init},
    {"nhdp_if", "Register IF at NHDP", _nhdp_sh_reg_if},
    {"print", "Print last packet", _nhdp_sh_print},
    {NULL, NULL, NULL}
};

int main(void)
{
    puts("NHDP Link Layer example");

    vtimer_init();

    /* start shell */
    posix_open(uart0_handler_pid, 0);

    net_if_set_src_address_mode(0, NET_IF_TRANS_ADDR_M_SHORT);
    address = net_if_get_hardware_address(0);

    shell_t shell;
    shell_init(&shell, shell_commands, UART0_BUFSIZE, uart0_readc, uart0_putc);

    shell_run(&shell);
    return 0;
}
