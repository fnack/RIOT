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
 * @brief       LPM example application
 *
 * @author      Fabian Nack <nack@inf.fu-berlin.de>
 *
 * @}
 */

#ifndef WKUP_GPIO
#error "WKUP_GPIO not defined"
#endif

#include <stdio.h>

#include "cpu-conf.h"
#include "periph/gpio.h"
#include "arch/lpm_arch.h"
#include "vtimer.h"

static void button_handler(void *args)
{
    lpm_arch_awake();
}

int main(void)
{
    puts("LPM example application");

    vtimer_init();

    gpio_init_int(WKUP_GPIO, GPIO_NOPULL, GPIO_RISING, &button_handler, 0);

    /* Check whether the device was reset from standby */
    if (PWR->CSR & PWR_CSR_SBF) {
        puts("LPM tests successful");
        PWR->CR |= PWR_CR_CSBF;
        return 0;
    }

    lpm_arch_init();

    puts("Entering LPM_IDLE");
    vtimer_usleep(100000);
    lpm_arch_set(LPM_IDLE);
    puts("Successfully awoke from LPM_IDLE");

    puts("Entering LPM_SLEEP");
    vtimer_usleep(100000);
    lpm_arch_set(LPM_SLEEP);
    puts("Successfully awoke from LPM_SLEEP");

    puts("Entering LPM_POWERDOWN");
    vtimer_usleep(100000);
    lpm_arch_set(LPM_POWERDOWN);

    /* Never reached */
    return 0;
}
