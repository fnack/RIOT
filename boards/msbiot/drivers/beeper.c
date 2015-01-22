/*
 * Copyright (C) 2015 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     board_msbiot
 * @{
 *
 * @file
 * @brief       Beeper functionality implementation
 *
 * @author      Fabian Nack <nack@inf.fu-berlin.de>
 *
 * @}
 */

#include "board.h"
#include "periph/pwm.h"
#include "vtimer.h"

void msbiot_beeper_init(unsigned int frequency, unsigned int resolution)
{
    pwm_init(PWM_0, PWM_LEFT, frequency, resolution);
    pwm_set(BEEPER_PWM, BEEPER_CHANNEL, 4);
    pwm_stop(PWM_0);
}

void beep(uint32_t milliseconds)
{
    pwm_start(PWM_0);
    vtimer_usleep(milliseconds * 1000);
    pwm_stop(PWM_0);
}

void msbiot_beeper_start(void)
{
    pwm_start(PWM_0);
}

void msbiot_beeper_stop(void)
{
    pwm_stop(PWM_0);
}
