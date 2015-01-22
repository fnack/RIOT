/*
 * Copyright (C) 2015 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

#include <stdint.h>
#include "periph/gpio.h"
#include "periph/pwm.h"

#ifndef BEEPER_H_
#define BEEPER_H_

void msbiot_beeper_init(unsigned int frequency, unsigned int resolution);

void beep(uint32_t milliseconds);

void msbiot_beeper_start(void);

void msbiot_beeper_stop(void);

#endif /* BEEPER_H_ */
