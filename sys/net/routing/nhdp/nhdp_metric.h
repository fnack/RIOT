/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     nhdp
 * @{
 *
 * @file
 * @brief       Macros for NHDP metric computation
 *
 * @author      Fabian Nack <nack@inf.fu-berlin.de>
 */

#ifndef NHDP_METRIC_H_
#define NHDP_METRIC_H_

#include "rfc5444/rfc5444.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NHDP_METRIC_TIMER           (5445)

#define NHDP_LMT_HOP_COUNT          (0)
#define NHDP_LMT_ETX                (1)
#define NHDP_LMT_DAT                (2)
#define NHDP_METRIC                 NHDP_LMT_DAT

#define NHDP_METRIC_UNKNOWN         (0)
#define NHDP_METRIC_MINIMUM         (RFC5444_METRIC_MIN)
#define NHDP_METRIC_MAXIMUM         (RFC5444_METRIC_MAX)

#define NHDP_Q_MEM_LENGTH           (64)
#define NHDP_SEQNO_RESTART_DETECT   (256)

#define NHDP_KD_LM_INC              (0x8000)
#define NHDP_KD_LM_OUT              (0x4000)
#define NHDP_KD_NM_INC              (0x2000)
#define NHDP_KD_NM_OUT              (0x1000)

#define NHDP_PDR_MAX                (100000)
#define NHDP_PDR_MIN                (NHDP_METRIC_UNKNOWN)

#define DAT_MEMORY_LENGTH           (NHDP_Q_MEM_LENGTH)
#define DAT_REFRESH_INTERVAL        (1)
#define DAT_HELLO_TIMEOUT_FACTOR    (1.2)
#define DAT_MINIMUM_BITRATE         (1000)
#define DAT_MAXIMUM_LOSS            (8)
#define DAT_CONSTANT                (16777216)

#ifdef __cplusplus
}
#endif

#endif /* NHDP_METRIC_H_ */
