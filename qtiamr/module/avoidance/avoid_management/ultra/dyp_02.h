/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/
#ifndef _MODULE_DYP_02_H
#define _MODULE_DYP_02_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Public function prototypes
 ****************************************************************************/
int rs485_ultra_init(uint8_t addr, int fd);
int rs485_ultra_raw_dist(uint8_t addr, int fd);

#endif
