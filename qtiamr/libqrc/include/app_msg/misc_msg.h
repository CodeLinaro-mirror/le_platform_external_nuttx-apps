/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#ifndef __MISC_MSG_H
#define __MISC_MSG_H

#include <stdio.h>
#include <stdint.h>

/* MISC pipe name */
#define MISC_PIPE "misc"

struct watchdog_msg
{
  uint8_t count;
}__attribute__((aligned(4)));

#endif