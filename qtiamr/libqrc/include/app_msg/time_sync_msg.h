/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#ifndef __TIME_SYNC_MSG_H
#define __TIME_SYNC_MSG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <time.h>

#define TIME_SYNC_PIPE "time"

enum time_sync_msg_type_e
{
  TIME_LOOP,
  GET_TIME,
  SET_TIME,
};

struct time_sync_msg_s
{
  enum time_sync_msg_type_e type;
  struct timespec ts;
} __attribute__((aligned(4)));

#ifdef __cplusplus
}
#endif

#endif  // __TIME_SYNC_MSG_H
