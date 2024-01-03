/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#ifndef __IMU_MSG_H
#define __IMU_MSG_H

#include <stdio.h>
#include <stdint.h>

/* IMU pipe name */
#define IMU_PIPE "imu"

/* just imu data, no msg type */

struct imu_msg_s
{
  float xa;
  float ya;
  float za;
  float xg;
  float yg;
  float zg;
}__attribute__((align(4)));

#endif