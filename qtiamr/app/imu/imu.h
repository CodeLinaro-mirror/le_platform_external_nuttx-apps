/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#ifndef __IMU_H
#define __IMU_H

/* IMU application header */
#include "imu_msg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define IMU_DATA_LEN_7          (7)

/****************************************************************************
 * Public Types
 ****************************************************************************/
struct imu_pkg_s{
  int   fd_imu;
  /* temp,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z */
  uint8_t raw_data[IMU_DATA_LEN_7 * 2];
  struct imu_data_s deviation_data;
  struct imu_data_s data;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
int imu_task(int argc, char *argv[]);


#endif