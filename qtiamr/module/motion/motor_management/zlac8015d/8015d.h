
/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#ifndef __MOTOR_ZLAC_8015d_H
#define __MOTOR_ZLAC_8015d_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "motion_msg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum driver_bus_mode_e
{
  BUS_CANOPEN = 0x0,
  BUS_RS485,
};

struct motor_zlac_8015d_s
{
  int driver_fd;
  bool bus_mode;
  pthread_mutex_t motor_mutex;
  enum control_mode_e mode;
  bool initialized;
} __attribute__((aligned(4)));

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* motor management IF */
extern struct motor_hal_ops zlac_8015d_ops;
extern struct motor_zlac_8015d_s g_zlac_8015d;

#endif /* __MOTOR_ZLAC_8015d_H */