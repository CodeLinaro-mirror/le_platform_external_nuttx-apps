
/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#ifndef __MOTOR_8015D_H
#define __MOTOR_8015D_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum driver_bus_mode
{
    BUS_CANOPEN = 0x0,
    BUS_RS485,
};

struct motor_8015d_s
{
  int driver_fd;
  bool bus_mode;
  pthread_mutex_t motor_mutex;
  enum control_mode_e mode;
  bool initialized;
}__attribute__((aligned(4)));

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* motor management IF */
extern struct motor_hal_ops 8015d_ops;
extern struct motor_8015d_s g_8015d;

#endif /* __MOTOR_8015D_H */