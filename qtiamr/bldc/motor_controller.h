/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/


#ifndef __APP_QTIAMR_MOTOR_CONTROLLER_H
#define __APP_QTIAMR_MOTOR_CONTROLLER_H
/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <nuttx/fs/fs.h>


#ifdef CONFIG_APP_QTIAMR

#define MC_STACK_PRIORITY    100
#define MC_STACK_STACKSIZE  (2048)



#define MOTOR_CONTROLLER_DEV  "/dev/ttyS1"

struct motor_controller_s {
	int mc_fd;
	uint8_t bus_mode;	/* 0: can bus ; 1:rs485 bus  */
};

static int amr_mc_init(void);

int amr_mc_task(int argc, char *argv[]);
void  amr_mc_deinit(void);


#endif
#endif
