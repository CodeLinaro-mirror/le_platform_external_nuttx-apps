/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <sched.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include "main.h"
#include "imu.h"
#include "motor_controller.h"


#define TASK_NUM	(2)

struct amr_task_s {
	const char  *name;
	uint32_t	priority;
	uint32_t	stack_size;
	int	(*task_func)(int argc, char *argv[]);
	char	*argv;
};

static struct amr_task_s  tasks[TASK_NUM] = {
	{"motor_control", MC_STACK_PRIORITY, MC_STACK_STACKSIZE, amr_mc_task, NULL},
	{"imu_task", CAR_IMU_PRIORITY, CAR_IMU_STACKSIZE, imu_task, NULL},
};

int main(int argc, FAR char *argv[])
{
	int ret;
	uint8_t index;
	int errcode;

/******************** start tasks **********************************/
	for (index = 0; index < TASK_NUM; index ++) {
		ret = task_create(tasks[index].name, tasks[index].priority,
						tasks[index].stack_size, tasks[index].task_func,
						tasks[index].argv);
		if (ret < 0) {
			errcode = errno;
			printf("car_main: ERROR: Failed to start %s: %d\n",
					tasks[index].name,errcode);
			return EXIT_FAILURE;
		}
		printf("amr_main: Starting the task %s\n",tasks[index].name);
	}

	printf("main: app-qcomamr  main started\n");

	return EXIT_SUCCESS;
}

