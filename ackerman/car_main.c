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

#include "car_main.h"
#include "motion_task.h"
#include "ros_com.h"
#include "car_imu.h"
#include "car_adc.h"
#include "oled.h"
#include "odom.h"
#include "rc.h"

#define TASK_NUM	(6)

struct ackerman_task_s {
	const char  *name;
	uint32_t	priority;
	uint32_t	stack_size;
	int	(*task_func)(int argc, char *argv[]);
	char	*argv;
};

static struct ackerman_task_s  tasks[TASK_NUM] = {
	{"motion_task", CAR_MOTION_PRIORITY, CAR_MOTION_STACKSIZE, motion_task, NULL},
	{"ros_com_task", CAR_ROS_COM_PRIORITY, CAR_ROS_COM_STACKSIZE, ros_com_task, NULL},
	{"imu_task", CAR_IMU_PRIORITY, CAR_IMU_STACKSIZE, imu_task, NULL},
	{"oled_task", CAR_OLED_PRIORITY, CAR_OLED_STACKSIZE, oled_task, NULL},
	{"odom_task", CAR_ODOM_PRIORITY, CAR_ODOM_STACKSIZE, odom_task, NULL},
	{"rc_task", CAR_RC_PRIORITY, CAR_RC_STACKSIZE, rc_task, NULL},
};

int main(int argc, FAR char *argv[])
{
	int ret;
	uint8_t index;
	int errcode;

	/* init servo adc */
	car_adc_init(ADC_SERVO_POWER);
	car_adc_init(ADC_CAR_MODE);

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
		printf("car_main: Starting the task %s\n",tasks[index].name);
	}

	printf("main: ackerman main started\n");
	return EXIT_SUCCESS;
}

