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
#include <syslog.h>

#include "main.h"
#include "imu.h"
#include "ultrasound.h"

#include "motion_task.h"
#include "rc.h"
#include "ros_com.h"
#include "auto_charging.h"


struct amr_task_s {
	const char  *name;
	uint32_t	priority;
	uint32_t	stack_size;
	int	(*task_func)(int argc, char *argv[]);
	char	*argv;
};

struct task_signal_info task_info[TASK_NUM] =
{
  {0,SIGUSR1},
  {0,SIGUSR1},
  {0,SIGUSR1},
  {0,SIGUSR1},
  {0,SIGUSR1},
};
//TODO: get_task_info

/*modify enum task_list when modify the sequence of the task*/
static struct amr_task_s  tasks[TASK_NUM] = {
	{"imu_task", AMR_IMU_PRIORITY, AMR_IMU_STACKSIZE, imu_task, NULL},
	{"rc_task", CAR_RC_PRIORITY, CAR_RC_STACKSIZE, rc_task, NULL},
	{"motion_task", MOTION_PRIORITY, MOTION_STACKSIZE, motion_task, NULL},
	{"ros_com_task", ROS_COM_PRIORITY, ROS_COM_STACKSIZE, ros_com_task, NULL},
	{"ultrasound_task", CAR_ULTRASOUND_PRIORITY, CAR_ULTRASOUND_STACKSIZE, ultrasound_task, NULL },
	{"auto_charging_task", CAR_AUTO_CHARGING_PRIORITY, CAR_AUTO_CHARGING_STACKSIZE, auto_charging_task, NULL },
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
		    //syslog(LOG_INFO,"car_main: ERROR: Failed to start %s: %d\n", tasks[index].name,errcode);
		    return EXIT_FAILURE;
		}
		//syslog(LOG_INFO, "amr_main: Starting the pid %d\n", ret);
		task_info[index].task_id = ret;
	}

	//syslog(LOG_INFO, "main: app-qtiamr  main started\n");

	return EXIT_SUCCESS;
}
