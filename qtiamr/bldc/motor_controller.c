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

#include "commands.h"
#include "motor_controller.h"



static struct motor_controller_s  g_motor_controller;

static int amr_mc_init(void)
{
	int ret;
	int fd;

	
	/* open imu fd */
	fd = open(MOTOR_CONTROLLER_DEV, O_RDWR);
	if (fd < 0){
		printf("amr_mc_init: open %s failed: %d\n",
									MOTOR_CONTROLLER_DEV, errno);
		return fd;
	}
	printf("amr_mc_init: opened %s\n",MOTOR_CONTROLLER_DEV);
	g_motor_controller.mc_fd = fd;

	return OK;
}




int amr_mc_task(int argc, char *argv[]){
	int ret, i;
	int fd;
	

	/** this task used to check amr motor controller if worked **/
	ret = amr_mc_init();
	if (ret != OK){
		printf("amr_mc_task: init failed\n");
	}
	
	fd = g_motor_controller.mc_fd;

	i = 3;
	while(i > 1){

		mc_init(fd);
		sleep(5);
		mc_set_speed(50, 50, fd);
		sleep(10);
		mc_set_speed(-50, 50, fd);
		sleep(10);
		mc_set_speed(0, 0, fd);
		sleep(5);
		//test_ultrasound(fd);
		usleep(500000);
	}
	amr_mc_deinit();
	return ret;
}


void  amr_mc_deinit(void)
{
	close(g_motor_controller.mc_fd);
}
