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

#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <string.h>
#include <inttypes.h>

#include <nuttx/timers/pwm.h>
#include <nuttx/ioexpander/gpio.h>

#include "car_pwm.h"
#include "car_encoder.h"
#include "motion_task.h"


static struct  motion_gpio_s  g_motion_gpio;

/* Motor direction control */
int gpio_motion_init(struct  motion_gpio_s **ackerman){
	int ret;
	int fd;

	if (ackerman == NULL)
		return ERROR;

	/* Open pins driver */
	fd = open(MOTOR_A_GP1, O_RDWR);
	if (fd < 0) {
		int errcode = errno;
		printf("ERROR:gpio_motion_init: Failed to open %s: %d\n", MOTOR_A_GP1, errcode);
		goto errout_with_dev;
    }
	g_motion_gpio.gpio_a1_fd = fd;
	fd = open(MOTOR_A_GP2, O_RDWR);
	if (fd < 0) {
		int errcode = errno;
		printf("ERROR:gpio_motion_init: Failed to open %s: %d\n", MOTOR_A_GP2, errcode);
		goto errout_with_dev;
    }
	g_motion_gpio.gpio_a2_fd = fd;
	fd = open(MOTOR_B_GP1, O_RDWR);
	if (fd < 0) {
		int errcode = errno;
		printf("ERROR:gpio_motion_init: Failed to open %s: %d\n", MOTOR_B_GP1, errcode);
		goto errout_with_dev;
    }
	g_motion_gpio.gpio_b1_fd = fd;
	fd = open(MOTOR_B_GP2, O_RDWR);
	if (fd < 0) {
		int errcode = errno;
		printf("ERROR:gpio_motion_init: Failed to open %s: %d\n", MOTOR_B_GP2, errcode);
		goto errout_with_dev;
    }
	g_motion_gpio.gpio_b2_fd = fd;

	/* default pins value is 0 */
	fd = g_motion_gpio.gpio_a1_fd;
	ret = ioctl(fd, GPIOC_WRITE, (unsigned long)false);
	if (ret < 0) {
		int errcode = errno;
		fprintf(stderr,
			"ERROR: Failed to write: %d\n", errcode);

                 goto errout_with_dev;
	}
	fd = g_motion_gpio.gpio_a2_fd;
	ret = ioctl(fd, GPIOC_WRITE, (unsigned long)false);
	if (ret < 0) {
		int errcode = errno;
		fprintf(stderr,
			"ERROR: Failed to write: %d\n", errcode);

                 goto errout_with_dev;
	}

	fd = g_motion_gpio.gpio_b1_fd;
	ret = ioctl(fd, GPIOC_WRITE, (unsigned long)false);
	if (ret < 0) {
		int errcode = errno;
		fprintf(stderr,
			"ERROR: Failed to write: %d\n", errcode);

                 goto errout_with_dev;
	}

	fd = g_motion_gpio.gpio_b2_fd;
	ret = ioctl(fd, GPIOC_WRITE, (unsigned long)false);
	if (ret < 0) {
		int errcode = errno;
		fprintf(stderr,
			"ERROR: Failed to write: %d\n", errcode);

                 goto errout_with_dev;
	}


	*ackerman = &g_motion_gpio;
	return OK;

errout_with_dev:
	close(fd);
	return ERROR;
}

/* gpio set/reset to control motor direction */
int car_motor_direction(struct  motion_gpio_s *ackerman, int32_t V_right, int32_t V_left){
	int ret_1,ret_2,ret_3,ret_4;
	bool R1,R2,L1,L2;

	if (V_right > 0) {
		R1 = true;
		R2 = false;
		ackerman->right_motor_direction = true;
    }
	else {
        R1 = false;
		R2 = true;
		ackerman->right_motor_direction = false;
    }
    if (V_left > 0) {
		L1 = true;
		L2 = false;
		ackerman->left_motor_direction = true;
    }
	else {
		L1 = false;
		L2 = true;
		ackerman->left_motor_direction = false;
    }
	ret_1 = ioctl(ackerman->gpio_a1_fd, GPIOC_WRITE, (unsigned long)R1);
	ret_2 = ioctl(ackerman->gpio_a2_fd, GPIOC_WRITE, (unsigned long)R2);
	ret_3 = ioctl(ackerman->gpio_b1_fd, GPIOC_WRITE, (unsigned long)L1);
	ret_4 = ioctl(ackerman->gpio_b2_fd, GPIOC_WRITE, (unsigned long)L2);
	if (ret_1 < 0 || ret_2 < 0  || ret_3 < 0 || ret_4 < 0 ) {
		fprintf(stderr,
			"ERROR:car_motor_direction: Failed to write gpio %d;%d;%d;%d\n",
				ret_1,ret_2,ret_3,ret_4);
		fprintf(stderr,
		"ERROR:file_fd %d;%d;%d;%d\n",
				ackerman->gpio_a1_fd,ackerman->gpio_a2_fd,ackerman->gpio_b1_fd,ackerman->gpio_b2_fd);
		return ERROR;
	}

	return OK;

}

void gpio_motion_deinit(void)
{
	close(g_motion_gpio.gpio_a1_fd);
	close(g_motion_gpio.gpio_a2_fd);
	close(g_motion_gpio.gpio_b1_fd);
	close(g_motion_gpio.gpio_b2_fd);
}

