/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/


#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <limits.h>
#include <inttypes.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/sensors/qencoder.h>

#include "car_encoder.h"

#ifdef CAR_MOTOR_QE
static struct car_qe_s g_motor_qe[CAR_MOTOR_QE_NUM];
#endif



/** Read count and clear zero.
 * @position : encoder count,range 0~2^32 
 * @len : position size , 1 or 2 
 *
 */
int car_get_qe_count(int32_t *position,uint8_t index){
	int ret;
	int fd;
	ret = ERROR;

	if (position && index < 0 && index >= CAR_MOTOR_QE_NUM){
		ret = -EINVAL;
		return ret;
	}

	/* Get the positions data using the ioctl */
	fd = g_motor_qe[index].qe_fd;
	ret = ioctl(fd, QEIOC_POSITION,
				(unsigned long)((uintptr_t)position));
	if (ret < 0) {
		printf("car_get_qe_count: ioctl(QEIOC_POSITION) failed: %d\n", errno);
		goto errout_with_dev;
	}
	g_motor_qe[index].position = *position;
	/* Reset the count */
	if (g_motor_qe[index].reset) {
		ret = ioctl(fd, QEIOC_RESET, 0);
		if (ret < 0){
			printf("car_get_qe_count: ioctl(QEIOC_RESET) failed: %d\n", errno);
			goto errout_with_dev;
		}
	}
 
	return OK;

errout_with_dev:
	close(fd);
	fflush(stdout);
	return ret;
	
}

/** init QE, open qe dev node & reset **/
int car_qe_init(struct car_qe_s **ackerman, uint8_t *encoder_num){
	int ret,num;
	int fd;
	int exitval = EXIT_SUCCESS;

	ret = ERROR;
	
#ifdef CAR_MOTOR_QE
	for (num =0; num < CAR_MOTOR_QE_NUM; num++){
		g_motor_qe[num].initialized = false;
		g_motor_qe[num].reset = true;
		g_motor_qe[num].position = 0;
		g_motor_qe[num].qe_fd = 0;
	}

	g_motor_qe[CAR_MOTOR_QE_A].devpath = strdup(QE_MOTOR_A_DEV);
	g_motor_qe[CAR_MOTOR_QE_B].devpath = strdup(QE_MOTOR_B_DEV);

	fd = open(g_motor_qe[CAR_MOTOR_QE_A].devpath, O_RDONLY);
	if (fd < 0)
    {
      printf("car_qe_init: open %s failed: %d\n", g_motor_qe[CAR_MOTOR_QE_A].devpath, errno);
      ret = EXIT_FAILURE;
      goto errout_with_dev;
    }
	g_motor_qe[CAR_MOTOR_QE_A].initialized = true;
	g_motor_qe[CAR_MOTOR_QE_A].qe_fd = fd;

	fd = open(g_motor_qe[CAR_MOTOR_QE_B].devpath, O_RDONLY);
	if (fd < 0)
    {
      printf("car_qe_init: open %s failed: %d\n", g_motor_qe[CAR_MOTOR_QE_B].devpath, errno);
      ret = EXIT_FAILURE;
      goto errout_with_dev;
    }
	g_motor_qe[CAR_MOTOR_QE_B].initialized = true;
	g_motor_qe[CAR_MOTOR_QE_B].qe_fd = fd;

	ret = OK;
#endif

	*ackerman = g_motor_qe;
	*encoder_num = CAR_MOTOR_QE_NUM;

	return ret;

errout_with_dev:
	close(fd);
	fflush(stdout);
	return ret;
}

void car_qe_deinit(void)
{
	close(g_motor_qe[CAR_MOTOR_QE_A].qe_fd);
	close(g_motor_qe[CAR_MOTOR_QE_B].qe_fd);
}