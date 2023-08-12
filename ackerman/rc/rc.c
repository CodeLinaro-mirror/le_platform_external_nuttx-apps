/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <sched.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include <nuttx/timers/capture.h>


#include "ros_com.h"
#include "car_main.h"
#include "motion_task.h"
#include "rc.h"

static struct rc_data_s g_rc_data;

static void rc_init(void){
	g_rc_data.angle_fd = open(RC_ANGLE_DEV, O_RDONLY);

	if (g_rc_data.angle_fd < 0){
		printf("rc_init: open %s failed: %d\n",
									RC_ANGLE_DEV, errno);
	}
	printf("rc_init: opened  %s fd = %d \n",RC_ANGLE_DEV,g_rc_data.angle_fd);

	g_rc_data.speed_fd = open(RC_SPEED_DEV, O_RDONLY);
	if (g_rc_data.speed_fd < 0){
		printf("rc_init: open %s failed: %d\n",
									RC_SPEED_DEV, errno);
	}
	printf("rc_init: opened %s fd = %d\n",RC_SPEED_DEV,g_rc_data.speed_fd);

	g_rc_data.rc_attached = false;
	g_rc_data.rc_data_ready = false;
	g_rc_data.sample_time = RC_SAMPLE_TIME;

}

static int get_rc_duty(int fd, uint8_t* duty){
	int ret;
	uint8_t count = 0;
	fflush(stdout);
      /* Get the dutycycle data using the ioctl */
	do {
		ret = ioctl(fd, CAPIOC_DUTYCYCLE, (unsigned long)((uintptr_t)duty));
   		if (ret < 0){
        	printf("get_rc_duty: ioctl(CAPIOC_DUTYCYCLE) failed: %d, ret = %d \n", fd,ret);
        	return ret;
		}
		count ++;
		usleep(5*1000);
		if (count > 10){
			*duty = RC_MIDDLE_VALUE;
			break;
		}

	}while(*duty < RC_MIN_VALUE ||*duty > RC_MAX_VALUE );
	
	return OK;
}

static void rc_filter(uint8_t *speed_duty, uint8_t *angle_duty)
{
	/* filter */
	*speed_duty = AMP_LIMIT(*speed_duty, RC_MIN_VALUE, RC_MAX_VALUE);
	*angle_duty = AMP_LIMIT(*angle_duty, RC_MIN_VALUE, RC_MAX_VALUE);
}

static void rc_check_attach(void)
{
	uint8_t duty;
	int ret;

	while(!g_rc_data.rc_attached){
		if (OK == get_rc_duty(g_rc_data.speed_fd, &duty)) {
			if (duty >= RC_HAVE_VALUE){
				g_rc_data.rc_attached = true;
				break;
			}
		}
		sleep(2);
	}
	printf("Attached remote controller\n");
}

/* public function */
int get_rc_goal_speed(float* vx, float* vz){

	if(g_rc_data.rc_attached){
		*vx = g_rc_data.speed_x;
		*vz = g_rc_data.speed_z;
		return OK;
	}

	return -1;
}

int rc_task(int argc, char *argv[]){
	int ret;
	uint8_t speed_duty = 0;
	uint8_t angle_duty = 0;

	/* init rc device */
	rc_init();

	/* attach rc */
	rc_check_attach();

	/* loop and receive rc data */
	while(g_rc_data.rc_attached){
		/* Get channels data*/

		if (OK == get_rc_duty(g_rc_data.speed_fd, &speed_duty) &&
			OK == get_rc_duty(g_rc_data.angle_fd, &angle_duty))
		{
			rc_filter(&speed_duty, &angle_duty);
			g_rc_data.speed_duty = speed_duty;
			g_rc_data.angle_duty = angle_duty;
			g_rc_data.speed_x = 2 * (float)(g_rc_data.speed_duty -RC_MIDDLE_VALUE) /(float)(RC_MAX_VALUE - RC_MIN_VALUE);
			g_rc_data.speed_z = 2 * (float)(g_rc_data.angle_duty -RC_MIDDLE_VALUE) /(float)(RC_MAX_VALUE - RC_MIN_VALUE);
#ifdef ACKERMAN_DEBUG
			printf("rc raw data  speed =%d , rotate = %d\n ",speed_duty,angle_duty);
			printf("rc get speed =%f , rotate = %f\n\n ",g_rc_data.speed_x,g_rc_data.speed_z);
#endif
			g_rc_data.rc_data_ready == true;
		}
		else
			printf("ERROR: rc_task: get bad data\n");

		usleep(g_rc_data.sample_time*1000);
	}

	return ret;
}
