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
#include <syslog.h>

#include <nuttx/timers/capture.h>


#include "main.h"

static struct rc_data_s g_rc_data;

static void rc_init(void){
	g_rc_data.angle_fd = open(RC_ANGLE_DEV, O_RDONLY);

	if (g_rc_data.angle_fd < 0){
		syslog(LOG_INFO,"rc_init: open %s failed: %d\n",
									RC_ANGLE_DEV, errno);
	}
	syslog(LOG_INFO,"rc_init: opened  %s fd = %d \n",RC_ANGLE_DEV,g_rc_data.angle_fd);

	g_rc_data.speed_fd = open(RC_SPEED_DEV, O_RDONLY);
	if (g_rc_data.speed_fd < 0){
		syslog(LOG_INFO,"rc_init: open %s failed: %d\n",
									RC_SPEED_DEV, errno);
	}
	syslog(LOG_INFO,"rc_init: opened %s fd = %d\n",RC_SPEED_DEV,g_rc_data.speed_fd);

	g_rc_data.rc_attached = false;
	g_rc_data.rc_data_ready = false;
	g_rc_data.sample_time = RC_SAMPLE_TIME;

}

static int get_rc_duty(int fd, uint8_t* duty){
	int ret;
	uint8_t count = 0;
	fflush(stdout);
	/* Get the dutycycle data using the ioctl */
	uint32_t start = clock_systime_ticks();
	do {
		ret = ioctl(fd, CAPIOC_DUTYCYCLE, (unsigned long)((uintptr_t)duty));
		if (ret < 0){
			syslog(LOG_INFO,"get_rc_duty: ioctl(CAPIOC_DUTYCYCLE) failed: %d, ret = %d \n", fd,ret);
			return ret;
		}
		count ++;
		usleep(5*1000);
		if (count > 30){
			*duty = RC_MIDDLE_VALUE;
			break;
		}
	}while(*duty < RC_MIN_VALUE ||*duty > RC_MAX_VALUE );

	//syslog(LOG_INFO, "rc read delay %dms\n", (clock_systime_ticks() - start)*10);

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
	struct timespec tp;

	while(!g_rc_data.rc_attached){
		if (OK == get_rc_duty(g_rc_data.speed_fd, &duty)) {
			if (duty >= RC_HAVE_VALUE){
				if (parameter_data.rc_enable == true)
				{
					g_rc_data.rc_attached = true;
				}
				break;
			}
		}
		sleep(2);
		//clock_gettime(CLOCK_MONOTONIC, &tp);
		//syslog(LOG_INFO,  "now tick is %d time is %ds %dns\n", clock_systime_ticks(), tp.tv_sec, tp.tv_nsec);
	}
	syslog(LOG_INFO,"Attached remote controller\n");
}

/* public function */
int get_rc_goal_speed(float* vx, float* vz, uint32_t* vt)
{
	if(g_rc_data.rc_attached && parameter_data.rc_enable)
	{
		*vx = g_rc_data.speed_x;
		*vz = g_rc_data.speed_z;
		*vt = g_rc_data.timestamp;

		return OK;
	}

	return -1;
}

int rc_task(int argc, char *argv[])
{
	int ret;
	uint8_t speed_duty = 0;
	uint8_t angle_duty = 0;
	int i = 0;

	/* init rc device */
	rc_init();

	/* attach rc */
	rc_check_attach();

	/* loop and receive rc data */
	while(g_rc_data.rc_attached){
		/* Get channels data*/
		usleep(g_rc_data.sample_time*1000);
		if (OK == get_rc_duty(g_rc_data.speed_fd, &speed_duty) &&
			OK == get_rc_duty(g_rc_data.angle_fd, &angle_duty))
		{
			rc_filter(&speed_duty, &angle_duty);
			g_rc_data.speed_duty = speed_duty;
			g_rc_data.angle_duty = angle_duty;
			g_rc_data.timestamp = clock_systime_ticks();
			g_rc_data.speed_x = 2 * (float)(g_rc_data.speed_duty -RC_MIDDLE_VALUE) /(float)(RC_MAX_VALUE - RC_MIN_VALUE) * MAX_SPEED;
			g_rc_data.speed_z = -2 * (float)(g_rc_data.angle_duty -RC_MIDDLE_VALUE) /(float)(RC_MAX_VALUE - RC_MIN_VALUE) * MAX_ANGULAR_VELOCITY;
#ifdef QTIAMR_DEBUG
			//printf("rc raw data  speed =%d , rotate = %d\n ",speed_duty,angle_duty);
			if (i++ % 100 == 0)
			{
				syslog(LOG_INFO, "rc get speed =%f , rotate = %f \n",g_rc_data.speed_x,g_rc_data.speed_z);
			}				
#endif
			g_rc_data.rc_data_ready == true;
		}
		else
			printf("ERROR: rc_task: get bad data\n");

	}

	return ret;
}

