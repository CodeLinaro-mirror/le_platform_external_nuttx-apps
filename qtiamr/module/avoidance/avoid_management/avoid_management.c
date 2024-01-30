/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <syslog.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/ioctl.h>

#include "dyp_02.h"
#include "avoidance.h"
#include "avoid_management.h"

/****************************************************************************
 * Public data
 ****************************************************************************/
static int g_avoid_fd;

uint8_t ultras_num = 5;
const struct avoid_sensor g_sensor_list[SENSOR_MAX] = {
	{ BOTTOM, 0X1},
	{ BOTTOM, 0X2},
	{ SIDE,   0X3},
	{ SIDE,   0X4},
	{ FRONT,  0X5},
	{ FRONT,  0X6},
	{ FRONT,  0X7},
};

/****************************************************************************
 * Public Function
 ****************************************************************************/
int avoid_init(void)
{
	int ret, fd;

	fd = open(ULTRASOUND_DEV, O_RDWR|O_NONBLOCK);
	if (fd < 0 )
	{
		syslog(LOG_INFO, "ultrasound device open failed \n");
		return fd;
	}

	syslog(LOG_INFO, "ultrasound device open  %s done \n",ULTRASOUND_DEV);
	g_avoid_fd = fd;

	usleep(1000000 *2);

	for (int i = 0; i < ultras_num; i++)
	{
		syslog(LOG_INFO, "check sensor %u \n", g_sensor_list[i].addr);
		ret = rs485_ultra_check(g_sensor_list[i].addr , g_avoid_fd);

		if ( ret < 0 )
		{
			syslog(LOG_INFO, "ultrasound sensor %u unreachable \n",g_sensor_list[i].addr);
			return ERROR;
		}
	}

	syslog(LOG_DEBUG, "avoid init successfully \n");
	return OK;
}

int avoid_management_thread(int argc, char *argv[])
{
	int ret;
	sigset_t set;
	struct avoid_client *client;

	sigemptyset(&set);
	sigaddset(&set, AVOID_WAKEUP);
	sigprocmask(SIG_UNBLOCK, &set, NULL);

	while(1)
	{
		if(list_is_empty(&g_avoid_client)) {
			syslog(LOG_INFO, "avoidance client empty \n");
			sigwaitinfo(&set, NULL);
		}

		for( int i = 0; i < ultras_num; i++)
		{
			ret = rs485_ultra_raw_dist(g_sensor_list[i].addr, g_avoid_fd);
			if (ret > 0)
			{
				list_for_every_entry(&g_avoid_client, client, struct avoid_client, node)
				{
					switch(g_sensor_list[i].type)
					{
						case BOTTOM:
							if ( ret > client->thres_bottom )
							{
								client->cb(g_sensor_list[i].addr, ret);
							}
							break;

						case SIDE:
							if ( ret < client->thres_side )
							{
								client->cb(g_sensor_list[i].addr, ret);
							}
								break;

						case FRONT:
							if ( ret < client->thres_front)
							{
								client->cb(g_sensor_list[i].addr, ret);
							}
							break;
						}
					}
			}else {
				syslog(LOG_DEBUG,"sensor %d read failed\n", g_sensor_list[i].addr);
				return ERROR;
			}
		}
	}
}


