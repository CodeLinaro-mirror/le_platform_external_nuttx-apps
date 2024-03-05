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
 * Pre-processor Definitions
 ****************************************************************************/
#define RETRY_NUM 3


/****************************************************************************
 * Public data
 ****************************************************************************/
static int g_avoid_fd;

uint8_t ultras_num = 5;
bool emerg_enter = FALSE;
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
		syslog(LOG_ERR, "ultrasound device open failed \n");
		return fd;
	}

	syslog(LOG_DEBUG, "ultrasound device open  %s done \n",ULTRASOUND_DEV);
	g_avoid_fd = fd;

	usleep(100000 *2);

	for (int i = 0; i < ultras_num; i++)
	{
		syslog(LOG_DEBUG, "check sensor %u \n", g_sensor_list[i].addr);
		ret = rs485_ultra_check(g_sensor_list[i].addr , g_avoid_fd);

		if ( ret < 0 )
		{
			syslog(LOG_ERR, "ultrasound sensor %u unreachable \n",g_sensor_list[i].addr);
			return ERROR;
		}
	}

	syslog(LOG_DEBUG, "avoid init successfully \n");
	return OK;
}

int avoid_management_thread(int argc, char *argv[])
{
	int ret = 0;
	int b = 0;
	int s = 0;
	int f = 0;
	int thres_r = 0;
	sigset_t set;
	struct avoid_client *client;

	sigemptyset(&set);
	sigaddset(&set, AVOID_WAKEUP);
	sigprocmask(SIG_UNBLOCK, &set, NULL);

	while(1)
	{
		if(list_is_empty(&g_avoid_client)) {
			syslog(LOG_INFO, "[avoidance mangement] client empty, pending...\n");
			sigwaitinfo(&set, NULL);
			syslog(LOG_INFO, "[avoidance mangement] receive client register...\n");
		}

		syslog(LOG_DEBUG,"[avoidance mangement]fetch ultra sensor dist, unit(mm)\n"); 
		for( int i = 0; i < ultras_num; i++)
		{
			ret = rs485_ultra_raw_dist(g_sensor_list[i].addr, g_avoid_fd);
			if (ret < 0)
			{
				syslog(LOG_ERR,"sensor %d read failed\n", g_sensor_list[i].addr);
				return ERROR;
			}

			list_for_every_entry(&g_avoid_client, client, struct avoid_client, node)
			{
				switch(g_sensor_list[i].type)
				{
					case BOTTOM:
						if ( ret > client->thres_bottom )
						{
							thres_r |= (0x1 << g_sensor_list[i].addr);
							syslog(LOG_INFO, "!!!!!! [%#X] BOTTOM sensor %d triggerd emergency stop: %d\n",
								thres_r, g_sensor_list[i].addr, ret);

							client->cb(g_sensor_list[i].addr, ret, TRUE);
						} else {
							if (thres_r & (0x1 << g_sensor_list[i].addr))
							{
								if (++b > RETRY_NUM)
								{
									thres_r ^= (0x1 << g_sensor_list[i].addr);
									b = 0;
									syslog(LOG_INFO, "!!!!!! [%#X] BOTTOM sensor %d exit emergency stop: %d\n",
										thres_r, g_sensor_list[i].addr, ret);
								}
							}
						}
						break;

					case SIDE:
						if ( ret < client->thres_side )
						{
							thres_r |= (0x1 << g_sensor_list[i].addr);
							syslog(LOG_INFO, "!!!!!![%#X] SIDE sensor %d triggerd emergency stop: %d\n",
								thres_r, g_sensor_list[i].addr, ret);

							client->cb(g_sensor_list[i].addr, ret, TRUE);
						} else {
							if (thres_r & (0x1 << g_sensor_list[i].addr))
							{
								if (++s > RETRY_NUM)
								{
									thres_r ^= (0x1 << g_sensor_list[i].addr);
									s = 0;
									syslog(LOG_INFO, "!!!!!! [%#X] SIDE sensor %d exit emergency stop: %d\n",
										thres_r, g_sensor_list[i].addr, ret);
								}
							}
						}
						break;

					case FRONT:
						if ( ret < client->thres_front)
						{
							thres_r |= (0x1 << g_sensor_list[i].addr);
							syslog(LOG_INFO, "!!!!!! [%#X] FRONT sensor %d triggerd emergency stop: %d\n",
								thres_r, g_sensor_list[i].addr, ret);

							client->cb(g_sensor_list[i].addr, ret, TRUE);
						} else {
							if (thres_r & (0x1 << g_sensor_list[i].addr))
							{
								if (++f > RETRY_NUM)
								{
									thres_r ^= (0x1 << g_sensor_list[i].addr);
									f = 0;
									syslog(LOG_INFO, "!!!!!! [%#X] FRONT sensor %d exit emergency stop: %d\n",
										thres_r, g_sensor_list[i].addr, ret);
								}
							}
						}
						break;

					default:
						syslog(LOG_ERR,"sensor type unrecognized\n");
				}

				if ( emerg_enter && !thres_r)
				{
					client->cb(g_sensor_list[i].addr, ret, FALSE);
				}
			}
		}

		usleep(100000);
	}
}


