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

static struct avoid_sensor g_sensor_list[SENSOR_MAX] = {
	{ BOTTOM, 0X1, 0},
	{ BOTTOM, 0X2, 0},
	{ SIDE,   0X3, 0},
	{ SIDE,   0X4, 0},
	{ FRONT,  0X5, 0},
	{ FRONT,  0X6, 0},
	{ FRONT,  0X7, 0},
};

/****************************************************************************
 * Pravite Function
 ****************************************************************************/
void check_client_trigger(struct list_node* avoid_client_list, struct avoid_sensor *sensor, int dist)
{
	bool thres_meet = FALSE;
	struct avoid_client *client;

	list_for_every_entry(avoid_client_list, client, struct avoid_client, node)
	{
		switch(sensor->type)
		{
		case BOTTOM:
			thres_meet = dist > client->thres_bottom;
			break;
		case SIDE:
			thres_meet = dist < client->thres_side;
			break;
		case FRONT:
			thres_meet = dist < client->thres_front;
			break;
		default:
			syslog(LOG_ERR,"sensor type unrecognized\n");
			return;
		}

		if(thres_meet)
		{
			client->trigger |= (0x1 << sensor->addr);
			syslog(LOG_INFO, "!!!!!! [%#X] Sensor %d triggerd emergency stop: %d\n",
					client->trigger, sensor->addr, dist);

			client->cb(sensor->addr, dist, TRUE);
		} else {
			if(((client->trigger >> sensor->addr) & 0x1) && (++(sensor->count) >= RETRY_NUM))
			{
				client->trigger ^= (0x1 << sensor->addr);
				sensor->count = 0;
				syslog(LOG_INFO, "!!!!!! [%#X] Sensor %d exit emergency stop: %d\n",
							client->trigger, sensor->addr, dist);
			}
		}

		if (!(client->trigger ^ 0x1))
		{
			client->cb(sensor->addr, dist, FALSE);
		}
	}
}

/****************************************************************************
 * Public Function
 ****************************************************************************/
struct avoid_sensor* get_ultra_sensor_list(void)
{
	return g_sensor_list;
}

int avoid_init(void)
{
	int ret, fd;
	int ultra_sensor_num = get_ultra_num();

	fd = open(ULTRASOUND_DEV, O_RDWR|O_NONBLOCK);
	if (fd < 0 )
	{
		syslog(LOG_ERR, "ultrasound device open failed \n");
		return fd;
	}

	syslog(LOG_DEBUG, "ultrasound device open  %s done \n",ULTRASOUND_DEV);
	g_avoid_fd = fd;

	usleep(100000 *2);

	for (int i = 0; i < ultra_sensor_num; i++)
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
	sigset_t set;

	struct avoid_sensor *sensor;

	int ultra_sensor_num = get_ultra_num();
	struct list_node* avoid_client_list = get_avoid_client_list();

	sigemptyset(&set);
	sigaddset(&set, AVOID_WAKEUP);
	sigprocmask(SIG_UNBLOCK, &set, NULL);
	
	while(1)
	{
		if(list_is_empty(avoid_client_list))
		{
			syslog(LOG_INFO, "[avoidance mangement] client empty, pending...\n");
			sigwaitinfo(&set, NULL);
			syslog(LOG_INFO, "[avoidance mangement] receive client register...\n");
		}

		syslog(LOG_DEBUG,"[avoidance mangement]fetch ultra sensor dist, unit(mm)\n"); 
		for( int i = 0; i < ultra_sensor_num; i++)
		{
			sensor = &g_sensor_list[i];
			ret = rs485_ultra_raw_dist(sensor->addr, g_avoid_fd);
			if (ret < 0)
			{
				syslog(LOG_ERR,"sensor %d read failed\n", g_sensor_list[i].addr);
				return ERROR;
			}
			check_client_trigger(avoid_client_list, sensor, ret);
		}

		usleep(100000);
	}
}


