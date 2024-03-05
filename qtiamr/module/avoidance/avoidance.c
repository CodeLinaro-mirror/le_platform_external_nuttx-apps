/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#include <stdio.h>
#include <syslog.h>
#include <nuttx/mutex.h>

#include "main.h"
#include "avoidance.h"
#include "config_msg.h"
#include "avoid_management.h"

/****************************************************************************
 * Public data
 ****************************************************************************/
int avoidance_inited = 0;
static int g_avoid_pid;
static rmutex_t g_client_lock = NXRMUTEX_INITIALIZER;

struct list_node g_avoid_client = LIST_INITIAL_VALUE(g_avoid_client);
/****************************************************************************
 * Public Function
 ****************************************************************************/

void register_ultra_client(struct avoid_client * client)
{
	union sigval svalue;

	svalue.sival_int = 1;

	if list_in_list(&client->node)
	{
		syslog(LOG_INFO,"%s already registered",client->name);
		return;
	}
	nxrmutex_lock(&g_client_lock);

	list_add_tail(&g_avoid_client, &client->node);
	sigqueue(g_avoid_pid, AVOID_WAKEUP, svalue);

	nxrmutex_unlock(&g_client_lock);

	syslog(LOG_DEBUG, "avoidance register client %s\n", client->name);
}

 void unregister_ultra_client(struct avoid_client * client)
 {
	 nxrmutex_lock(&g_client_lock);

	 if list_in_list(&client->node)
		list_delete(&client->node);

	 nxrmutex_unlock(&g_client_lock);
	 syslog(LOG_DEBUG, "avoidance unregister client %s\n", client->name);
 }

 int avoidance_main(int argc, char *argv[])
 {
	int ret = 0;
	struct config_sensor_s ultra_sensor_param;

	ret = get_configuration_parameters(SENSOR, &ultra_sensor_param);
	if (ret < 0)
	{
		syslog(LOG_ERR, "avoidance distance parameters get failed\n");
		config_notify_completed(false);
		return ERROR;
	};
		
	if(!ultra_sensor_param.ultra_enable)
	{
		syslog(LOG_ERR, "Warning: ultra disabled\n");
		config_notify_completed(true);
		return ret;
	}

	ultras_num =(uint8_t)ultra_sensor_param.ultra_quantity;

	if (ultras_num <= 0)
	{
		syslog(LOG_ERR, "Error: ultra number invalid\n");
		config_notify_completed(false);
		return ERROR;
	}
	
	if (OK != avoid_init())
	{
		config_notify_completed(false);
		return ERROR;
	}

	g_avoid_pid = task_create("avoid_management",
					DEFAULT_PRIORITY,
					DEFAULT_STACK_SIZE,
					avoid_management_thread,
					NULL);

	if ( g_avoid_pid < 0 )
	{
		syslog(LOG_ERR, "create avoid management thread failed\n");
		config_notify_completed(false);
		return ERROR;
	} else {
		syslog(LOG_INFO, "create avoid management thread with pid %d\n", g_avoid_pid);
	}

	syslog(LOG_DEBUG,"ultrasound management init done\n");
	
	avoidance_inited = 1;
	config_notify_completed(true);

	return ret;
 }

