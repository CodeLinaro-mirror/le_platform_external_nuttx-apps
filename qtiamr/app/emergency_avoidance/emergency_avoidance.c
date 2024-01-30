/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/
#include <stdio.h>
#include <unistd.h>
#include <syslog.h>
#include <string.h>

#include "main.h"
#include "qrc.h"
#include "config_msg.h"
#include "emergency_msg.h"
#include "qrc_msg_management.h"

#include "avoidance.h"
#include "avoid_management.h"
#include "emergency_avoidance.h"
#include "motion_management.h"

/****************************************************************************
 * Public data
 ****************************************************************************/
static struct avoid_client *emerg_client;
static struct qrc_pipe_s *emerg_pipe = NULL;


/****************************************************************************
 * Private Functions
 ****************************************************************************/
 /*callback of qrc_message*/
static void emerg_msg_cb (struct qrc_pipe_s *pipe,void * data, size_t len, bool response)
{
	struct emerg_msg_s *emerg_msg = NULL;

	if(!pipe || !data)
		return;

	if(len == sizeof(struct emerg_msg_s))
	{
		emerg_msg = (struct emerg_msg_s *)data;

		if(emerg_msg->value)
		{
			register_ultra_client(emerg_client);
		} else {
			motion_set_emergency(FALSE);
			unregister_ultra_client(emerg_client);
		}
	}

	syslog(LOG_DEBUG, "Received emergency message %u\n", emerg_msg->value);
}

/*callback of avoidance client*/
static void emerg_client_cb (uint8_t addr, uint16_t dist)
{
	struct emerg_msg_s msg = {0};

	msg.msg_type = T_SENSOR;
	msg.value = (int)addr;

	motion_set_emergency(TRUE);

	if(emerg_pipe)
	{
		qrc_write(emerg_pipe, (uint8_t*)&msg, sizeof(struct emerg_msg_s), FALSE);
	}

	syslog(LOG_DEBUG, "sensor %d triggerd emergency stop: %d\n", addr, dist);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
int emergency_main(int argc, char *argv[])
{
	int ret = 0;
	struct config_obstacle_avoidance_s obs_avoid_param;

	if (!avoidance_inited)
	{
		syslog(LOG_INFO, "emergency_main exit cause avoidance management fail\n");
		return ERROR;
	}

	memset(emerg_client, 0, sizeof(struct avoid_client));
	emerg_client->name = EMERG_PIPE;
	emerg_client->cb = emerg_client_cb;

	ret = get_configuration_parameters(SENSOR, &obs_avoid_param);
	if (ret > 0 )
	{
		emerg_client->thres_bottom = (uint16_t) (obs_avoid_param.bottom_dist * 1000);
		emerg_client->thres_side = (uint16_t) (obs_avoid_param.side_dist * 1000);
		emerg_client->thres_front = (uint16_t) (obs_avoid_param.front_dist * 1000);
	} else {
		syslog(LOG_INFO, "emergency avoidance parameters get failed\n");
		config_notify_completed(false);
		return ERROR;
	}

	register_ultra_client(emerg_client);

	emerg_pipe = qrc_get_pipe(EMERG_PIPE);
	if (!emerg_pipe)
	{
		syslog(LOG_INFO, "emergency avoidance get pipe failed\n");
		config_notify_completed(false);
		return ERROR;
	}

	if(!qrc_register_message_cb(emerg_pipe, emerg_msg_cb)) {
		syslog(LOG_INFO, "register qrc_msg callback failed\n");
		config_notify_completed(false);
		return ERROR;
	}

	syslog(LOG_DEBUG,"ultrasound management init done\n");
	config_notify_completed(true);
	return ret;
}



