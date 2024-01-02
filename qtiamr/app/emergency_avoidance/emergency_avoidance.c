/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#include <syslog.h>

#include "emergency_avoidance.h"
#include "emergency_msg.h"
#include "qrc_msg_management.h"

void emerg_msg_cb (struct qrc_pipe_s *pipe,void * data, size_t len, bool response)
{
	syslog(LOG_INFO, "Received emergency message\n")
}

int emergency_avoidance(int argc, char *argv[])
{
	ret = 0;
	struct qrc_pipe_s emerg_pipe;

	emerg_pipe = qrc_get_pipe(EMERG_PIPE)

	if(qrc_register_message_cb(emerg_pipe, emerg_msg_cb)) {
		syslog(LOG_ERR, "register qrc_msg callback failed\n");
		return -1;
	}

	syslog(LOG_INFO,"ultrasound management init done\n");
	return ret;
}



