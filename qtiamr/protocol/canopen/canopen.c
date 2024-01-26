/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#include <fcntl.h>
#include <stdio.h>
#include <inttypes.h>
#include <nuttx/can/can.h>
#include <syslog.h>

#include "canopen.h"

int canopen_send(int fd, char* buffer, size_t msgsize)
{
    struct can_msg_s txmsg;
	size_t nbytes;
	int res = ERROR;

	if(msgsize > CAN_MSDGLC)
	{
		syslog(LOG_ERR, "canopen message length error !\n");
		return res;
	}

	if(fd <= 0)
	{
		syslog(LOG_ERR, "canopen file description error !\n");
		return res;
	}

	if(buffer == NULL)
	{
		syslog(LOG_ERR, "canopen address error !\n");
		return res;
	}

    txmsg.cm_hdr.ch_id     = COB_ID_SSDO;
    txmsg.cm_hdr.ch_rtr    = false;
    txmsg.cm_hdr.ch_dlc    = msgsize;
	txmsg.cm_hdr.ch_unused = 0;

	memcpy(txmsg.cm_data, buffer, msgsize);


	nbytes = CAN_MSGLEN(msgsize);

	nbytes = write(fd, &txmsg, nbytes);
	if (nbytes < msgsize)
	{
		syslog(LOG_ERR, "ERROR: canopen write(%ld) returned %ld\n", (long)msgsize, (long)nbytes);
		return res;
	}
	syslog(LOG_ERR, "debug canopen_send write =%d end\n",nbytes);

    return OK;
}

int canopen_receive(int fd, char* buffer, size_t len)
{
    struct can_msg_s rxmsg;

	size_t msgsize;
	ssize_t rcv_bytes = 0;
	int res = ERROR;

	if(fd <= 0)
	{
		syslog(LOG_ERR, "canopen file description error !\n");
		return res;
	}

	if(buffer == NULL)
	{
		syslog(LOG_ERR, "canopen address error !\n");
		return res;
	}

	msgsize = sizeof(struct can_msg_s);
	rcv_bytes = read(fd, &rxmsg, msgsize);
	if (rcv_bytes < CAN_MSGLEN(0) || rcv_bytes > msgsize)
	{
		syslog(LOG_ERR, "ERROR: canopen read(%ld) returned %ld\n", (long)msgsize, (long)rcv_bytes);
		return res;
	}

	rcv_bytes = rxmsg.cm_hdr.ch_dlc;
	/*
	int i = 0;
	for (i = 0;i<8;i++)
	{
		syslog(LOG_INFO, "[%d]0x%x",i,rxmsg.cm_data[i]);
	}
	syslog(LOG_ERR, " canopen receive bytes=%d\n",rcv_bytes);
	*/
	if(rcv_bytes >= len)
	{
		memcpy(buffer, rxmsg.cm_data, len);
	}

	fflush(stdout);
	
	return rcv_bytes;
}
