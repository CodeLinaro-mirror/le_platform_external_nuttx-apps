/***************************************************************************
* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
****************************************************************************/

#include "auto_charging.h"

#define CAN_OFLAGS O_RDONLY
#define CONFIG_EXAMPLES_CAN_DEVPATH "/dev/can1"
#define PRI_CAN_ID PRIu16

int fd;
static audo_charging_data_s g_charging_data;

int auto_charging_init(){
	int ret;

	struct canioc_bittiming_s bt;
	long minid    = 1;
	long maxid    = 0x07ff;

	long nmsgs    = 1;
	int errval    = 0;

	syslog(LOG_INFO,"nmsgs: %ld\n", nmsgs);
	syslog(LOG_INFO,"min ID: %ld max ID: %ld\n", minid, maxid);

	fd = open(CONFIG_EXAMPLES_CAN_DEVPATH, CAN_OFLAGS);
	if (fd < 0){
		syslog(LOG_INFO,"ERROR: open %s failed: %d\n", CONFIG_EXAMPLES_CAN_DEVPATH, errno);
		errval = 2;
	}

	ret = ioctl(fd, CANIOC_GET_BITTIMING, (unsigned long)((uintptr_t)&bt));
	if (ret < 0)
	{
		syslog(LOG_INFO,"Bit timing not available: %d\n", errno);
	}
	else
	{
		syslog(LOG_INFO,"Bit timing:\n");
		syslog(LOG_INFO,"   Baud: %lu\n", (unsigned long)bt.bt_baud);
		syslog(LOG_INFO,"  TSEG1: %u\n", bt.bt_tseg1);
		syslog(LOG_INFO,"  TSEG2: %u\n", bt.bt_tseg2);
		syslog(LOG_INFO,"    SJW: %u\n", bt.bt_sjw);
	}
	return 0;
}

int get_charging_goal_speed(float* vx, float* vz, uint32_t* vt)
{
	*vx = 0;
	*vz = 0;
	*vt = g_charging_data.timestamp;
	if(g_charging_data.If_charging != 1)
	{
		*vx = - (g_charging_data.vx / 30);
		*vz = - (g_charging_data.vz / 8 );
		*vt = g_charging_data.timestamp;
	}
	return OK;
}

int g_i;
static void charging_data_sync(infrared_dock_cmd_s *buff, int len)
{
	float vx;
	float vz;
	float current;

	bool If_infrared = false;
	bool If_charging = false;
	g_charging_data.vx = (((buff->vx_h << 8) + buff->vx_l) - (( buff->vx_h >> 7) * (0xFFFF + 1)) )/50;
	g_charging_data.vz = (((buff->vz_h << 8) + buff->vz_l) - (( buff->vz_h >> 7) * (0xFFFF + 1)) )/100;
	g_charging_data.If_infrared = buff->flags & (0x01 << 1);
	g_charging_data.If_charging = buff->flags & (0x01);
	g_charging_data.current = 0.033 * ( buff->current - ((buff->current >> 7) * (0xFF + 1)));
	g_charging_data.timestamp = clock_systime_ticks();
	if((g_i++ % 50) == 0)
	{
		g_i = 1;
		syslog(LOG_DEBUG, "receive data: \n");
		syslog(LOG_INFO,"vx_h = 0x%02x ", buff->vx_h);
		syslog(LOG_INFO,"vx_l = 0x%02x \n", buff->vx_l);
		syslog(LOG_INFO,"vz_h = 0x%02x ", buff->vz_h);
		syslog(LOG_INFO,"vz_l = 0x%02x \n", buff->vz_l);
		syslog(LOG_INFO,"flags = 0x%02x \n", buff->flags);
		syslog(LOG_INFO,"current = 0x%02x \n", buff->current);

		syslog(LOG_INFO,"vx = %.2fmm/s, vz = %.2frad/s\n", g_charging_data.vx, g_charging_data.vz);
		syslog(LOG_INFO,"If_infrared = %d, If_charging = %d\n", g_charging_data.If_infrared, g_charging_data.If_charging);
		syslog(LOG_INFO,"current = %.2fA\n", g_charging_data.current);
	}
}

static uint8_t auto_chanrging_get_data(void){
	size_t msgsize;
	ssize_t nbytes;
	infrared_dock_cmd_s charging_buf;
	struct can_msg_s rxmsg;
	long nmsgs = 1;
	long msgno;
	int msgdlc;
	int errval = 0;
	uint8_t rev_buff[CAN_MAXDATALEN];
	for (msgno = 0; !nmsgs || msgno < nmsgs; msgno++){
		/* Flush any output before the loop entered or from the previous pass
		 * * through the loop.
		 * */
		fflush(stdout);
		msgsize = sizeof(struct can_msg_s);
		nbytes = read(fd, &rxmsg, msgsize);
		if (nbytes < CAN_MSGLEN(0) || nbytes > msgsize)
		{
			syslog(LOG_INFO,"ERROR: read(%ld) returned %ld\n", (long)msgsize, (long)nbytes);
			errval = 4;
		}

		//printf("  ID: %4u DLC: %u\n", rxmsg.cm_hdr.ch_id, rxmsg.cm_hdr.ch_dlc);
		msgdlc = rxmsg.cm_hdr.ch_dlc;

		//printf("Data received:\n");
		if(rxmsg.cm_hdr.ch_id == 0x182)
		{
			memcpy(&charging_buf, &rxmsg.cm_data, CAN_MAXDATALEN);
		}
	}
	charging_data_sync(&charging_buf, msgdlc);
}

static auto_charging_sync_power_voltage(void)
{
	int ret=0;
	float power_voltage;
	ret = get_power_voltage(&power_voltage);
}

int auto_charging_task(int argc, char *argv[]){
	g_i =0;
	auto_charging_init();
	amr_adc_init();
	while(1){
		auto_charging_sync_power_voltage();
		auto_chanrging_get_data();
		usleep(100);
	}
	return 0;
}
