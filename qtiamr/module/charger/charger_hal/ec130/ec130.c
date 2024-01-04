/***************************************************************************
* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
****************************************************************************/

#include "charger_core.h"
#include "ec130.h"
#include "voltage_adc.h"



ec130_dev_t g_ec130_dev;



int g_i;
void ec130_raw_data_parse(ec130_raw_data_s *buff){
	ec130_data_t *ec130_data_p = &g_ec130_dev.ec130_data;
	ec130_data_p->vx = (((buff->vx_h << 8) + buff->vx_l) - (( buff->vx_h >> 7) * (0xFFFF + 1)) );//0.001m/s
	ec130_data_p->vz = (((buff->vz_h << 8) + buff->vz_l) - (( buff->vz_h >> 7) * (0xFFFF + 1)) );//0.001rad/s
	ec130_data_p->If_infrared = buff->flags & (0x01 << 1);
	ec130_data_p->If_charging = buff->flags & (0x01);
	ec130_data_p->current = ( buff->current - ((buff->current >> 7) * (0xFF + 1)))* CHARGING_CURR_UNIT;
	ec130_data_p->timestamp = clock_systime_ticks();

	if(ec130_data_p->If_infrared == 0) ec130_data_p->vx = ec130_data_p->vz = 0;
	if(ec130_data_p->If_charging) ec130_data_p->vx = ec130_data_p->vz = 0;
	if((g_i++ % 100) == 0)
	{
		g_i = 1;
		syslog(LOG_DEBUG,"receive data: \n");
		syslog(LOG_DEBUG,"vx_h = 0x%02x ", buff->vx_h);
		syslog(LOG_DEBUG,"vx_l = 0x%02x \n", buff->vx_l);
		syslog(LOG_DEBUG,"vz_h = 0x%02x ", buff->vz_h);
		syslog(LOG_DEBUG,"vz_l = 0x%02x \n", buff->vz_l);
		syslog(LOG_DEBUG,"flags = 0x%02x \n", buff->flags);
		syslog(LOG_DEBUG,"current = 0x%02x \n", buff->current);
		syslog(LOG_DEBUG,"parse data: \n");
		syslog(LOG_DEBUG,"vx = %.2fmm/s, vz = %.2frad/s\n", ec130_data_p->vx, ec130_data_p->vz);
		syslog(LOG_DEBUG,"If_infrared = %d, If_charging = %d\n", ec130_data_p->If_infrared, ec130_data_p->If_charging);
		syslog(LOG_DEBUG,"current = %.2fA\n", ec130_data_p->current);
	}
}

static bool ec130_get_can_data(){
	ec130_dev_t *ec130_dev_p = &g_ec130_dev;
	int ret = OK;
	int fd;
	ssize_t nbytes;
	struct can_msg_s rxmsg;
	long nmsgs = 1;
	long msgno;
	if(!ec130_dev_p->initialized){
		return ERROR;
	}
	fd = ec130_dev_p->fd;
	for (msgno = 0; !nmsgs || msgno < nmsgs; msgno++){
		/* Flush any output before the loop entered or from the previous pass
		 * * through the loop.
		 * */
		fflush(stdout);
		nbytes = read(fd, &rxmsg, sizeof(struct can_msg_s));
		if (nbytes < CAN_MSGLEN(0) || nbytes > sizeof(struct can_msg_s))
		{
			syslog(LOG_ERR,"ERROR: read(%ld) returned %ld\n", (long)sizeof(struct can_msg_s), (long)nbytes);
			ret = ERROR;
		}
		//printf("  ID: %4u DLC: %u\n", rxmsg.cm_hdr.ch_id, rxmsg.cm_hdr.ch_dlc);
		if(rxmsg.cm_hdr.ch_id == AUTO_CHARGER_MSG_ID){
			ec130_raw_data_parse((ec130_raw_data_s)&rxmsg.cm_data);
		}
		else
			ret = ERROR;
	}
	return ret;
} 


bool ec130_get_wheel_speed(float *vx, float *vz){
	
	ec130_data_t *ec130_data_p = &g_ec130_dev.ec130_data;
	if(ec130_get_can_data() < 0)
		return ERROR;

	*vx = -(ec130_data_p->vx / WHELL_SPEED_VX_DIV);
	*vz = -(ec130_data_p->vz / WHELL_SPEED_VZ_DIV);
	//speed->timestamp = ec130_data_p->timestamp;

	return OK;
}

bool ec130_get_current(float *curr){
	ec130_data_t *ec130_data_p = &g_ec130_dev.ec130_data;
	if(ec130_get_can_data() < 0)
		return ERROR;
	*curr = ec130_data_p->current;
	return OK;
}

bool ec130_infrared_sig_stat(bool *stat){
	ec130_data_t *ec130_data_p = &g_ec130_dev.ec130_data;
	if(ec130_get_can_data() < 0)
		return ERROR;
	*stat = ec130_data_p->If_infrared;
	return OK;
}

bool ec130_if_charging_stat(bool *stat){
	ec130_data_t *ec130_data_p = &g_ec130_dev.ec130_data;
	if(ec130_get_can_data() < 0)
		return ERROR;
	*stat = ec130_data_p->If_charging;
	return OK;
}


static const chr_drv_ops_t ec130_drv_ops = {
	.get_voltage		= adc_get_voltage,
	.get_current		= ec130_get_current,
	.get_wheel_speed	= ec130_get_wheel_speed,
	.chr_if_charging	= ec130_if_charging_stat,
	.chr_pile_sig_stat  = ec130_infrared_sig_stat,
};


bool ec130_can_init(){
	int ret = OK;
	int fd;
	struct canioc_bittiming_s bt;
	long minid    = 1;
	long maxid    = 0x07ff;
	long nmsgs    = 1;

	syslog(LOG_DEBUG,"nmsgs: %ld\n", nmsgs);
	syslog(LOG_DEBUG,"min ID: %ld max ID: %ld\n", minid, maxid);

	fd = open(CONFIG_EXAMPLES_CAN_DEVPATH, O_RDONLY);
	if (fd < 0){
		syslog(LOG_ERR,"ERROR: open %s failed: %d\n", CONFIG_EXAMPLES_CAN_DEVPATH, errno);
		ret = ERROR;
	}
	g_ec130_dev.fd = fd;
	ret = ioctl(fd, CANIOC_GET_BITTIMING, (unsigned long)((uintptr_t)&bt));
	if (ret < 0){
		syslog(LOG_ERR,"Bit timing not available: %d\n", errno);
		ret = ERROR;
	}
	else{
		syslog(LOG_DEBUG,"Bit timing:\n");
		syslog(LOG_DEBUG,"   Baud: %lu\n", (unsigned long)bt.bt_baud);
		syslog(LOG_DEBUG,"  TSEG1: %u\n", bt.bt_tseg1);
		syslog(LOG_DEBUG,"  TSEG2: %u\n", bt.bt_tseg2);
		syslog(LOG_DEBUG,"    SJW: %u\n", bt.bt_sjw);
	}
	return ret;
}


bool ec130_driver_init(){
	int ret;
	if (g_ec130_dev.initialized){
    syslog(LOG_ERR, "CHARGER: ec130_driver_init: has been Initialized!\n");
		return ERROR;
	}
	memset(&g_ec130_dev, 0, sizeof(ec130_dev_t));
	ret = ec130_can_init();
	if(ret < 0)
  { 
    syslog(LOG_ERR, "CHARGER: ec130_driver_init: ec130 can init failed!\n");
    return ERROR;
  }
	chr_core_drv_ops_register("ec130",&ec130_drv_ops);

	g_ec130_dev.initialized = true;
  syslog(LOG_DEBUG,"CHARGER: ec130_driver_init: ec130 Initialize successfully!\n");
  
	return OK;
}

bool ec130_and_adc_driver_init(){

	if(charger_voltage_adc_init() != TRUE)
		return ERROR;
	if(ec130_driver_init() != TRUE)
		return ERROR;
	return OK;
}



