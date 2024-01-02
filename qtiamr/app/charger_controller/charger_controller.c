/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#include "charger_interface.h"
#include "qrc_msg_management.h"



void charger_qrc_msg_send(struct charger_ctl_cmd_s ctl_cmd_data,uint8_t sync){

	char pipe_name[] = CHARGER_MSG_PIPE;
	qrc_write_status_e result;
	struct qrc_pipe_s *pipe;
	
	
	/* get pipe */
	pipe =	qrc_get_pipe(pipe_name);
	if (pipe == NULL)
	  {
	  syslog(LOG_ERR,"charger_qrc_msg_send: qrc get pipe failed!\n");
	  return ERROR;
	  }
	if(sync)
	{
		result = qrc_response(pipe , (void *) ctl_cmd_data, sizeof(struct charger_ctl_cmd_s));
		if (result != SUCCESS)
    	{
      	syslog(LOG_ERR,"charger_qrc_msg_send: qrc sync write pipe failed(%d)!\n",result);
	  	return result;
    	}
	}
	else{
		result = qrc_write(pipe , (void *) ctl_cmd_data, sizeof(struct charger_ctl_cmd_s), true);
		if (result != SUCCESS)
    	{
      	syslog(LOG_ERR,"charger_qrc_msg_send: qrc write pipe failed(%d)!\n",result);
	  	return result;
    	}
	}
	return result;
	
}

int charger_qrc_async_msg_send_cb(void * data, size_t len, bool ack){
	char pipe_name[] = CHARGER_MSG_PIPE;
	qrc_write_status_e result;
	struct qrc_pipe_s *pipe;
	
	/* get pipe */
	pipe =	qrc_get_pipe(pipe_name);
	if (pipe == NULL)
	  {
	  syslog(LOG_ERR,"charger_qrc_msg_send_callback: qrc get pipe failed!\n");
	  return ERROR;
	  }
	result = qrc_write(pipe , data, len, true);
  	if (result != SUCCESS)
    	{
      	syslog(LOG_ERR,"charger_qrc_msg_send_callback: qrc write pipe failed(%d)!\n",result);
	  	return result;
    	}

  return result;

}

int charger_motion_speed_send_cb(float vx, float vz){
  
   return motion_control_speed(CHARGER_CONTROLLER, vx,vz);
}

int charger_start_charging_async_response(){
	int ret;
	set_sm_searching_state_intf();
	ret = charger_sm_signal_send_intf();
	if(ret < 0){
		syslog(LOG_ERR,"charger_qrc_msg_send_callback: qrc get pipe failed!\n");
	}
	return ret;
}

int charger_stop_charging_async_response(){
  stop_sm_charging_intf();
	
}


void battery_voltage_sync_response(bool sync){
	float voltage = 0;
	struct charger_ctl_cmd_s voltage_response;
	voltage = get_battery_voltage_intf();
	voltage_response.cmd_type = GET_VOLTAGE;
	voltage_response.cmd_value.voltage = voltage;
	charger_qrc_msg_send(voltage_response,sync);
	return;
}


void charging_current_sync_response(){
	float current = 0;
	struct charger_ctl_cmd_s current_response;
	current = get_charging_current_intf();
	current_response.cmd_type = GET_CURRENT;
	current_response.cmd_value.current= current;
	charger_qrc_msg_send(current_response,true);
	return;
}

void charger_pile_stats_sync_response(){
	uint32_t stats;
	struct charger_ctl_cmd_s stats_response;
	stats = (uint32_t)get_charger_pile_signal_stats_intf();
	stats_response.cmd_type = GET_PILE_STATS;
	stats_response.cmd_value.pile_stats= stats;
	charger_qrc_msg_send(stats_response,true);
	return;
}

void charger_sm_stats_sync_response(){
	
	uint32_t stats;
	struct charger_ctl_cmd_s stats_response;
	stats = get_charger_sm_stats_intf();
	stats_response.cmd_type = GET_SM_STATS;
	stats_response.cmd_value.sm_stats= stats;
	charger_qrc_msg_send(stats_response,true);
	return;
}

static void battery_capacity_sync_response(void){

}

static void battery_full_voltage_sync_response(void){

}



static void parse_charger_cmd_cb(void * data, size_t len , bool response)
{
  struct charger_ctl_cmd_s *cmd;

  if (len == sizeof(struct charger_ctl_cmd_s))
    {
      /* get the motion command */
      
      cmd = (struct charger_ctl_cmd_s *)data;

      	switch(cmd.cmd_type){
			
			case START_CHARGING:
				
				charger_start_charging_async_response();
				break;
				
			case STOP_CHARGING:
				
				charger_stop_charging_async_response();
				break;
				
			case GET_VOLTAGE:
				
				battery_voltage_sync_response(response);
				break;
				
			case GET_FULL_BATT_VOLT:
				
				battery_full_voltage_sync_response();
				break;
				
			case GET_CURRENT:
				
				charging_current_sync_response();
				break;
				
			case GET_PILE_STATS:
				
				charger_pile_stats_sync_response();
				break;
				
			case GET_SM_STATS:
				
				charger_sm_stats_sync_response();
				break;
				
			case GET_BATT_CAP:
				
				battery_capacity_sync_response();
				break;

			default:
				break;
	    }
    }
	return;
}

static bool charger_qrc_callback_register(void)
{
  char pipe_name[] = CHARGER_MSG_PIPE;
  struct qrc_pipe_s *pipe;
  bool ret = OK;

  /* get pipe */
  pipe =  qrc_get_pipe(pipe_name);
  if (pipe == NULL)
    {
    syslog(LOG_ERR, "CHARGER: set_charger_msg_callback: qrc get pipe failed!\n");
    ret = ERROR;
    goto errout;
    }

  if (!qrc_register_message_cb(pipe, &parse_charger_cmd_cb))
    {
      syslog(LOG_ERR, "CHARGER: set_charger_msg_callback: register callback failed!\n");
      ret = ERROR;
      goto errout;
    }

    syslog(LOG_INFO, "CHARGER: set_charger_msg_callback: register qrc callback successfully!\n");

errout:
    return ret;
}


static const charger_notify_ops_s charger_notify_ops = {
	.qrc_notify_callback	= 	charger_qrc_async_msg_send_cb,
	.motion_speed_nofity	=	charger_motion_speed_send_cb
};

void charger_polling_debug(){
  float volt;
  float curr;
  uint32_t pile_sig;
  uint32_t sm_stats;
  vx = %.2fmm/s, vz = %.2frad/s\n
  volt = get_battery_voltage_intf();
  syslog(LOG_DEBUG,"CHARGER: charger_polling_debug: BATTERY VOLTAGE:%.2fV!\n",volt);
  curr = get_charging_current_intf();
  syslog(LOG_DEBUG,"CHARGER: charger_polling_debug: CHARGING CURRENT:%.2f!\n",curr);
  pile_sig = get_charger_pile_signal_stats_intf();
  sm_stats = get_charger_sm_stats_intf();
  
  
}

int charger_main_task(int argc, char *argv[])
{
	if(set_charger_msg_callback() != OK){
		syslog(LOG_ERR,"CHARGER: charger_main_task: set charger msg callback failed!\n");
		goto errout;
	}
	if(charger_core_init_intf() != OK){
		syslog(LOG_ERR,"CHARGER: charger_main_task: charger core init failed!\n");
		goto errout;
	}
	if(notification_ops_register_intf(&charger_notify_ops)) 
   {
    syslog(LOG_ERR,"CHARGER: charger_main_task: notification ops register failed!\n");
    goto errout;
   }

	voltage_start_polling_intf();

errout:
  return;
  
}
