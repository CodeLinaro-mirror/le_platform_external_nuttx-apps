/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/


#include "charger_device.h"



bool get_voltage_hal(charger_dev_t *charger_dev, float *voltage){
	if(charger_dev->initialized != TRUE)
		return ERROR;
	if((charger_dev->drv_ops!= NULL)&&(charger_dev->drv_ops->get_voltage != NULL))
		return charger_dev->drv_ops->get_voltage(voltage);
	return ERROR;
}

bool get_charging_current_hal(charger_dev_t *charger_dev, float *current){
	if(charger_dev->initialized != TRUE)
		return ERROR;
	if((charger_dev->drv_ops!= NULL)&&(charger_dev->drv_ops->get_current != NULL))
		return charger_dev->drv_ops->get_current(current);
	return ERROR;
}

bool get_wheel_speed_hal(charger_dev_t *charger_dev, float *vx, float *vz){
	if(charger_dev->initialized != TRUE)
		return ERROR;
	if((charger_dev->drv_ops!= NULL)&&(charger_dev->drv_ops->get_wheel_speed != NULL))
		return charger_dev->drv_ops->get_wheel_speed(vx, vz);
	return ERROR;
}

bool get_pile_sig_stats(charger_dev_t *charger_dev, bool *stats){
	if(charger_dev->initialized != TRUE)
		return ERROR;
	if((charger_dev->drv_ops!= NULL)&&(charger_dev->drv_ops->chr_pile_sig_stat != NULL))
		return charger_dev->drv_ops->chr_pile_sig_stat(stats);
	return ERROR;
}

bool charger_if_charging(charger_dev_t *charger_dev, bool *stats){
	if(charger_dev->initialized != TRUE)
		return ERROR;
	if((charger_dev->drv_ops!= NULL)&&(charger_dev->drv_ops->chr_pile_sig_stat != NULL))
		return charger_dev->drv_ops->ec130_if_charging_stat(stats);
	return ERROR;
}

chr_dev_tab_t charger_drv_table[] =
{
	{ "ec130", ec130_and_adc_driver_init},
	//add more driver here
};


bool charger_dirver_init_hal(const char *name){

	int drv_num;
	int i;
	int ret;
	bool (*drv_act_fun)() = NULL;
	drv_num = sizeof(charger_drv_table) / sizeof(chr_dev_tab_t);
	for(i=0;i<drv_num;i++){
		if (!strcmp(name, &charger_drv_table->name))
			drv_act_fun = charger_drv_table[i].drv_init_func;
			break;
	}
	if(drv_act_fun == NULL)
  { 
    syslog(LOG_ERR,"CHARGER: charger_dirver_init_hal:(%s) driver not found\n",name);
		return ERROR;
  }
	syslog(LOG_DEBUG,"CHARGER: charger_dirver_init_hal:(%s) driver found\n",name);
  if(drv_act_fun() == OK)
    syslog(LOG_DEBUG,"CHARGER: charger_dirver_init_hal:(%s) driver init successfully\n",name);
	return ret;
}


