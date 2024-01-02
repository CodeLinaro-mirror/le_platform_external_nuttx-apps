/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#ifndef __APP_CHARGER_DEVICE_H
#define __APP_CHARGER_DEVICE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <signal.h>
#include <nuttx/signal.h>
#include "amr_signal.h"

/********************************************************************************
 * Pre-processor Definitions
 ********************************************************************************/

#define CHARGER_DEVICE_NAME_SIZE  20
#define BATT_VOLT_NOTIFY_2S       2000000


typedef struct charger_driver_ops_s {
	CODE int (*get_voltage)(float *volt);
	CODE bool (*get_current)(float *curr);
	CODE bool (*get_wheel_speed)(float *vx, float *vz);
	CODE int (*set_def_speed)(float vx, float vz);
	CODE int (*chr_pile_sig_stat)(bool *stat);
	CODE int (*chr_if_charging)(bool *stat);
}chr_drv_ops_t;

typedef struct charger_notify_ops_s {
	CODE int (*qrc_notify_callback)(void * data, size_t len, bool ack);
	CODE int (*motion_speed_nofity)(float vx, float vz);
}chr_notify_ops_t;

struct voltage_cmd_s
{
  uint8_t cmd_type;
  float   cmd_value;
};


typedef struct charger_device_s {
	struct amr_signal_s charger_signal;
	bool initialized;
	char name[CHARGER_DEVICE_NAME_SIZE];
	chr_sm_state_t sm;
	const chr_notify_ops_t *nfy_ops;
	const chr_drv_ops_t *drv_ops;
  pthread_mutex_t cc_mutex;

  
}__attribute__((align(4)))charger_dev_t;


void chr_core_drv_ops_register(const char *name,const chr_drv_ops_t *ops);
void chr_core_notify_ops_register(const chr_drv_ops_t *ops);
float chr_core_get_voltage();
float chr_core_get_charging_curr();
bool chr_core_get_pile_stats();

int chr_core_sm_signal_init();
int chr_core_sm_signal_send();
int chr_core_sm_signal_wait(struct siginfo *info, FAR const struct timespec *timeout);



#endif /* __APP_CHARGER_DEVICE_H */
