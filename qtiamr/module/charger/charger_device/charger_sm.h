/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#ifndef __APP_CHARGER_SM_H
#define __APP_CHARGER_SM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/clock.h>
#include "charger_device.h"
#include "charger_qrc_common.h"

/********************************************************************************
 * Pre-processor Definitions
 ********************************************************************************/

#define CHR_CTL_MODE_TIMEOUT            SEC2TICK(200)
#define WHEEL_SPEED_NOTIFY_20MS         20000
#define CHARGING_POLL_DELAY_1S          1000000
#define BATTERY_FORCE_VOLT              2.5
#define BATTERY_FULL_VOLT               25.2 //25.2v
#define BATTERY_LOW_VOLT                20   //20v
#define CHARGER_SM_PRIORITY             100
#define CHARGER_SM_STACKSIZE            8192
#define CHARGER_SM_STOP_VX_SPEED        0.1 //0.1m/s
#define CHARGER_SM_STOP_VX_ZERO        0.1 //0.1m/s
#define CHARGER_SM_STOP_VZ_ZERO        0    //0 rad/s
#define CHARGER_SM_STOP_DURATION_20MS  20000



enum chr_sm_st_e
{
	CHR_SM_IDLE = 1,
	CHR_SM_SEARCHING,
	CHR_SM_CONTROLLING,
	CHR_SM_FORCE_CHARGING,
	CHR_SM_CHARGING,
	CHR_SM_CHARGER_DONE,
	CHR_SM_ERROR,
	CHR_SM_STATE_MAX
};
/*
typedef struct charger_sm_table_s
{
	int cur_state;
	void (*state_act_fun)();
}chr_sm_tab_t;
*/

typedef struct charger_sm_state {
	bool init;
  pthread_rwlock_t sm_rw_lock;
	enum chr_sm_st_e cur_state;

}chr_sm_state_t;


int set_charger_state(enum state);



#endif /* __APP_CHARGER_SM_H */

