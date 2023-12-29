/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#ifndef __MOTION_SM_H
#define __MOTION_SM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

typedef void (*motion_cb)(void *user);  /* motion cb for user */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* motion state machine state */

enum motion_sm_state_e
{
  ST_INACTIVE = 0x00,
  ST_SWITCHING,
  ST_SPEED,
  ST_POSITION_IDLE,
  ST_POSITION_RUNNING,
  ST_EMERGENCY,
  ST_DRIVER_ERR
};

enum motion_sm_event_e
{
  EV_CMD_SWITCH_SPEED = 0x00,
  EV_CMD_SWITCH_POS,
  EV_SPEED_SWITCH_DONE,
  EV_POS_SWITCH_DONE,
  EV_CMD_SPEED,
  EV_CMD_POSITION,
  EV_POSITION_ATTACHED,
  EV_ENTER_EMERGENCY,
  EV_EXIT_EMERGENCY,
  EV_DRI_ERR
};


/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Motion sm */

enum motion_sm_e get_motion_sm_state(void);

int motion_sm_event(enum motion_sm_event_e event,struct motion_control_data_s data);

void register_motion_switch_done_cb(motion_cb cb_fun, void *arg);
void register_motion_odom_done_cb(motion_cb cb_fun, void *arg);

#endif /* __MOTION_SM_H */