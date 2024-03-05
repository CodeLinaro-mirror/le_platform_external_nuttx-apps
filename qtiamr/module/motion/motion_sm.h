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

#include "motion_msg.h"
#include "motion_management.h"
#include "motor_management.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

typedef void (*motion_cb)(void *user,int data);  /* motion cb for user */

typedef int (*do_action_fun)(union motion_control_data_u data);  /* action function format */

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

enum motion_sm_state_e get_motion_sm_state(void);

int motion_sm_event(enum motion_sm_event_e event ,union motion_control_data_u data);

void register_motion_switch_done_cb(motion_cb cb_fun, void *arg);
void register_motion_emergency_done_cb(motion_cb cb_fun, void *arg);
void register_motion_drv_err_cb(motion_cb cb_fun, void *arg);

void motion_sm_stop_speed(bool stop);
void register_motor_emergency_check_cb(emergency_speed_check_cb check_cb);

int motion_sm_init(void);


/* motion thread pool */
struct motion_args_s
{
  do_action_fun fun_cb;  /* qrc_msg_cb */
  union motion_control_data_u data;
};

typedef struct motion_thread_pool_s * motion_thread_pool;
typedef void (*motion_work)(struct motion_args_s args);

struct motion_thread_pool_s * motion_thread_pool_init(int num);
int motion_threadpool_add_work(struct motion_thread_pool_s * thpool, motion_work work_fun, struct motion_args_s args);
void motion_threadpool_wait(struct motion_thread_pool_s * thpool);
void motion_threadpool_destroy(struct motion_thread_pool_s * thpool);

void motion_threads_join(struct motion_thread_pool_s * thpool);

void motion_sm_join(void);

void motion_add_work(do_action_fun work_fun, union motion_control_data_u data);
#endif /* __MOTION_SM_H */