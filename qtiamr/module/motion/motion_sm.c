/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <debug.h>

#include "motion_sm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* motion sm structure */

typedef int (*do_action_fun)(union motion_control_data_u data);  /* action function format */

struct motion_sm_s
{
  enum motion_sm_state_e state;
  pthread_mutex_t mutex;
  motion_cb switch_done_cb;
  motion_cb position_done_cb;
  motion_cb drv_err_cb;
  void *pose_cb_data;
  void *switch_cb_data;
  void *drv_err_cb_data;
}__attribute__((aligned(4)));

struct motion_sm_transform_s
{
  enum motion_sm_event_e event;
  enum motion_sm_state_e current_state;
  enum motion_sm_state_e next_state;
  do_action_fun action_fun;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int motion_sm_do_action(struct motion_sm_transform_s *statetrans, union motion_control_data_u data);

static bool acquire_motion_sm_lock(void);
static void release_motion_sm_lock(void);

static int do_action_switch(union motion_control_data_u data);
static int do_action_emergency(union motion_control_data_u data);
static int do_action_drv_error(union motion_control_data_u data);
static int do_action_speed(union motion_control_data_u data);
static int do_action_switch_done(union motion_control_data_u data);

/* need to do: position done & switch done callback function */

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct motion_sm_s g_motion_sm;

/* inactive */
struct motion_sm_transform_s statetrans_inactive[]={
  {EV_CMD_SWITCH_SPEED, ST_INACTIVE, ST_SWITCHING,  do_action_switch},
  {EV_CMD_SWITCH_POS,   ST_INACTIVE, ST_SWITCHING,  do_action_switch},
  {EV_SPEED_SWITCH_DONE,ST_INACTIVE, ST_INACTIVE,   NULL},
  {EV_POS_SWITCH_DONE,  ST_INACTIVE, ST_INACTIVE,   NULL},
  {EV_CMD_SPEED,        ST_INACTIVE, ST_INACTIVE,   NULL},
  {EV_CMD_POSITION,     ST_INACTIVE, ST_INACTIVE,   NULL},
  {EV_POSITION_ATTACHED,ST_INACTIVE, ST_INACTIVE,   NULL},
  {EV_ENTER_EMERGENCY,  ST_INACTIVE, ST_INACTIVE,   NULL},
  {EV_EXIT_EMERGENCY,   ST_INACTIVE, ST_INACTIVE,   NULL},
  {EV_DRI_ERR,          ST_INACTIVE, ST_DRIVER_ERR, NULL},
};

/* switching */
struct motion_sm_transform_s statetrans_switching[]={
  {EV_CMD_SWITCH_SPEED, ST_SWITCHING, ST_SWITCHING,     NULL},
  {EV_CMD_SWITCH_POS,   ST_SWITCHING, ST_SWITCHING,     NULL},
  {EV_SPEED_SWITCH_DONE,ST_SWITCHING, ST_SPEED,         do_action_switch_done},
  {EV_POS_SWITCH_DONE,  ST_SWITCHING, ST_POSITION_IDLE, do_action_switch_done},
  {EV_CMD_SPEED,        ST_SWITCHING, ST_SWITCHING,     NULL},
  {EV_CMD_POSITION,     ST_SWITCHING, ST_SWITCHING,     NULL},
  {EV_POSITION_ATTACHED,ST_SWITCHING, ST_SWITCHING,     NULL},
  {EV_ENTER_EMERGENCY,  ST_SWITCHING, ST_EMERGENCY,     do_action_emergency},
  {EV_EXIT_EMERGENCY,   ST_SWITCHING, ST_SWITCHING,     NULL},
  {EV_DRI_ERR,          ST_SWITCHING, ST_INACTIVE,      do_action_drv_error},
};

/* speed */
struct motion_sm_transform_s statetrans_speed[]={
  {EV_CMD_SWITCH_SPEED, ST_SPEED, ST_SWITCHING,     do_action_switch},
  {EV_CMD_SWITCH_POS,   ST_SPEED, ST_SWITCHING,     do_action_switch},
  {EV_SPEED_SWITCH_DONE,ST_SPEED, ST_SPEED,         NULL},
  {EV_POS_SWITCH_DONE,  ST_SPEED, ST_SPEED,         NULL},
  {EV_CMD_SPEED,        ST_SPEED, ST_SPEED,         do_action_speed},
  {EV_CMD_POSITION,     ST_SPEED, ST_SPEED,         NULL},
  {EV_POSITION_ATTACHED,ST_SPEED, ST_SPEED,         NULL},
  {EV_ENTER_EMERGENCY,  ST_SPEED, ST_EMERGENCY,     do_action_emergency},
  {EV_EXIT_EMERGENCY,   ST_SPEED, ST_SPEED,         NULL},
  {EV_DRI_ERR,          ST_SPEED, ST_INACTIVE,      do_action_drv_error},
};

/* emergency */
struct motion_sm_transform_s statetrans_emergency[]={
  {EV_CMD_SWITCH_SPEED, ST_EMERGENCY, ST_EMERGENCY,     NULL},
  {EV_CMD_SWITCH_POS,   ST_EMERGENCY, ST_EMERGENCY,     NULL},
  {EV_SPEED_SWITCH_DONE,ST_EMERGENCY, ST_EMERGENCY,     NULL},
  {EV_POS_SWITCH_DONE,  ST_EMERGENCY, ST_EMERGENCY,     NULL},
  {EV_CMD_SPEED,        ST_EMERGENCY, ST_EMERGENCY,     NULL},
  {EV_CMD_POSITION,     ST_EMERGENCY, ST_EMERGENCY,     NULL},
  {EV_POSITION_ATTACHED,ST_EMERGENCY, ST_EMERGENCY,     NULL},
  {EV_ENTER_EMERGENCY,  ST_EMERGENCY, ST_EMERGENCY,     NULL},
  {EV_EXIT_EMERGENCY,   ST_EMERGENCY, ST_INACTIVE,      do_action_emergency},
  {EV_DRI_ERR,          ST_EMERGENCY, ST_INACTIVE,      do_action_drv_error},
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool acquire_motion_sm_lock(void)
{
  int status;

  status = status = pthread_mutex_lock(&g_motion_sm.mutex);

  if (status == 0)
    return true;
  else
    return false;
}

static void release_motion_sm_lock(void)
{
  int status;
  status = pthread_mutex_unlock(&g_motion_sm.mutex);
  if (status != 0)
    {
      syslog(LOG_ERR,"pthread_rwlock:"
                      "ERROR Failed to unlock lock. Status: %d\n", status);
      ASSERT(false);
    }
}

static int motion_sm_do_action(struct motion_sm_transform_s *statetrans, union motion_control_data_u data)
{
  enum motion_sm_state_e *curr_state;
  int result;

  if(NULL == statetrans)
    {
      syslog(LOG_ERR,"pointer statetrans is invalid \n");
      return ERROR;
    }

  /*lock sm */
  if (!acquire_motion_sm_lock())
    {
      release_motion_sm_lock();
      syslog(LOG_ERR,"acquire motion sm lock failed\n");
      return ERROR;
    }

  curr_state = &g_motion_sm.state;
  /* check again the present state if matched */
  if (*curr_state == statetrans->current_state)
    {
      *curr_state = statetrans->next_state;
      release_motion_sm_lock();
    }
  else
    {
      release_motion_sm_lock();
      syslog(LOG_INFO,"motion sm current state is changed=%d \n", *curr_state);
      return ERROR;
    }

  /* do action */

  if(statetrans->action_fun != NULL)
    {
      /* call action function */
      result = statetrans->action_fun(data);
      return result;
    }
  else
    {
      syslog(LOG_INFO,"motion sm no action \n");
      return OK;
    }
}


/****************************************************************************
 * Action functions
 ****************************************************************************/

static int do_action_switch(union motion_control_data_u data)
{
  enum control_mode_e mode = data.mode;
  int result;

  /* call motor api to set motor mode */
  result = motor_switch_mode(mode);
  if (OK == result)
    {
      if (SPEED == mode)
        {
          /* send event to notify switch done */
          motion_sm_event(EV_SPEED_SWITCH_DONE, data);
        }
      else if (POSITION == mode)
        {
          motion_sm_event(EV_POS_SWITCH_DONE, data);
        }
    }

  return result;
}

static int do_action_emergency(union motion_control_data_u data)
{
  bool emergency = data.emergency;

  return motor_quick_stop(emergency);
}

static int do_action_drv_error(union motion_control_data_u data)
{
  /* try stop motor driver */
  motor_quick_stop(true);
}

static int do_action_speed(union motion_control_data_u data)
{
  float vx = data.speed_cmd.vx;
  float vz = data.speed_cmd.vz;

  return motor_set_speed(vx, vz);
}

static int do_action_switch_done(union motion_control_data_u data)
{
  /* switch done call back*/
  if (NULL != g_motion_sm.switch_done_cb)
    {
      g_motion_sm.switch_done_cb(g_motion_sm.switch_cb_data);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: get_motion_sm_state
 ****************************************************************************/

enum motion_sm_state_e get_motion_sm_state(void)
{
  enum motion_sm_state_e present_state;

	  /*lock sm */
  if (!acquire_motion_sm_lock())
    {
      release_motion_sm_lock();
      syslog(LOG_ERR,"acquire motion sm lock failed\n");
      return ERROR;
    }
  present_state = g_motion_sm.state;
  release_motion_sm_lock();
  return present_state;
}

int motion_sm_event(enum motion_sm_event_e event ,union motion_control_data_u data)
{
  enum motion_sm_state_e present_state;
  int result;

  present_state = get_motion_sm_state();

  switch (present_state)
    {
      case ST_INACTIVE:
        {
          result = motion_sm_do_action(&statetrans_inactive[event],data);
          break;
        }
      case ST_SWITCHING:
        {
          result = motion_sm_do_action(&statetrans_switching[event],data);
          break;
        }
      case ST_SPEED:
        {
          result = motion_sm_do_action(&statetrans_speed[event],data);
          break;
        }
      case ST_POSITION_IDLE:
      case ST_POSITION_RUNNING:
        {
		      syslog(LOG_ERR, "motion state in state POSITION\n");
          break;
        }
      case ST_EMERGENCY:
        {
          result = motion_sm_do_action(&statetrans_emergency[event],data);
          break;
        }
      case ST_DRIVER_ERR:
        {
          syslog(LOG_ERR, "motion state in state ST_DRIVER_ERR\n");
          break;
        }
      default:
        {
          syslog(LOG_ERR, "motion state invalid state=%d\n", present_state);
          break;
        }
    }

  return result;
}

void register_motion_switch_done_cb(motion_cb cb_fun, void *arg)
{
  g_motion_sm.switch_done_cb = cb_fun;
  g_motion_sm.switch_cb_data = arg;
}

void register_motion_odom_done_cb(motion_cb cb_fun, void *arg)
{
  return ERROR;
}

int motion_sm_init(void)
{
  int status;

  status = pthread_mutex_init(&g_motion_sm.mutex, NULL);
  if (status != 0)
    {
      syslog(LOG_ERR,"ERROR pthread_mutex_init failed, status=%d\n",status);
      ASSERT(false);
    }

  /* init state as inactive */
  g_motion_sm.state = ST_INACTIVE;

  return status;
}