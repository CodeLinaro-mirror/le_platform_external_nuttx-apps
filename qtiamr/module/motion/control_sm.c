/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <debug.h>
#include <pthread.h>

#include "motion_management.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DEFAULT_CLIENT  (ST_ROBOT_CONTROLLING)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* control state machine state */

enum control_sm_state_e
{
  ST_ROBOT_CONTROLLING = 0x00,
  ST_CHARGER_CONTROLLING,
  ST_REMOTE_CONTROLLING
};

/* control state machine event */

enum control_sm_event_e
{
  EV_CMD_ROBOT_CONTROL = 0x00,
  EV_CMD_CHARGER_CONTROL,
  EV_CMD_REMOTE_CONTROL
};

/* control state machine no action */
struct control_sm_transform_s
{
  enum control_sm_event_e event;
  enum control_sm_state_e current_state;
  enum control_sm_state_e next_state;
};

/* control sm structure */

struct control_sm_s
{
  enum control_sm_state_e control_state;
  pthread_rwlock_t control_rwlock;
  control_sm_notify_cb cb_list[MAX_CLIENT];
}__attribute__((aligned(4)));

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void client_sm_change_state(enum control_sm_e state);
static int client_sm_state(enum control_sm_e state *state);
static enum control_sm_e client_sm_state(void);
static int client_contrl_sm_event(enum control_sm_event_e event);

static int control_sm_state_trans(struct control_sm_transform_s *statetrans);
static void do_action(enum control_sm_state_e state);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct control_sm_s g_control_sm;

struct control_sm_transform_s statetrans_robot_control[]={
  {EV_CMD_ROBOT_CONTROL,    ST_ROBOT_CONTROLLING,   ST_ROBOT_CONTROLLING},
  {EV_CMD_CHARGER_CONTROL,  ST_ROBOT_CONTROLLING,   ST_CHARGER_CONTROLLING},
  {EV_CMD_REMOTE_CONTROL,   ST_ROBOT_CONTROLLING,   ST_REMOTE_CONTROLLING},
};

struct control_sm_transform_s statetrans_charger_control[]={
  {EV_CMD_ROBOT_CONTROL,    ST_CHARGER_CONTROLLING, ST_ROBOT_CONTROLLING},
  {EV_CMD_CHARGER_CONTROL,  ST_CHARGER_CONTROLLING, ST_CHARGER_CONTROLLING},
  {EV_CMD_REMOTE_CONTROL,   ST_CHARGER_CONTROLLING, ST_REMOTE_CONTROLLING},
};

struct control_sm_transform_s statetrans_remote_control[]={
  {EV_CMD_ROBOT_CONTROL,    ST_REMOTE_CONTROLLING, ST_ROBOT_CONTROLLING},
  {EV_CMD_CHARGER_CONTROL,  ST_REMOTE_CONTROLLING, ST_CHARGER_CONTROLLING},
  {EV_CMD_REMOTE_CONTROL,   ST_REMOTE_CONTROLLING, ST_REMOTE_CONTROLLING},
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int control_sm_state_trans(struct control_sm_transform_s *statetrans)
{
  enum control_sm_state_e *curr_state;
  int result;

  if(NULL == statetrans)
    {
      syslog(LOG_ERR,"pointer statetrans is invalid \n");
      return ERROR;
    }

  /* lock */
  status = pthread_rwlock_wrlock(&g_control_sm.control_rwlock);
  if (status != 0)
    {
      syslog(LOG_ERR,"pthread_rwlock: "
             "ERROR Failed to open rwlock for control_sm_state_trans. Status: %d\n",
             status);
      ASSERT(false);
    }

  curr_state = &g_control_sm.control_state;
  if (*curr_state == statetrans->current_state)
    {
      *curr_state = statetrans->next_state;
      do_action(*curr_state);
    }

  /* unlock */
  status = pthread_rwlock_unlock(&g_control_sm.control_rwlock);
  if (status != 0)
    {
      syslog(LOG_ERR,"pthread_rwlock: "
             "ERROR Failed to unlock lock. Status: %d\n", status);
      ASSERT(false);
    }

  return OK;
}

/* control sm action function */
static void do_action(enum control_sm_state_e state)
{
  int num;

  for(num = 0; num < MAX_CLIENT; num++)
    {
      control_sm_notify_cb cb_list[MAX_CLIENT];
      if (NULL != cb_list[num])
        {
          cb_list[num](state);
        }
    }
}

static int client_sm_state(enum control_sm_state_e *state)
{
  int result;

  status = pthread_rwlock_tryrdlock(&g_control_sm.control_rwlock);
  if (status == 0)
    {
      *state = g_control_sm.control_state;
    }

  status = pthread_rwlock_unlock(&g_control_sm.control_rwlock);
  if (status != 0)
    {
      syslog(LOG_ERR,"pthread_rwlock: "
             "ERROR Failed to unlock lock. Status: %d\n", status);
      ASSERT(false);
    }
  return status;
}

/****************************************************************************
 * Name: client_contrl_sm_event
 ****************************************************************************/

static int client_contrl_sm_event(enum control_sm_event_e event)
{
  enum control_sm_state_e state;
  int result;

  result = client_sm_state(&state);

  if (result == 0)
    {
      switch (state)
        {
          case ST_ROBOT_CONTROLLING:
            {
              result = control_sm_state_trans(&statetrans_robot_control[event]);
              break;
            }
          case ST_CHARGER_CONTROLLING:
            {
              result = control_sm_state_trans(&statetrans_robot_control[event]);
              break;
            }
          case ST_REMOTE_CONTROLLING:
            {
              result = control_sm_state_trans(&statetrans_robot_control[event]);
              break;
            }
          default :
            syslog(LOG_ERR, "ERROR client_sm_state , state=%d\n", state);
            break;
        }
    }

  return result;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: client_sm_init
 ****************************************************************************/

void client_sm_init(void)
{
  int status;
  int i;
  status = pthread_rwlock_init(&g_control_sm.control_rwlock, NULL);
  if (status != 0)
    {
      syslog(LOG_ERR, "ERROR pthread_rwlock_init failed, status=%d\n", status);
      ASSERT(false);
    }

  /* Set default control client state */
  g_control_sm.control_state = DEFAULT_CLIENT;

  for (i=0; i<MAX_CLIENT; i++)
    {
      g_control_sm.cb_list[i] = NULL;
    }
}

/****************************************************************************
 * Name: get_current_client
 ****************************************************************************/

enum control_client_e get_current_client(void)
{
  int result;
  enum control_sm_state_e state;
  enum control_client_e client = MAX_CLIENT;

  result = client_sm_state(&state);
  if (result == 0)
    {
      switch (state)
        {
          case ST_ROBOT_CONTROLLING:
            {
              client = ROBOT_CONTROLLER;
              break;
            }
          case ST_CHARGER_CONTROLLING:
            {
              client = CHARGER_CONTROLLER;
              break;
            }
          case ST_REMOTE_CONTROLLING:
            {
              client = REMOTE_CONTROLLER;
              break;
            }
          default :
            syslog(LOG_ERR, "ERROR client_sm_state , state=%d\n", state);
            break;
        }
    }

  return client;
}

/****************************************************************************
 * Name: set_control_client
 ****************************************************************************/

int set_control_client(enum control_client_e client)
{
  enum control_sm_event_e event;

  switch (client)
    {
      case ROBOT_CONTROLLER:
        {
          event = EV_CMD_ROBOT_CONTROL;
          break;
        }
      case CHARGER_CONTROLLER:
        {
          event = EV_CMD_CHARGER_CONTROL;
          break;
        }
      case REMOTE_CONTROLLER:
        {
          event = EV_CMD_REMOTE_CONTROL;
          break;
        }
      default :
        {
          syslog(LOG_ERR, "ERROR client =%d\n", client);
          return ERROR;
        }

    }

  return client_contrl_sm_event(EV_CMD_ROBOT_CONTROL);
}
