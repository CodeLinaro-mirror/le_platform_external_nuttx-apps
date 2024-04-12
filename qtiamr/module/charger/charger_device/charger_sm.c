/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#include <stdio.h>
#include <errno.h>
#include <syslog.h>
#include <signal.h>
#include <assert.h>
#include <pthread.h>
#include <nuttx/clock.h>

#include "charger_sm.h"
#include "charger_device.h"

extern struct charger_device_s g_charger_dev;

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void    set_curr_state(enum chr_sm_st_e state);
static int32_t state_available(int state);
static int32_t sm_enter_state(enum chr_sm_st_e state);
static void    sm_init(void);
static int     sm_task(int argc, FAR char *argv[]);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct charger_device_s *charger_dev_p = &g_charger_dev;

/****************************************************************************
 * Public Data
 ****************************************************************************/

const char *sm_event_labels[] = {
  [SM_EVENT_START_CHARGING]     = "start charging event",
  [SM_EVENT_FIND_PILE]          = "find pile event",
  [SM_EVENT_ATTACH_PILE]        = "attached pile event",
  [SM_EVENT_TO_NORMAL_CHARGING] = "nomal charging event",
  [SM_EVENT_STOP_CHARGING]      = "stop charging event",
  [SM_EVENT_BACK_TO_IDLE]       = "back to idle event",
  [SM_EVENT_EXCEPTION]          = "exception event",
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void set_curr_state(enum chr_sm_st_e state)
{

  pthread_rwlock_t *rw_lock = &charger_dev_p->sm.sm_rw_lock;

  charger_dev_rwlock_wrlock_acquire(rw_lock);
  charger_dev_p->sm.cur_state = state;
  charger_dev_rwlock_release(rw_lock);
  return;
}

static int32_t state_available(int state)
{

  int ret = OK;
  switch (get_curr_state())
    {

      case CHR_SM_IDLE:
        if (state != CHR_SM_SEARCHING && state != CHR_SM_EXCEPTION)
          ret = ERROR;
        break;
      case CHR_SM_SEARCHING:
        if (state != CHR_SM_IDLE && state != CHR_SM_CONTROLLING && state != CHR_SM_EXCEPTION)
          ret = ERROR;
        break;
      case CHR_SM_CONTROLLING:
        if (state != CHR_SM_IDLE && state != CHR_SM_FORCE_CHARGING && state != CHR_SM_CHARGER_DONE && state != CHR_SM_EXCEPTION)
          ret = ERROR;
        break;
      case CHR_SM_FORCE_CHARGING:
        if (state != CHR_SM_CHARGING && state != CHR_SM_EXCEPTION)
          ret = ERROR;
        break;
      case CHR_SM_CHARGING:
        if (state != CHR_SM_IDLE && state != CHR_SM_CHARGER_DONE && state != CHR_SM_EXCEPTION)
          ret = ERROR;
        break;
      case CHR_SM_CHARGER_DONE:
        if (state != CHR_SM_IDLE && state != CHR_SM_EXCEPTION)
          ret = ERROR;
        break;
      case CHR_SM_EXCEPTION:
        ret = ERROR;
        break;

      default:
        break;
    }

  return ret;
}

static int32_t sm_enter_state(enum chr_sm_st_e state)
{

  int ret;

  ret = state_available(state);
  if (ret == ERROR)
    {
      charger_exception_notify(CHARGER_SM_STATE_ERROR);
      return ret;
    }
  set_curr_state(state);
  sm_state_notify(state);
  return ret;
}

static void sm_init(void)
{

  pthread_rwlock_t *rw_lock = &charger_dev_p->sm.sm_rw_lock;

  charger_dev_sm_signal_init();
  charger_dev_rwlock_init(rw_lock);

  set_curr_state(CHR_SM_IDLE);
  charger_dev_p->sm.sm_task_started = TRUE;
  return;
}

static int sm_task(int argc, FAR char *argv[])
{

  struct siginfo sig_value;
  int32_t        event;
  uint32_t       store_polling;

  syslog(LOG_INFO, "CHARGER: sm_task: Running...\n");
  usleep(500 * 1000L);
  sm_init();

  while (1)
    {
      memset(&sig_value, 0, sizeof(struct siginfo));
      event = 0;
      syslog(LOG_INFO, "CHARGER: sm_task: waiting for signal...\n");
      if (charger_dev_sm_signal_wait(&sig_value, NULL) == ERROR)
        {
          goto errout;
        }
      event = sig_value.si_value.sival_int;
      syslog(LOG_INFO, "CHARGER: sm_task: received signal event(%s)\n", sm_event_labels[event]);
      switch (event)
        {

          case SM_EVENT_START_CHARGING:
            if (sm_enter_state(CHR_SM_SEARCHING) == OK)
              {
                set_polling_interval(500 * 1000);
              }
            break;

          case SM_EVENT_FIND_PILE:
            store_polling = get_polling_interval();
            set_polling_interval(CONTROLLING_SPEED_NOTIFY);
            if (sm_enter_state(CHR_SM_CONTROLLING) == ERROR)
              {
                set_polling_interval(store_polling);
              }
            break;

          case SM_EVENT_ATTACH_PILE:
            store_polling = get_polling_interval();
            set_polling_interval(CHARGING_POLL_DELAY);
            if (sm_enter_state(CHR_SM_FORCE_CHARGING) == ERROR)
              {
                set_polling_interval(store_polling);
              }
            break;

          case SM_EVENT_TO_NORMAL_CHARGING:
            store_polling = get_polling_interval();
            set_polling_interval(CHARGING_POLL_DELAY);
            if (sm_enter_state(CHR_SM_CHARGING) == ERROR)
              {
                set_polling_interval(store_polling);
              }

            break;

          case SM_EVENT_STOP_CHARGING:
            store_polling = get_polling_interval();
            set_polling_interval(CHARGING_DONE_DELAY);
            if (sm_enter_state(CHR_SM_CHARGER_DONE) == ERROR)
              {
                set_polling_interval(store_polling);
              }
            break;

          case SM_EVENT_BACK_TO_IDLE:
            store_polling = get_polling_interval();
            set_polling_interval(IDLE_BATT_VOLT_NOTIFY);
            if (sm_enter_state(CHR_SM_IDLE) == ERROR)
              {
                set_polling_interval(store_polling);
              }
            break;

          case SM_EVENT_EXCEPTION:
            store_polling = get_polling_interval();
            set_polling_interval(CHARGER_EXCEPTION_DELAY);
            if (sm_enter_state(CHR_SM_EXCEPTION) == ERROR)
              {
                set_polling_interval(store_polling);
              }
            break;

          default:
            break;
        }
    }
errout:
  charger_dev_p->sm.sm_task_started = FALSE;
  syslog(LOG_ERR, "charger sm_task: Terminating\n");
  return ERROR;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

uint32_t get_curr_state(void)
{
  uint32_t          state;
  pthread_rwlock_t *rw_lock = &charger_dev_p->sm.sm_rw_lock;
  charger_dev_rwlock_rdlock_acquire(rw_lock);
  state = (uint32_t)charger_dev_p->sm.cur_state;
  charger_dev_rwlock_release(rw_lock);
  return state;
}

void sm_start_charging(void)
{
  syslog(LOG_DEBUG, "CHARGER: sm_start_charging: start charging...\n");
  if (charger_dev_p->initialized == TRUE && charger_dev_p->sm.sm_task_started == TRUE)
    {
      charger_dev_sm_signal_send((int32_t)SM_EVENT_START_CHARGING);
    }
  else
    {
      charger_exception_notify(CHARGER_DEV_NOT_READY_ERROR);
    }
}

void sm_stop_charging(void)
{
  syslog(LOG_DEBUG, "CHARGER: sm_stop_charging: stop charging...\n");
  if (charger_dev_p->initialized == TRUE && charger_dev_p->sm.sm_task_started == TRUE)
    {
      charger_dev_sm_signal_send((int32_t)SM_EVENT_STOP_CHARGING);
    }
  else
    {
      charger_exception_notify(CHARGER_DEV_NOT_READY_ERROR);
    }
}

int32_t start_sm_task(void)
{
  pid_t sm_task_pid;

  syslog(LOG_INFO, "CHARGER: start_sm_task: create task(%d)\n", charger_dev_p->sm.sm_task_started);
  if (charger_dev_p->sm.sm_task_started == FALSE)
    {
      sm_task_pid = task_create("sm_task", CHARGER_SM_PRIORITY,
                                CHARGER_SM_STACKSIZE, sm_task, NULL);
      if (sm_task_pid < 0)
        {
          syslog(LOG_ERR, "CHARGER: start_sm_task: ERROR: Failed to start sm_task: %d\n", errno);
          return ERROR;
        }
      usleep(500 * 1000L);
      syslog(LOG_INFO, "CHARGER: start_sm_task: state machine start successfully\n");
    }

  return OK;
}
