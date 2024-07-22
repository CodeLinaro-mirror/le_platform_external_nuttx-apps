/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#include <stdio.h>
#include <pthread.h>
#include <syslog.h>
#include <signal.h>
#include <assert.h>

#include "charger_device.h"
#include "charger_sm.h"
#include "charger_hal.h"
#include "amr_signal.h"
#include "misc_msg.h"

//#define CHARGER_DEBUG
extern const char *sm_event_labels[];

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/*battery property*/
#define BATTERY_FORCE_VOLT_PERCETAGE (0.4)
#define BATTERY_FULL_VOLT            24.5
#define BATTERY_LOW_VOLT             22
#define BATTERY_VOLTAGE_NOTIFY_THRES 0.2 /*this value need to debug*/

/*define charger driver name for use*/
#define CHARGER_DEVICE_NAME "ec130"

/*move forward and stop charging param*/
#define CHARGER_SM_STOP_VX_SPEED 0.15 //m/s
#define CHARGER_SM_STOP_VX_ZERO  0    //m/s
#define CHARGER_SM_STOP_VZ_ZERO  0    //rad/s

#define CHARGER_DEV_TRYING_COUNT 5

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int32_t charger_dev_get_speed(float *vx, float *vz);
static void    charger_send_speed(void);

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct charger_device_s g_charger_dev;

/****************************************************************************
 * Private Data
 ****************************************************************************/
static struct charger_device_s *charger_dev_p = &g_charger_dev;

static const char *sm_state_labels[] = {
  [CHR_SM_IDLE]           = "idle",
  [CHR_SM_SEARCHING]      = "searching",
  [CHR_SM_CONTROLLING]    = "controlling",
  [CHR_SM_FORCE_CHARGING] = "force charging",
  [CHR_SM_CHARGING]       = "charging",
  [CHR_SM_CHARGER_DONE]   = "charging done",
  [CHR_SM_EXCEPTION]      = "sm exception",
};

static const char *exception_labels[] = {
  [CHARGER_VOLTAGE_ERROR]          = "exception: voltage error!",
  [CHARGER_CHARGING_CURRENT_ERROR] = "exception: current error!",
  [CHARGER_PILE_STAT_ERROR]        = "exception: pile state error!",
  [CHARGER_IS_CHARGING_ERROR]      = "exception: is charging error!",
  [CHARGER_GET_SPEED_ERROR]        = "exception: get speed error!",
  [CHARGER_SM_STATE_ERROR]         = "exception: sm state error!",
  [CHARGER_DEV_NOT_READY_ERROR]    = "exception: device not ready error!",
  [CHARGER_ENTER_EXCEPTION]        = "exception: common error!",
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void charger_find_pile_event_in_searching(void)
{
  bool pile_stats;
  int32_t ret;
  clock_t elapsed;
  clock_t start;
  start = clock_systime_ticks();
  while(1)
    {
      ret = charger_dev_get_pile_stats(&pile_stats);
      if(ret != OK)
      {
        charger_exception_notify(CHARGER_PILE_STAT_ERROR);
        if(charger_dev_p->sm.sm_task_started)
          charger_dev_sm_signal_send((int32_t)SM_EVENT_EXCEPTION);
        return;
      }
      elapsed = clock_systime_ticks() - start;
      syslog(LOG_DEBUG, "CHARGER: searching state: pile_stats = (%d)! timeout = %ld\n", pile_stats, TICK2SEC(elapsed));
      if(pile_stats == TRUE)
        {
          if(charger_dev_p->sm.sm_task_started)
            {
              charger_dev_sm_signal_send((int32_t)SM_EVENT_FIND_PILE);
              break;
            }
        }
      else
        {
          if(elapsed >= CHR_SEARCH_MODE_TIMEOUT)
            {
              if(charger_dev_p->sm.sm_task_started)
                {
                  charger_dev_sm_signal_send((int32_t)SM_EVENT_BACK_TO_IDLE);
                  break;
                }
            }
          sleep(1);
        }
    }
}

static void charger_pile_attached_event_in_controlling(void)
{
  bool    is_charging;
  bool    pile_stats;
  int32_t ret;
  int     i;
  ret = charger_dev_get_all_stats(NULL, NULL, &pile_stats, &is_charging);
  if (ret != OK)
    {
      charger_exception_notify(CHARGER_ENTER_EXCEPTION);
      if (charger_dev_p->sm.sm_task_started)
        charger_dev_sm_signal_send((int32_t)SM_EVENT_EXCEPTION);
      return;
    }
  /*
  if(pile_stats == FALSE)
    {
      if(charger_dev_p->sm.sm_task_started)
      {
        charger_dev_sm_signal_send((int32_t)SM_EVENT_BACK_TO_IDLE);
      }
      motion_speed_notify(CHARGER_SM_STOP_VX_ZERO, CHARGER_SM_STOP_VZ_ZERO);
      syslog(LOG_DEBUG, "CHARGER: controlling state: loss pile signal, back to idle!!!!\n");
      return;
    }*/
  if (is_charging == TRUE)
    {
      /*stop the car*/
      motion_speed_notify(CHARGER_SM_STOP_VX_ZERO, CHARGER_SM_STOP_VZ_ZERO);
      for (i = 0; i < CHARGER_DEV_TRYING_COUNT; i++)
        {
          sleep(2);

          /*re-check charging status*/
          ret = charger_dev_get_all_stats(NULL, NULL, &pile_stats, &is_charging);
          if (ret != OK)
            {
              charger_exception_notify(CHARGER_ENTER_EXCEPTION);
              if (charger_dev_p->sm.sm_task_started)
                charger_dev_sm_signal_send((int32_t)SM_EVENT_EXCEPTION);
              return;
            }
          syslog(LOG_DEBUG, "CHARGER: controlling state: is_charging = (%d) trying(%d)!\n", is_charging, i);

          /*if disconnect, try again with turn around a small angle*/
          if (is_charging == FALSE)
            {
              motion_speed_notify(CHARGER_SM_STOP_VX_ZERO, 0.15);
              usleep(500000);
              motion_speed_notify(CHARGER_SM_STOP_VX_ZERO, CHARGER_SM_STOP_VZ_ZERO);
              usleep(50000);
              motion_speed_notify(-0.05, CHARGER_SM_STOP_VZ_ZERO);
              usleep(50000);
              motion_speed_notify(CHARGER_SM_STOP_VX_ZERO, CHARGER_SM_STOP_VZ_ZERO);
            }
          else
            {
              if (charger_dev_p->sm.sm_task_started && i == CHARGER_DEV_TRYING_COUNT - 1)
                {
                  charger_dev_sm_signal_send((int32_t)SM_EVENT_ATTACH_PILE);
                }
            }
        }
    }
  else
    {
      charger_send_speed();
    }
}
static void charger_send_speed(void)
{
  float   vx;
  float   vz;
  int32_t ret;
  ret = charger_dev_get_speed(&vx, &vz);
  if (ret != OK)
    {
      charger_exception_notify(CHARGER_GET_SPEED_ERROR);
      if (charger_dev_p->sm.sm_task_started)
        charger_dev_sm_signal_send((int32_t)SM_EVENT_EXCEPTION);
      return;
    }
  motion_speed_notify(vx, vz);
}

static void charger_normal_charging_event_in_forcecharging(void)
{
  float   voltage = 0;
  float   current = 0;
  bool    is_charging;
  int32_t ret;
  int32_t voltage_gap = charger_dev_p->full_battery_voltage - charger_dev_p->low_battery_voltage;

  ret = charger_dev_get_all_stats(&voltage, &current, NULL, &is_charging);
  if (ret != OK)
    {
      charger_exception_notify(CHARGER_ENTER_EXCEPTION);
      if (charger_dev_p->sm.sm_task_started)
        charger_dev_sm_signal_send((int32_t)SM_EVENT_EXCEPTION);
      return;
    }
  if (is_charging == FALSE)
    {
      if (charger_dev_p->sm.sm_task_started)
        charger_dev_sm_signal_send((int32_t)SM_EVENT_TO_NORMAL_CHARGING);
      syslog(LOG_INFO, "CHARGER: force charging: charging disconnect, jump to charging state!!!!\n");
      return;
    }
  syslog(LOG_DEBUG, "CHARGER: force charging state: is_charging = (%d) !\n", is_charging);

  charging_current_notify(current);
  battery_voltage_notify(voltage);
  if (voltage >= (BATTERY_FORCE_VOLT_PERCETAGE * voltage_gap + charger_dev_p->low_battery_voltage))
    {
      if (charger_dev_p->sm.sm_task_started)
        charger_dev_sm_signal_send((int32_t)SM_EVENT_TO_NORMAL_CHARGING);
    }
}

static void charger_stop_charging_event_in_charging(void)
{
  float   voltage = 0;
  float   current = 0;
  int32_t ret;
  bool    is_charging;
  ret = charger_dev_get_all_stats(&voltage, &current, NULL, &is_charging);
  if (ret != OK)
    {
      charger_exception_notify(CHARGER_ENTER_EXCEPTION);
      if (charger_dev_p->sm.sm_task_started)
        charger_dev_sm_signal_send((int32_t)SM_EVENT_EXCEPTION);
      return;
    }
  if (is_charging == FALSE)
    {
      if (charger_dev_p->sm.sm_task_started)
        charger_dev_sm_signal_send((int32_t)SM_EVENT_BACK_TO_IDLE);
      syslog(LOG_INFO, "CHARGER: charging state: charging disconnect, back to idle!!!!\n");
      return;
    }
  syslog(LOG_DEBUG, "CHARGER: charging state: is_charging = (%d) !\n", is_charging);

  charging_current_notify(current);
  battery_voltage_notify(voltage);

  if (voltage >= charger_dev_p->full_battery_voltage)
    {
      if (charger_dev_p->sm.sm_task_started)
        charger_dev_sm_signal_send((int32_t)SM_EVENT_STOP_CHARGING);
    }
}

static void charger_back_to_idle_event_charging_done(void)
{
  bool    is_charging;
  int32_t ret;
  while (1)
    {
      ret = charger_dev_get_is_charging_stats(&is_charging);
      if (ret != OK)
        {
          charger_exception_notify(CHARGER_IS_CHARGING_ERROR);
          if (charger_dev_p->sm.sm_task_started)
            charger_dev_sm_signal_send((int32_t)SM_EVENT_EXCEPTION);
          return;
        }
      if (is_charging == TRUE)
        {
          syslog(LOG_INFO, "CHARGER: charging done state:  *** move *** !!!!\n");
          motion_speed_notify(CHARGER_SM_STOP_VX_SPEED, CHARGER_SM_STOP_VZ_ZERO);
          usleep(50000);
        }
      else
        {
          syslog(LOG_INFO, "CHARGER: charging done state:  *** stop *** !!!!\n");
          motion_speed_notify(CHARGER_SM_STOP_VX_ZERO, CHARGER_SM_STOP_VZ_ZERO);
          if (charger_dev_p->sm.sm_task_started)
            charger_dev_sm_signal_send((int32_t)SM_EVENT_BACK_TO_IDLE);
          return;
        }
    }
}

static int32_t charger_dev_get_speed(float *vx, float *vz)
{
  return get_speed_hal(vx, vz);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int charger_dev_sm_signal_send(int32_t sigval_int)
{
  union sigval sigev_value;
  sigev_value.sival_int = sigval_int;
#ifdef CHARGER_DEBUG
  syslog(LOG_DEBUG, "CHARGER: charger_dev_sm_signal_send: send signal with (%s)!\n", sm_event_labels[sigev_value.sival_int]);
#endif
  return amr_signal_send(&charger_dev_p->charger_signal, sigev_value);
}

int charger_dev_sm_signal_wait(struct siginfo *info, FAR const struct timespec *timeout)
{
  return amr_signal_wait(&charger_dev_p->charger_signal, info, timeout);
}

void charger_dev_sm_signal_init(void)
{

  if (amr_signal_init(&charger_dev_p->charger_signal, nxsched_getpid(), CONFIG_CHARGER_SM_SIGNO) != OK)
    {
      syslog(LOG_ERR, "CHARGER: charger_core_sm_signal_init: sm signal init failed!\n");
      ASSERT(false);
    }
  return;
}

void charger_dev_rwlock_init(pthread_rwlock_t *cc_rw_lock)
{

  int status;
  status = pthread_rwlock_init(cc_rw_lock, NULL);
  if (status != 0)
    {
      syslog(LOG_ERR, "CHARGER: sm_init: ERROR pthread_rwlock_init failed, status=%d\n", status);
      ASSERT(false);
    }
}

void charger_dev_rwlock_rdlock_acquire(FAR pthread_rwlock_t *cc_rw_lock)
{
  int status;
  status = pthread_rwlock_rdlock(cc_rw_lock);
  if (status != 0)
    {
      syslog(LOG_ERR, "CHARGER: sm_rwlock_rdlock_acquire: ERROR Failed to open rwlock for reading. Status: %d\n", status);
      ASSERT(false);
    }
}

void charger_dev_rwlock_wrlock_acquire(FAR pthread_rwlock_t *cc_rw_lock)
{
  int status;
  status = pthread_rwlock_wrlock(cc_rw_lock);
  if (status != 0)
    {
      syslog(LOG_ERR, "CHARGER: sm_rwlock_wrlock_acquire: ERROR Failed to lock for writing\n");
      ASSERT(false);
    }
}

void charger_dev_rwlock_release(FAR pthread_rwlock_t *rw_lock)
{
  int status;
  status = pthread_rwlock_unlock(rw_lock);
  if (status != 0)
    {
      syslog(LOG_ERR, "CHARGER: sm_rwlock_release: ERROR Failed to unlock lock held for writing\n");
      ASSERT(false);
    }
}

void charger_dev_mutex_init(pthread_mutex_t *cc_mutex)
{
  int status;
  status = pthread_mutex_init(cc_mutex, NULL);
  if (status != 0)
    {
      syslog(LOG_ERR, "CHARGER: chr_core_mutex_init: ERROR pthread_mutex_init failed, status=%d\n", status);
      ASSERT(false);
    }
}

void charger_dev_mutex_lock(pthread_mutex_t *cc_mutex)
{
  int status;
  status = pthread_mutex_lock(cc_mutex);
  if (status != 0)
    {
      syslog(LOG_ERR, "CHARGER: chr_core_mutex_lock: ERROR pthread_mutex_lock failed, status=%d\n", status);
      ASSERT(false);
    }
}

void charger_dev_mutex_unlock(pthread_mutex_t *cc_mutex)
{
  int status;
  status = pthread_mutex_unlock(cc_mutex);
  if (status != 0)
    {
      syslog(LOG_ERR, "CHARGER: chr_core_mutex_unlock: ERROR pthread_mutex_unlock failed, status=%d\n", status);
      ASSERT(false);
    }
}

/* get battery voltage */
int32_t charger_dev_get_voltage(float *voltage)
{
  return get_voltage_hal(voltage);
}

/* get charging current */
int32_t charger_dev_get_charging_current(float *current)
{
  return get_charging_current_hal(current);
}

/* search charger pile to see if it is nearby charger pile */
int32_t charger_dev_get_pile_stats(bool *pile_stats)
{
  return get_pile_signal_stats_hal(pile_stats);
}

/* check if it is in charging status */
int32_t charger_dev_get_is_charging_stats(bool *charging_stat)
{
  return get_charger_is_charging_hal(charging_stat);
}

/* get battery voltage, charging current, charger pile and charging status */
int32_t charger_dev_get_all_stats(float *voltage, float *current, bool *pile_stat, bool *charging_stat)
{
  return get_all_stats_hal(voltage, current, pile_stat, charging_stat);
}

/* get charger state machine status */
uint32_t charger_dev_get_sm_stats(void)
{
  return get_curr_state();
}

/* get full battery voltage */
int32_t charger_dev_set_full_batt_volt(float voltage)
{
  if (charger_dev_p->initialized)
    {
      charger_dev_p->full_battery_voltage = voltage;
      syslog(LOG_INFO, "CHARGER: set full battery voltage to (%fV) !\n", voltage);
      return OK;
    }
  return ERROR;
}

/* get low battery voltage */
int32_t charger_dev_set_low_batt_volt(float voltage)
{
  if (charger_dev_p->initialized)
    {
      charger_dev_p->low_battery_voltage = voltage;
      syslog(LOG_INFO, "CHARGER: set low battery voltage to (%fV) !\n", voltage);
      return OK;
    }
  return ERROR;
}

int32_t charger_dev_set_battery_cap(float cap)
{
  if (charger_dev_p->initialized)
    {
      charger_dev_p->battery_capacity = cap;
      syslog(LOG_INFO, "CHARGER: set battery capacity to (%fV) !\n", cap);
      return OK;
    }
  return ERROR;
}

/* register notification callback  */
int32_t notification_callback_register(notification_cb notify_fun_cb)
{
  if (charger_dev_p->initialized)
    {
      charger_dev_p->notify_callback = notify_fun_cb;
      syslog(LOG_INFO, "CHARGER: notification_callback_register done(%p)!!!!!!\n", charger_dev_p->notify_callback);
      return OK;
    }
  syslog(LOG_ERR, "CHARGER: no notification_callback_registered!\n");
  return ERROR;
}

/* register motion speed callback  */
int32_t motion_callback_register(motion_speed_cb motion_fun_cb)
{
  if (charger_dev_p->initialized)
    {
      charger_dev_p->motion_callback = motion_fun_cb;
      syslog(LOG_INFO, "CHARGER: motion_callback_register done(%p)!!!!!!\n", charger_dev_p->motion_callback);
      return OK;
    }
  syslog(LOG_ERR, "CHARGER: no motion_callback_registered!\n");
  return ERROR;
}

void charger_exception_notify(enum charger_exception_type_e exception_err)
{
  struct charger_msg_s exception_msg;
  exception_msg.msg_type           = KEY_EXCEPTION;
  exception_msg.data.exception_err = ERROR_CHARGER;

  if (charger_dev_p->notify_callback)
    {
      charger_dev_p->notify_callback(&exception_msg);
      syslog(LOG_DEBUG, "CHARGER: charger_exception_notify: exception = (%s) !\n", exception_labels[exception_err]);
    }
  else
    {
      syslog(LOG_DEBUG, "CHARGER: charger_exception_notify: no callback registered! exception = (%s) !\n", exception_labels[exception_err]);
    }
}

void battery_voltage_notify(float voltage)
{
  struct charger_msg_s voltage_msg;
  voltage_msg.msg_type     = KEY_VOLTAGE;
  voltage_msg.data.voltage = voltage;
  if (charger_dev_p->notify_callback)
    {
      charger_dev_p->notify_callback(&voltage_msg);
    }
  else
    {
      syslog(LOG_DEBUG, "CHARGER: battery_voltage_notify: no callback registered! battery voltage = (%fV) !\n", voltage);
    }
}

void voltage_change_notify(void)
{
  float   curr_volt = 0;
  int32_t ret;
  ret = charger_dev_get_voltage(&curr_volt);
  if (ret != OK)
    {
      charger_exception_notify(CHARGER_VOLTAGE_ERROR);
      charger_dev_sm_signal_send((int32_t)SM_EVENT_EXCEPTION);
      return;
    }
#ifdef CHARGER_DEBUG
  syslog(LOG_DEBUG, "CHARGER: voltage_change_notify: voltage diff = (%fV)\n", charger_dev_p->battery_voltage - curr_volt);
#endif
  if (curr_volt != 0 && ((charger_dev_p->battery_voltage - curr_volt) > BATTERY_VOLTAGE_NOTIFY_THRES))
    {
      battery_voltage_notify(curr_volt);
      charger_dev_p->battery_voltage = curr_volt;
    }
}

void charging_current_notify(float current)
{
  struct charger_msg_s current_msg;
  current_msg.msg_type     = KEY_CURRENT;
  current_msg.data.current = current;
  if (charger_dev_p->notify_callback)
    {
      charger_dev_p->notify_callback(&current_msg);
    }
  else
    {
      syslog(LOG_DEBUG, "CHARGER: charging_current_notify: no callback registered! charging current = (%fA) !\n", current);
    }
}

void sm_state_notify(enum chr_sm_st_e state)
{
  struct charger_msg_s sm_state_msg;
  sm_state_msg.msg_type      = KEY_SM_STATE;
  sm_state_msg.data.sm_state = state;
  if (charger_dev_p->notify_callback)
    {
      charger_dev_p->notify_callback(&sm_state_msg);
    }
  else
    {
      syslog(LOG_DEBUG, "CHARGER: sm_state_notify: no callback registered! sm state = (%s)!\n", sm_state_labels[state]);
    }
}

void motion_speed_notify(float vx, float vz)
{
  if (charger_dev_p->motion_callback)
    {
      charger_dev_p->motion_callback(vx, vz);
    }
  else
    {
      syslog(LOG_DEBUG, "CHARGER: motion_speed_notify: no callback registered! vx = %fm/s, vz = %frad/s\n)!\n", vx, vz);
    }
}

#ifdef CHARGER_DEBUG
void driver_polling_print_debug(void)
{
  int32_t ret;
  float   curr_current;
  float   curr_voltage;
  bool    pile_stat;
  bool    is_charging_stat;
  float   vx;
  float   vz;

#  ifdef TEST
  /*polling charging current*/
  ret = charger_dev_get_charging_current(&curr_current);
  if (ret != OK)
    {
      charger_exception_notify(CHARGER_CHARGING_CURRENT_ERROR);
      return;
    }
  syslog(LOG_DEBUG, "CHARGER: driver_polling_print_debug: current = (%fA) !\n", curr_current);

  /*polling pile_stats*/
  ret = charger_dev_get_pile_stats(&pile_stat);
  if (ret != OK)
    {
      charger_exception_notify(CHARGER_PILE_STAT_ERROR);
      return;
    }
  syslog(LOG_DEBUG, "CHARGER: driver_polling_print_debug: pile_stat = (%d) !\n", pile_stat);

  /*polling is_charging_stats*/
  ret = charger_dev_get_is_charging_stats(&is_charging_stat);
  if (ret != OK)
    {
      charger_exception_notify(CHARGER_IS_CHARGING_ERROR);
      return;
    }
  syslog(LOG_DEBUG, "CHARGER: driver_polling_print_debug: is_charging_stat = (%d) !\n", is_charging_stat);
#  endif

  /*polling speed*/
  ret = charger_dev_get_speed(&vx, &vz);
  if (ret != OK)
    {
      charger_exception_notify(CHARGER_GET_SPEED_ERROR);
      return;
    }
  syslog(LOG_DEBUG, "***********************************************************************\n");
  syslog(LOG_DEBUG, "\n");
  syslog(LOG_DEBUG, "CHARGER: start_polling_voltage: speed vx = %fm/s, vz = %frad/s\n)!\n", vx, vz);

  /*polling all*/
  ret = charger_dev_get_all_stats(&curr_voltage, &curr_current, &pile_stat, &is_charging_stat);
  if (ret != OK)
    {
      charger_exception_notify(CHARGER_ENTER_EXCEPTION);
      return;
    }
  syslog(LOG_DEBUG, "CHARGER: start_polling_voltage: voltage = %fV, current = %fA, pile_stat = %d, is_charging_stat = %d\n",
         curr_voltage, curr_current, pile_stat, is_charging_stat);
  syslog(LOG_DEBUG, "\n");
  syslog(LOG_DEBUG, "***********************************************************************\n");
}
#endif

static void charger_dev_exception(void)
{
  charger_exception_notify(CHARGER_ENTER_EXCEPTION);
  syslog(LOG_DEBUG, "CHARGER: charger_dev_exception: charger enter exception !!!!\n");
}

uint32_t get_polling_interval(void)
{
  uint32_t          interval;
  pthread_rwlock_t *rw_lock = &charger_dev_p->polling_rw_lock;
  charger_dev_rwlock_rdlock_acquire(rw_lock);
  interval = charger_dev_p->polling_interval;
  charger_dev_rwlock_release(rw_lock);
  return interval;
}

void set_polling_interval(uint32_t interval)
{
  pthread_rwlock_t *rw_lock = &charger_dev_p->polling_rw_lock;
  charger_dev_rwlock_wrlock_acquire(rw_lock);
  charger_dev_p->polling_interval = interval;
  charger_dev_rwlock_release(rw_lock);
  return;
}

static void start_polling(void)
{
  clock_t start          = 0;
  clock_t elapsed        = 0;
  int     saved_sm_state = 0;
  int     curr_sm_state  = 0;
  int32_t ret;

  syslog(LOG_INFO, "CHARGER: start_polling: start polling...\n");

  ret = charger_dev_get_voltage(&charger_dev_p->battery_voltage);
  if (ret != OK)
    {
      charger_exception_notify(CHARGER_VOLTAGE_ERROR);
      if (charger_dev_p->sm.sm_task_started)
        charger_dev_sm_signal_send((int32_t)SM_EVENT_EXCEPTION);
    }

  while (1)
    {
      curr_sm_state = get_curr_state();
      if (saved_sm_state != curr_sm_state)
        {
          syslog(LOG_INFO, "CHARGER: sm state change from (%s) to (%s)\n", sm_state_labels[saved_sm_state], sm_state_labels[curr_sm_state]);
          saved_sm_state = curr_sm_state;
        }
      switch (curr_sm_state)
        {

          case CHR_SM_IDLE:
            voltage_change_notify();
#ifdef CHARGER_DEBUG
            driver_polling_print_debug();
#endif
            start = 0;
            break;

          case CHR_SM_SEARCHING:
            charger_find_pile_event_in_searching();
            break;

          case CHR_SM_CONTROLLING:
            if (start == 0)
              {
                start = clock_systime_ticks();
              }

            charger_pile_attached_event_in_controlling();

            elapsed = clock_systime_ticks() - start;
            if (elapsed >= CHR_CTL_MODE_TIMEOUT)
              {
                if (charger_dev_p->sm.sm_task_started)
                  charger_dev_sm_signal_send((int32_t)SM_EVENT_BACK_TO_IDLE);
              }
            break;

          case CHR_SM_FORCE_CHARGING:
            charger_normal_charging_event_in_forcecharging();
            break;

          case CHR_SM_CHARGING:
            charger_stop_charging_event_in_charging();
            break;

          case CHR_SM_CHARGER_DONE:
            charger_back_to_idle_event_charging_done();
            break;

          case CHR_SM_EXCEPTION:
            charger_dev_exception();
            break;

          default:
            break;
        }
      usleep(get_polling_interval());
    }
}

int charger_management(int argc, char *argv[])
{
  int32_t ret;
  memset(charger_dev_p, 0, sizeof(struct charger_device_s));

  strlcpy(charger_dev_p->name, CHARGER_DEVICE_NAME, CHARGER_DEVICE_NAME_SIZE);

  charger_dev_p->notify_callback      = NULL;
  charger_dev_p->motion_callback      = NULL;
  charger_dev_p->polling_interval     = IDLE_BATT_VOLT_NOTIFY;
  charger_dev_p->sm.sm_task_started   = FALSE;
  charger_dev_p->full_battery_voltage = BATTERY_FULL_VOLT;
  charger_dev_p->low_battery_voltage  = BATTERY_LOW_VOLT;
  charger_dev_rwlock_init(&charger_dev_p->polling_rw_lock);

  ret = charger_dirver_init_hal(charger_dev_p->name);
  if (ret != OK)
    {
      syslog(LOG_ERR, "CHARGER: charger_management_main_task:(%s) driver init failed!\n", charger_dev_p->name);
      config_notify_completed(false);
      return ERROR;
    }

  ret = start_sm_task();
  if (ret < 0)
    {
      syslog(LOG_ERR, "CHARGER: charger_management_main_task:state machine start failed!\n");
      config_notify_completed(false);
      return ERROR;
    }
  usleep(50 * 1000L);
  charger_dev_p->initialized = TRUE;
  syslog(LOG_INFO, "CHARGER: charger_management_main_task: Charger device Initialize successfully!\n");
  config_notify_completed(true);
  start_polling();
  return OK;
}
