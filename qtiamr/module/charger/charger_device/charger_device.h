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
#include <stdint.h>
#include <nuttx/signal.h>
#include "charger_sm.h"
#include "amr_signal.h"
#include "charger_management.h"
#include "main.h"

/********************************************************************************
 * Pre-processor Definitions
 ********************************************************************************/
#define CHARGER_DEVICE_NAME_SIZE 20

#define CHARGER_SM_PRIORITY  DEFAULT_PRIORITY
#define CHARGER_SM_STACKSIZE DEFAULT_STACK_SIZE

/*polling time definition*/
#define CHARGER_EXCEPTION_DELAY  10000000 //10s
#define IDLE_BATT_VOLT_NOTIFY    5000000  //5s
#define CHARGING_DONE_DELAY      2000000  //2s
#define CONTROLLING_SPEED_NOTIFY 20000    //20ms
#define CHARGING_POLL_DELAY      2000000  //2s
#define CHR_CTL_MODE_TIMEOUT     SEC2TICK(200)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/*charger dev main struct*/
struct charger_device_s
{
  struct amr_signal_s       charger_signal;
  bool                      initialized;
  char                      name[CHARGER_DEVICE_NAME_SIZE];
  struct charger_sm_state_s sm;
  float                     full_battery_voltage;
  float                     low_battery_voltage;
  float                     battery_capacity;
  float                     battery_voltage;
  pthread_rwlock_t          polling_rw_lock;
  uint32_t                  polling_interval;
  notification_cb           notify_callback;
  motion_speed_cb           motion_callback;

} __attribute__((aligned(4)));

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int      charger_dev_sm_signal_send(int32_t sigval_int);
int      charger_dev_sm_signal_wait(struct siginfo *info, FAR const struct timespec *timeout);
void     charger_dev_sm_signal_init(void);
void     set_polling_interval(uint32_t interval);
uint32_t get_polling_interval(void);
void     charger_dev_rwlock_init(pthread_rwlock_t *cc_rw_lock);
void     charger_dev_rwlock_rdlock_acquire(FAR pthread_rwlock_t *cc_rw_lock);
void     charger_dev_rwlock_wrlock_acquire(FAR pthread_rwlock_t *cc_rw_lock);
void     charger_dev_rwlock_release(FAR pthread_rwlock_t *rw_lock);
void     charger_exception_notify(enum charger_exception_type_e exception_err);
void     sm_state_notify(enum chr_sm_st_e state);
void     battery_voltage_notify(float voltage);
void     voltage_change_notify(void);
void     charging_current_notify(float current);
void     motion_speed_notify(float vx, float vz);

#endif /* __APP_CHARGER_DEVICE_H */
