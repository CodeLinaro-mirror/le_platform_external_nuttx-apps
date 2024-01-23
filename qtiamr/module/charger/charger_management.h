/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#ifndef __APP_CHARGER_MANAGEMENT_H
#define __APP_CHARGER_MANAGEMENT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "charger_sm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum charger_msg_type_e
{
  KEY_VOLTAGE = 10,
  KEY_CURRENT,
  KEY_SM_STATE,
  KEY_EXCEPTION
};

enum charger_exception_type_e
{
  CHARGER_VOLTAGE_ERROR = 1,
  CHARGER_CHARGING_CURRENT_ERROR,
  CHARGER_PILE_STAT_ERROR,
  CHARGER_IS_CHARGING_ERROR,
  CHARGER_GET_SPEED_ERROR,
  CHARGER_SM_STATE_ERROR,
  CHARGER_DEV_NOT_READY_ERROR,
  CHARGER_ENTER_EXCEPTION
};

struct charger_msg_s
{
  enum charger_msg_type_e msg_type;
  union
    {
      float voltage;
      float current;
      enum charger_exception_type_e exception_err;
      enum chr_sm_st_e sm_state;
    }data;
}__attribute__((aligned(4)));

typedef void (*notification_cb)(struct charger_msg_s *msg);
typedef void (*motion_speed_cb)(float vx, float vz);


/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void sm_start_charging(void);
void sm_stop_charging(void);
int32_t charger_dev_get_voltage(float *voltage);
int32_t charger_dev_get_charging_current(float *current);
int32_t charger_dev_get_pile_stats(bool *pile_stats);
int32_t charger_dev_get_is_charging_stats(bool *charging_stat);
int32_t charger_dev_get_all_stats(float *voltage, float *current, bool *pile_stat, bool *charging_stat);
uint32_t charger_dev_get_sm_stats(void);
int32_t charger_dev_set_full_batt_volt(float voltage);
int32_t charger_dev_set_low_batt_volt(float voltage);
int32_t charger_dev_set_battery_cap(float cap);
int32_t notification_callback_register(notification_cb notify_fun_cb);
int32_t motion_callback_register(motion_speed_cb motion_fun_cb);
int charger_management(int argc, char *argv[]);

#endif /* __APP_CHARGER_MANAGEMENT_H */
