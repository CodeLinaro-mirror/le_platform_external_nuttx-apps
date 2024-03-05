/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#include <stdint.h>
#include <syslog.h>

#include "ec130.h"
#include "voltage_adc.h"
#include "charger_hal.h"


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct chr_dev_table_s{
  char *name;
  int32_t (*init_func)(void);
}__attribute__((aligned(4)));

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct charger_drv_ops_s *g_drv_ops = NULL;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int32_t charger_dev_ops_cb_register(struct charger_drv_ops_s *ops){

  if(ops == NULL)
   {
    syslog(LOG_ERR, "CHARGER: charger_dev_ops_cb_register: ERROR Failed to register charger device ops callback function\n");
    return ERROR;
   }
  g_drv_ops = ops;
  return OK;
}

int32_t get_voltage_hal(float *voltage)
{
  if ((g_drv_ops != NULL) && (g_drv_ops->get_voltage))
    return g_drv_ops->get_voltage(voltage);
  return ERROR;
}

int32_t get_charging_current_hal(float *current)
{
  if ((g_drv_ops != NULL) && (g_drv_ops->get_current))
    return g_drv_ops->get_current(current);
  return ERROR;
}

int32_t get_speed_hal(float *vx, float *vz)
{
  if ((g_drv_ops != NULL) && (g_drv_ops->get_speed))
    return g_drv_ops->get_speed(vx,vz);
  return ERROR;
}

int32_t get_pile_signal_stats_hal(bool *stats)
{
  if ((g_drv_ops != NULL) && (g_drv_ops->get_pile_signal_stats))
    return g_drv_ops->get_pile_signal_stats(stats);
  return ERROR;
}

int32_t get_charger_is_charging_hal(bool *stats)
{
  if ((g_drv_ops != NULL) && (g_drv_ops->get_is_charging_stats))
    return g_drv_ops->get_is_charging_stats(stats);
  return ERROR;
}

int32_t get_all_stats_hal(float *voltage, float *current, bool *infrared_stat, bool *is_charging)
{
  if ((g_drv_ops != NULL) && (g_drv_ops->get_is_charging_stats))
    return g_drv_ops->get_all_stats(voltage, current, infrared_stat, is_charging);
  return ERROR;
}

struct chr_dev_table_s charger_drv_table[] =
{
    {"ec130", ec130_and_adc_driver_init},
    // add more driver here
};

int32_t charger_dirver_init_hal(const char *name)
{

  int drv_num;
  int i;
  int32_t (*init_func)(void);
  drv_num = sizeof(charger_drv_table) / sizeof(struct chr_dev_table_s);
  init_func = NULL;
  for (i = 0; i < drv_num; i++)
  {
    if (!strcmp(name, charger_drv_table[i].name))
      {
        init_func = charger_drv_table[i].init_func;
      }
  }

  if(init_func)
    {
      syslog(LOG_DEBUG, "CHARGER: charger_dirver_init_hal:(%s) driver found\n", name);
      if (init_func() != OK)
      {
        syslog(LOG_ERR, "CHARGER: charger_dirver_init_hal:(%s) driver init failed\n", name);
        return ERROR;
      }
      syslog(LOG_DEBUG, "CHARGER: charger_dirver_init_hal:(%s) driver init successfully\n", name);
    }
  else
    {
      syslog(LOG_ERR, "CHARGER: charger_dirver_init_hal:(%s) driver not found\n", name);
      return ERROR;
    }
  return OK;
}
