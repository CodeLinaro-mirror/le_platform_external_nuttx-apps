/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#ifndef __APP_CHARGER_HAL_H
#define __APP_CHARGER_HAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <stdint.h>


/***************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct charger_drv_ops_s {
  int32_t (*get_voltage)(float *volt);
  int32_t (*get_current)(float *curr);
  int32_t (*get_speed)(float *vx, float *vz);
  int32_t (*set_def_speed)(float vx, float vz);
  int32_t (*get_pile_signal_stats)(bool *stats);
  int32_t (*get_is_charging_stats)(bool *stats);
  int32_t (*get_all_stats)(float *voltage, float *current, bool *infrared_stat, bool *is_charging);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int32_t get_voltage_hal(float *voltage);
int32_t get_charging_current_hal(float *current);
int32_t get_speed_hal(float *vx, float *vz);
int32_t get_pile_signal_stats_hal(bool *stats);
int32_t get_charger_is_charging_hal(bool *stats);
int32_t get_all_stats_hal(float *voltage, float *current, bool *infrared_stat, bool *is_charging);
int32_t charger_dev_ops_cb_register(struct charger_drv_ops_s *ops);
int32_t charger_dirver_init_hal(const char *name);

#endif /* __APP_CHARGER_HAL_H */
