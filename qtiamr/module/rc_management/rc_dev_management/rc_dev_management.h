/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#ifndef __APP_QTIAMR_RC_DEV_MANAGE_H
#define __APP_QTIAMR_RC_DEV_MANAGE_H

#include "rc_management.h"

struct rc_hal_ops_s
{
  int (*init)(void);
  int (*release)(void);
  int (*get_vx_vz_speed)(struct speed_req_s *speed);
  int (*set_max_speed)(float x_speed, float z_speed);
};

int rc_dev_manag_hal_init(void);
int get_vx_vz_speed_from_hal(struct speed_req_s *speed);
int rc_dev_manage_release(void);
int set_rc_hal_max_speed(float x_speed, float z_speed);

#endif
