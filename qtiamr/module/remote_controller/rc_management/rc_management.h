/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#ifndef __APP_QTIAMR_RC_MANAGE_H
#define __APP_QTIAMR_RC_MANAGE_H

extern int regitster_hotrc_hal(struct remote_contrl_platform_s *hal_control);

struct remote_contrl_ops_s
{
  int *init_fucnt(struct remote_contrl_platform_s *hal_control);
  int *get_speed(struct speed_req_s *speed);
  int set_max_speed();
}

struct speed_req_s
{
  double x_speed;
  double z_speed;
}
 

Struct remote_contrl_platform_s
{
  char hw_name[8];
  char enable_state;
  struct max_req_s speed_limit;
  struct remote_contrl_ops_s ops;
}

int rc_management_task(int argc, char *argv[]);
int register_hal_ops();



#endif
