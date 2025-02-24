/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#ifndef __APP_QTIAMR_RC_MANAGE_H
#define __APP_QTIAMR_RC_MANAGE_H

enum rc_action_e
{
  RC_UPDATE_SPEED = 0,
  RC_MAX_INDEX,
};

struct rc_management_cb_s
{
  struct rc_management_cb_s *list_nlink;
  struct rc_management_cb_s *list_prelink;
  int (*rc_client_callback)(enum rc_action_e action, void *data);
};

struct rc_parameter_s
{
  float x_speed;
  float z_speed;
  bool  enable_rc_management;
};

struct speed_req_s
{
  float x_speed;
  float z_speed;
};

int rc_management_task(int argc, char *argv[]);
int register_rc_mgr_callback(struct rc_management_cb_s *cb);
int set_rc_mgr_max_speed(struct speed_req_s speed);
int set_rc_manage_init_setting(struct rc_parameter_s data);
int rc_manag_enable(void);
int rc_manag_disable(void);

#endif
