/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <sched.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <syslog.h>
#include <errno.h>

#include "rc_dev_management.h"
#include "hotrc.h"

static struct rc_hal_ops_s *g_rc_dev;

/*support rc hal*/
static struct rc_hal_ops_s *const hal_ops_g[1] = {
  [0] = &zl_ops,
};

int rc_dev_manag_hal_init(void)
{
  int status = OK;

  if ((hal_ops_g[0] == NULL) || (hal_ops_g[0]->get_vx_vz_speed == NULL))
    {
      syslog(LOG_INFO, "%s : set hal_ops is null", __func__);
      return ERROR;
    }
  g_rc_dev = hal_ops_g[0];

  if (g_rc_dev->init != NULL)
    {
      status = g_rc_dev->init();
    }

  return status;
}

int get_vx_vz_speed_from_hal(struct speed_req_s *speed)
{
  int status = 0;

  if (g_rc_dev == NULL)
    {
      return ERROR;
    }
  status = g_rc_dev->get_vx_vz_speed(speed);
  if (status != OK)
    {
      syslog(LOG_INFO, "%s : get speed fail", __func__);
      return ERROR;
    }

  return status;
}

int rc_dev_manage_release(void)
{
  int status = OK;

  if (g_rc_dev == NULL)
    {
      return ERROR;
    }
  if (g_rc_dev->release != NULL)
    {
      status = g_rc_dev->release();
      if (status != OK)
        {
          syslog(LOG_INFO, "%s: release hal fail, thread exit.\n", __func__);
        }
    }

  return status;
}

int set_rc_hal_max_speed(float x_speed, float z_speed)
{
  int status = OK;

  if (g_rc_dev == NULL)
    {
      /*do nothing*/
      return OK;
    }
  if (g_rc_dev->set_max_speed != NULL)
    {
      status = g_rc_dev->set_max_speed(x_speed, z_speed);
    }
  return status;
}
