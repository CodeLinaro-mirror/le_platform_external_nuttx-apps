/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
//#include <nuttx/config.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <sched.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <syslog.h>

#include "motion_management.h"
#include "motion_sm.h"
#include "rc_management.h"
#include "remote_controller.h"
#include "config_msg.h"
#include "main.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

//#define TEST_RC_ONLY	/*for debug*/

#define MAX_SPEED		(2.0f)	/* actual is 1.82 m/s */
#define MAX_ANGULAR_VELOCITY	2

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rc_controller_data_s
{
  struct rc_parameter_s init_setting;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int rc_mgr_state_handler(enum rc_action_e action, void *data);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct rc_controller_data_s g_rc_ctrl =
{
  .init_setting = {2,2,true},
};

struct rc_management_cb_s rc_mgr_cb =
{
  .rc_client_callback = rc_mgr_state_handler,
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int rc_mgr_state_handler(enum rc_action_e action, void *data)
{
  int ret = 0;
  struct speed_req_s *speed_req;
  switch (action)
  {
    case RC_UPDATE_SPEED:
      speed_req = (struct speed_req_s*) data;
#ifndef TEST_RC_ONLY
      /*update speed*/
      syslog(LOG_DEBUG,"rc send speed_req: vx: %f vz: %f \n",
	    speed_req->x_speed,speed_req->z_speed);
      motion_speed_control(REMOTE_CONTROLLER, speed_req->x_speed
		      , speed_req->z_speed);
#else
      syslog(LOG_INFO,"send speed_req: vx: %f vz: %f \n",
	    speed_req->x_speed,speed_req->z_speed);
#endif
      break;
    default:
      return -1;
  }
#ifdef TEST_RC_ONLY
  syslog(LOG_INFO,"rc_mgr_handler done. atcion: %d .\n",
	    action);
#endif
  return ret;
}

#ifndef TEST_RC_ONLY
//void (*sm_notify_cb)( enum mcb_sm_e state);

static void MCB_status_callback(enum control_sm_state_e state)
{
  int ret = 0;
  int target_state = 0;
  static int curr_state = -1;

  if (state != ST_REMOTE_CONTROLLING)
  {
    target_state = 1;
  }
  else
  {
    target_state = 0;
  }

  if (curr_state == target_state)
  {
    return;
  }

  if (target_state == 1)
  {
    /*enable rc data update*/
    syslog(LOG_DEBUG,"rc enable speed update, MCB status =%d.\n",state);
    ret = rc_manag_enable();
    if (ret != OK)
    {
      syslog(LOG_DEBUG,"rc enable speed update fail.\n");
      curr_state = -1;
    }
  }
  else
  {
    /*disable rc data update*/
    syslog(LOG_DEBUG,"rc disable speed update, MCB status =%d.\n",state);
    ret = rc_manag_disable();
    if (ret != OK)
    {
      syslog(LOG_DEBUG,"rc disable speed update fail.\n");
      curr_state = -1;
    }
    curr_state = target_state;
  }

  return;
}
#endif

static int read_the_RC_init_setting(void)
{
  int status = OK;
  struct config_remote_controller_s params;

  status = get_configuration_parameters(RC, &params);
  if (status == OK)
  {
    g_rc_ctrl.init_setting.x_speed = params.max_speed;
    g_rc_ctrl.init_setting.z_speed = params.max_angle_speed;
    g_rc_ctrl.init_setting.enable_rc_management = params.rc_enable;
  }
  return status;
}



/****************************************************************************
 * Public Functions
 ****************************************************************************/

// struct remote_contrl_platform remote_controller_manage_platform on HAL define

int rc_controller_task(int argc, char *argv[])
{
  int ret = 0;

#ifndef TEST_RC_ONLY
  /*read basic setting*/
  ret = read_the_RC_init_setting();
  if (ret != OK)
  {
    syslog(LOG_INFO,"%s read rc params fail.\n",__func__);
    goto err;
  }
#endif
  /*set the init setting into RC manager*/
  set_rc_manage_init_setting(g_rc_ctrl.init_setting);

  if (g_rc_ctrl.init_setting.enable_rc_management == false)
  {
    /*let rc_mgr_thread exit*/
    syslog(LOG_INFO,"%s disable rc function.\n",__func__);
    rc_manag_enable();
    goto end;
  }

#ifdef TEST_RC_ONLY
  rc_manag_enable();
#endif
  ret = register_rc_mgr_callback(&rc_mgr_cb);
#ifndef TEST_RC_ONLY
  ret = client_control_sm_register_notify_cb(MCB_status_callback);
#endif
end:
  syslog(LOG_INFO,"%s thread done.\n",__func__);
  config_notify_completed(true);
  return ret;
err:
  syslog(LOG_INFO,"%s thread error.\n",__func__);
  config_notify_completed(false);

  return ret;
}
