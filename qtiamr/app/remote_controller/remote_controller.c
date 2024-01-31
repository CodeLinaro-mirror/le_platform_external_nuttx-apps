/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
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
#include "rc_dev_management.h"
#include "config_msg.h"
#include "main.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

//#define SKIP_PARAM_INIT /*for debug*/
//#define TEST_RC_ONLY	/*for debug*/

#define MAX_SPEED		(0.8f)	/* actual is 1.82 m/s */
#define MAX_ANGULAR_VELOCITY	2

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rc_controller_data_s
{
  struct rc_parameter_s init_setting;
};

enum rc_control_status_s
{
  U_INIT = 0,
  RC_UPDATE,
  RC_STOP,
};

enum rc_enable_state_s
{
  DISABLE = 0,
  ENABLE,
  MAX_INDEX,
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
  .init_setting = {0.8,2,true},
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
  enum motion_result_e motion_result;


  switch (action)
  {
    case RC_UPDATE_SPEED:
      speed_req = (struct speed_req_s*) data;
#ifndef TEST_RC_ONLY
      /*update speed*/
      syslog(LOG_INFO,"rc send speed_req: vx: %f vz: %f \n",
	    speed_req->x_speed,speed_req->z_speed);
      motion_result = motion_speed_control(REMOTE_CONTROLLER, speed_req->x_speed
		      , speed_req->z_speed);
      if(motion_result != M_OK)
        {
          syslog(LOG_ERR,"send motion speed failed  %d\n",motion_result);
        }
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
  static enum rc_control_status_s curr_state = U_INIT;
  enum rc_control_status_s target_state;

  if (state != ST_REMOTE_CONTROLLING)
  {
    target_state = RC_STOP;
  }
  else
  {
    target_state = RC_UPDATE;
  }

  if (curr_state == target_state)
  {
    return;
  }

  if (target_state == RC_UPDATE)
  {
    /*enable rc data update*/
    syslog(LOG_INFO,"rc enable speed update, MCB status =%d.\n",state);
    ret = rc_manag_enable();
    if (ret != OK)
    {
      syslog(LOG_INFO,"rc enable speed update fail.\n");
      curr_state = U_INIT;
    }
  }
  else
  {
    /*disable rc data update*/
    syslog(LOG_INFO,"rc disable speed update, MCB status =%d.\n",state);
    ret = rc_manag_disable();
    if (ret != OK)
    {
      syslog(LOG_INFO,"rc disable speed update fail.\n");
      curr_state = U_INIT;
    }
  }
  curr_state = target_state;

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
    syslog(LOG_INFO,"rc read init_param: max_speed: %f, max_angle_speed: %f, rc_enable: %d.\n",
		    params.max_speed, params.max_angle_speed, params.rc_enable);
    if ((params.max_speed < 0) ||
	  (params.max_speed > MAX_SPEED))
    {
      g_rc_ctrl.init_setting.x_speed = MAX_SPEED;
    }
    else
    {
      g_rc_ctrl.init_setting.x_speed = params.max_speed;
    }

    if ((params.max_angle_speed < 0) ||
	  (params.max_angle_speed > MAX_ANGULAR_VELOCITY))
    {
      g_rc_ctrl.init_setting.z_speed = MAX_ANGULAR_VELOCITY;
    }
    else
    {
      g_rc_ctrl.init_setting.z_speed = params.max_angle_speed;
    }

    /*if rc_enable illegal, keep default value*/
    if (params.rc_enable == DISABLE)
    {
      g_rc_ctrl.init_setting.enable_rc_management = false;
    }
    else if (params.rc_enable == ENABLE)
    {
      g_rc_ctrl.init_setting.enable_rc_management = true;
    }
  }
  else
  {
    syslog(LOG_INFO,"rc read param fail \n");
    return status;
  }

  syslog(LOG_INFO,"rc param init result: max_speed: %f, max_angle_speed: %f, rc_enable: %d.\n",
		  g_rc_ctrl.init_setting.x_speed,
		  g_rc_ctrl.init_setting.z_speed,
		  g_rc_ctrl.init_setting.enable_rc_management);

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
#ifndef SKIP_PARAM_INIT
  /*read basic setting*/
  ret = read_the_RC_init_setting();
  if (ret != OK)
  {
    syslog(LOG_INFO,"%s read rc params fail.\n",__func__);
    goto err;
  }
#endif
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
