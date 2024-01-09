/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/
#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include "motion_management.h"
#include "motion_sm.h"
#include "kinematics.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: motion_speed_control
 ****************************************************************************/

enum motion_result_e motion_speed_control(enum control_client_e client,
                                          float vx, float vz)
{
  enum motion_result_e result;
  union motion_control_data_u data;
  enum motion_sm_event_e  motion_event = EV_CMD_SPEED;

  /* check client if fit  */
  if (client == get_current_client())
    {
      /* check motion_sm if fit  */
      if (ST_SPEED == get_motion_sm_state())
        {
          data.speed_cmd.vx = vx;
          data.speed_cmd.vz = vz;
          result = motion_sm_event(motion_event,data);
        }
      else
       {
          result = SM_ERR;
       }
    }
  else
    result = CLIENT_ERR;

  return result;
}

enum motion_result_e motion_set_emergency(bool enable)
{
  union motion_control_data_u data;
  enum motion_sm_event_e  motion_event;

  data.emergency = enable;

  if (enable)
    {
      motion_event = EV_ENTER_EMERGENCY;
      syslog(LOG_INFO, "Motion enter emergency\n");
    }
  else
    {
      motion_event = EV_EXIT_EMERGENCY;
      syslog(LOG_INFO, "Motion exit emergency\n");
    }

  return motion_sm_event(motion_event, data);
}

enum motion_result_e motion_switch_mode(enum control_mode_e mode)
{
  enum motion_result_e result;
  union motion_control_data_u data;
  enum motion_sm_event_e  motion_event;

  if (mode == SPEED)
    {
      motion_event = EV_CMD_SWITCH_SPEED;
    }
  else if (mode == POSITION)
    {
      motion_event = EV_CMD_SWITCH_POS;
    }
  else
    {
      syslog(LOG_ERR, "Motion switch mode invalid %d\n", mode);
      return ERROR;
    }
  data.mode = mode;
  result = motion_sm_event(motion_event,data);

  return result;
}

enum motion_result_e motion_position_control(enum control_client_e client,
                                              float pose,int pose_type,
                                              motion_action_done_cb position_done_cb,
                                              void *arg)
{
  return ERROR;
}

void register_motion_odom_cb(motion_odom_cb cb_fun)
{
  motor_register_odometry_cb(cb_fun);
}

/****************************************************************************
 * Name: motion_management_init
 ****************************************************************************/
void motion_management_init(void)
{
  struct kinematic_parameter_s parameters;

//get config parameters kinematic.
//call cofig parameters_api
  kinematic_init(&parameters);

//motor management init。
//motion sm init
//client control sm init.

}
