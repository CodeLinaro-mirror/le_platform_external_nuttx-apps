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
#include "main.h"
#include "config_msg.h"

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
int motion_management_init(int argc, char *argv[])
{
  struct kinematic_parameter_s parameters;
  struct config_car_s config_car;
  struct config_motion_s config_motion;
  struct config_scale_s config_scales;
  uint32_t odom_frequency;
  int result;

  /* get config parameters kinematic. */
  result = get_configuration_parameters(CAR, (void *)&config_car);
  result |= get_configuration_parameters(MOTION, (void *)&config_motion);
  result |= get_configuration_parameters(SCALE, (void *)&config_scales);
  if (result != OK)
    {
      config_notify_completed(false);
      return result;
    }
  parameters.speed_line_scale = config_scales.speed_scale[0];
  parameters.speed_angle_scale = config_scales.speed_scale[1];
  parameters.speed_odom_line_scale = config_scales.speed_odom_scale[0];
  parameters.speed_odom_angle_scale = config_scales.speed_odom_scale[1];
  parameters.wheel_perimeter = config_car.wheel_perimeter;
  parameters.wheel_space = config_car.wheel_space;
  parameters.speed_max = config_motion.max_speed;
  parameters.angle_speed_max = config_motion.max_angle_speed;
  parameters.kinematic_model = config_car.kinematic_model;

  /* kinematic init */
  if (OK != kinematic_init(&parameters))
    {
      config_notify_completed(false);
      return ERROR;
    }

  /* motor management init */
  odom_frequency = config_motion.odom_frequency;
  motor_set_odom_frquency(odom_frequency);

  result = task_create("motor_manag",
                        DEFAULT_PRIORITY,
                        DEFAULT_STACK_SIZE,
                        motor_management_thread,
                        NULL);
  if (result < 0)
    {
      syslog(LOG_INFO,"motion_management_init: Failed to start motor\n");
      config_notify_completed(false);
      return result;
    }

  /* motion sm init */
  if (0 != motion_sm_init())
    {
      config_notify_completed(false);
      return ERROR;
    }

  /* client control sm init */
  client_sm_init();

  /*notify done */
  config_notify_completed(true);

  return OK;
}
