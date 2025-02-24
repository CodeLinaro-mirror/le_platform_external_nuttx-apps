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
#include <math.h>

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
emergency_speed_check_cb g_speed_subscribe_fun = NULL;

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
  enum motion_result_e        result;
  union motion_control_data_u data;
  enum motion_sm_event_e      motion_event = EV_CMD_SPEED;

  /* check client if fit  */
  if (client == get_current_client())
    {
      /* check motion_sm if fit  */
      if (ST_SPEED == get_motion_sm_state())
        {
          data.speed_cmd.vx = vx;
          data.speed_cmd.vz = vz;
          result            = motion_sm_event(motion_event, data);
          if (NULL != g_speed_subscribe_fun)
            {
              g_speed_subscribe_fun(vx, vz);
            }
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
  enum motion_sm_event_e      motion_event;

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
  enum motion_result_e        result;
  union motion_control_data_u data;
  enum motion_sm_event_e      motion_event;

  if (mode == SPEED)
    {
      motion_event = EV_CMD_SWITCH_SPEED;
    }
  else if (mode == POSITION)
    {
      motion_event = EV_CMD_SWITCH_POS;
    }
  else if (mode == SET_DRV_ERR)
    {
      motion_event = EV_DRI_ERR;
    }
  else
    {
      syslog(LOG_ERR, "Motion switch mode invalid %d\n", mode);
      return ERROR;
    }
  data.mode = mode;
  result    = motion_sm_event(motion_event, data);
  syslog(LOG_INFO, "Motion sm state = %d \n", get_motion_sm_state());

  return result;
}

enum motion_result_e motion_position_control(enum control_client_e client,
                                             float pose, int pose_type,
                                             motion_action_done_cb position_done_cb,
                                             void *                arg)
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
  struct config_car_s          config_car;
  struct config_motion_s       config_motion;
  struct config_scale_s        config_scales;
  uint32_t                     odom_frequency;
  int                          result;
  int                          car_mode;

  /* get config parameters kinematic. */
  result = get_configuration_parameters(CAR, (void *)&config_car);
  result |= get_configuration_parameters(MOTION, (void *)&config_motion);
  result |= get_configuration_parameters(SCALE, (void *)&config_scales);
  if (result != OK)
    {
      syslog(LOG_INFO, "motion_management_init:  get config parameters Failed \n");
      config_notify_completed(false);
      return result;
    }

  parameters.speed_line_scale       = config_scales.speed_scale[0];
  parameters.speed_angle_scale      = config_scales.speed_scale[1];
  parameters.speed_odom_line_scale  = config_scales.speed_odom_scale[0];
  parameters.speed_odom_angle_scale = config_scales.speed_odom_scale[1];
  parameters.wheel_perimeter        = fabs(config_car.wheel_perimeter);
  parameters.wheel_space            = fabs(config_car.wheel_space);
  parameters.speed_max              = fabs(config_motion.max_speed);
  parameters.angle_speed_max        = fabs(config_motion.max_angle_speed);
  parameters.kinematic_model        = config_car.kinematic_model;

  /* check car mode */
  if (config_car.car_model >= CAR_MODE_MAX || config_car.car_model < 0)
    {
      syslog(LOG_ERR, "motion_management_init: car_model=%d is invalid\n", config_car.car_model);
      config_notify_completed(false);
      return ERROR;
    }

  /* check speed limit */
  if (parameters.speed_max < SPEED_RESOLUTION || parameters.angle_speed_max < SPEED_RESOLUTION)
    {
      syslog(LOG_ERR, "motion_management_init: speed_max=%f angle_speed=%f are invalid\n",
              parameters.speed_max, parameters.angle_speed_max);
      config_notify_completed(false);
      return ERROR;
    }

  /* kinematic init */
  if (OK != kinematic_init(&parameters))
    {
      config_notify_completed(false);
      syslog(LOG_INFO, "motion_management_init: init kinematic Failed \n");
      return ERROR;
    }

  /* motion sm init */
  if (OK != motion_sm_init())
    {
      syslog(LOG_ERR, "motion_management_init: motion sm init failed \n");
      config_notify_completed(false);
      return ERROR;
    }
  syslog(LOG_INFO, "motion_management_init: motion sm init done \n");

  /* client control sm init */
  client_sm_init();
  syslog(LOG_INFO, "motion_management_init: client sm init done \n");

  /* motor management init */
  if (parameters.wheel_space > 0.25 )
      car_mode = AMR_6040;
  else
      car_mode = CYCLE_CAR;

  odom_frequency = config_motion.odom_frequency;
  motor_set_parameters(odom_frequency,car_mode);

  result = task_create("motor_manag",
                       DEFAULT_PRIORITY,
                       DEFAULT_STACK_SIZE,
                       motor_management_thread,
                       NULL);
  if (result < 0)
    {
      syslog(LOG_INFO, "motion_management_init: Failed to start motor\n");
      config_notify_completed(false);
      return result;
    }
//add pid control thread
  result = task_create("motor_pid",
                       DEFAULT_PRIORITY,
                       DEFAULT_STACK_SIZE,
                       motor_pid_control_thread,
                       NULL);
  if (result < 0)
    {
      syslog(LOG_INFO, "motion_management_init: Failed to start pid\n");
      config_notify_completed(false);
      return result;
    }

  /* notify done */
  syslog(LOG_INFO, "motion_management_init:  notify done \n");
  config_notify_completed(true);

  /* motion action thread join */
  motion_sm_join();

  return OK;
}

void motion_motor_stop(bool stop)
{
  syslog(LOG_INFO, "motion_motor_stop %s\n", stop ? "ENTER" : "EXIT");
  motion_sm_stop_speed(stop);
}

void register_emergency_check_speed_cb(emergency_speed_check_cb check_cb)
{
  register_motor_emergency_check_cb(check_cb);
}

void register_speed_subscribe_cb(emergency_speed_check_cb fun_cb)
{
  g_speed_subscribe_fun = fun_cb;
}
