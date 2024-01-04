/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
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
#include "motor_management.h"

#include "8015d.h"
#include "motion_management.h"
#include "motion_sm.h"

/* motor management call kinematic & motor hal APIs to realize motor control */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MOTOR_HAL HAL_8015D
#define MOTOR_THREAD_FRQUENCY 50

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* motor management structure */

struct motor_management_s
{
  enum control_mode_e mode;
  void *motor_hal;
  struct motor_hal_ops *motor_ops;
  motor_odom_cb   motor_odom_cb;
  motor_notify_cb pose_done_cb;
  motor_notify_cb switch_done_cb;
}__attribute__((aligned(4)));

/* hal object */
enum motor_hal_index_e {
  HAL_8015D,
  HAL_MAX
}

struct motor_hal_s
{
  enum motor_hal_index_e index,
  void *motor;
  struct motor_hal_ops *hal_ops;
}

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static bool init_motor(void);
static int motor_get_speed_odom(float *vx, float *vz);
static bool motor_check_position_reach(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/
const struct motor_hal_s g_hal_list[] = {
  {HAL_8015D, &g_8015d, &8015d_ops}
}

static struct motor_management_s g_motor_manager =
{
  .motor_hal = NULL,
  .motor_ops = NULL,
  .motor_odom_cb = NULL,
  .pose_done_cb = NULL,
  .switch_done_cb = NULL
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static bool init_motor(void)
{
  bool result = false;
  void * motor = g_hal_list[MOTOR_HAL].motor;
  struct motor_hal_ops *ops = g_hal_list[MOTOR_HAL].hal_ops;;

  /* get hal */
  g_motor_manager.motor_hal = motor;
  g_motor_manager.motor_ops = ops;

  /* init hal */
  result = ops->motor_hal_init(motor);
  if (result == OK)
    {
      g_motor_manager.mode = SPEED;
    }

  return result;
}

static int motor_get_speed_odom(float *vx, float *vz)
{
  int result;
  void *motor = g_motor_manager.motor_hal;
  struct motor_hal_ops *hal_ops = g_motor_manager.motor_ops;
  float left_rpm,right_rpm;

  result = hal_ops->get_rpm(motor, &left_rpm, &right_rpm);
  if (result == OK)
    {
      result = speed_rpm_transfer_to_odom(left_rpm, right_rpm, vx, vz);
    }
  return result;
}

static bool motor_check_position_reach(void)
{
  return false;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: motor_set_pid
 ****************************************************************************/

int motor_set_pid(enum control_mode_e control_mode, struct motion_pid_s pid)
{
  void *motor = g_motor_manager.motor_ops->motor;

  return g_motor_manager.motor_ops->set_pid(motor, control_mode, pid);
}

int motor_set_speed(float vx, float vz)
{
  int16_t left_rpm,right_rpm;
  int result;
  void *motor = g_motor_manager.motor_hal;
  struct motor_hal_ops *hal_ops = g_motor_manager.motor_ops;

  if (speed_inverse_kinematics(vx, vz, &left_rpm, &right_rpm))
    {
        /* speed control */
        result = hal_ops->set_speed(motor, left_rpm, right_rpm);
    }
  else
    {
      result = ERROR;
    }

  return result;
}

int motor_quick_stop(bool enable)
{
  void *motor = g_motor_manager.motor_ops->motor;

  if (enable)
  {
    return g_motor_manager.motor_ops->quick_stop(motor);
  }

  return ERROR;
}

enum motor_err_e motor_get_status_code(void)
{
    return OK;
}

int motor_switch_mode(enum control_mode_e mode)
{
  if (g_motor_manager.mode == mode)
    {
      return OK;
    }
  else
    {
      syslog(LOG_ERR,"motor switch error present_mode=%d\n,
                      target_mode=%d",g_motor_manager.mode,mode);
      return ERROR;
    }
}

int motor_set_position(bool pose_type, float pose)
{
  return ERROR;
}

void motor_register_odometry_cb(motor_odom_cb odom_cb)
{
  g_motor_manager.motor_odom_cb = odom_cb;
}

/* register action done cb */
void motor_register_position_done_cb(motor_notify_cb pose_done_cb)
{
  g_motor_manager.pose_done_cb = pose_done_cb;
}

void motor_register_switching_done_cb(motor_notify_cb switch_done_cb)
{
  g_motor_manager.switch_done_cb = switch_done_cb;
}

/* get motor odom & status data */
void motor_management_thread(void)
{
  enum motion_sm_e control_state;
  struct motion_odom_s odom;
  int result =ERROR;
  float rpm

  if (!init_motor())
    {
      syslog(LOG_ERR,"motor management init error \n");
      return;
    }

  while(true)
    {
      /* set frequency */
      usleep(1000000/MOTOR_THREAD_FRQUENCY);

      control_state = get_motion_sm_state();
      if (control_state == SPEED)
        {
          /* get speed odometry */

          odom.type = ODOM_SPEED;
          clock_gettime(CLOCK_REALTIME, &odom.timestamp);
          motor_get_speed_odom(&odom.vx, &odom.vz);
          /* call callback */
          if (g_motor_manager.motor_odom_cb != NULL)
            {
              g_motor_manager.motor_odom_cb(odom);
            }
        }
    }
}
