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

#include "motion_management.h"
#include "motion_sm.h"
#include "kinematics.h"
#include "motion_msg.h"
#include "8015d.h"

/* motor management call kinematic & motor hal APIs to realize motor control */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MOTOR_HAL             HAL_8015D
#define MOTOR_THREAD_FRQUENCY 50

#define MOTOR_DRIVER_STATUS_CHECK_FRQUENCY 10

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* motor management structure */

struct motor_management_s
{
  enum control_mode_e   mode;
  void *                motor_hal;
  struct motor_hal_ops *motor_ops;
  motion_odom_cb        motion_odom_cb;
  motor_notify_cb       pose_done_cb;
  motor_notify_cb       switch_done_cb;
  uint32_t              frequency;
} __attribute__((aligned(4)));

/* hal object */
enum motor_hal_index_e
{
  HAL_8015D,
  HAL_MAX,
};

struct motor_hal_s
{
  enum motor_hal_index_e index;
  void *                 motor;
  struct motor_hal_ops * hal_ops;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int motor_get_speed_odom(float *vx, float *vz);
//static bool motor_check_position_reach(void);
static int motor_speed_odom_work(union motion_control_data_u data);
static int motor_driver_status_work(union motion_control_data_u data);

/****************************************************************************
 * Private Data
 ****************************************************************************/
static const struct motor_hal_s g_hal_list[] = {
  { HAL_8015D, &g_zlac_8015d, &zlac_8015d_ops },
};

static struct motor_management_s g_motor_manager = {
  .mode           = INACTIVE,
  .motor_hal      = NULL,
  .motor_ops      = NULL,
  .motion_odom_cb = NULL,
  .pose_done_cb   = NULL,
  .switch_done_cb = NULL,
  .frequency      = MOTOR_THREAD_FRQUENCY,
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int motor_get_speed_odom(float *vx, float *vz)
{
  int                   result;
  void *                motor   = g_motor_manager.motor_hal;
  struct motor_hal_ops *hal_ops = g_motor_manager.motor_ops;
  float                 left_rpm, right_rpm;
  static int            num = 0;

  if (g_motor_manager.mode != SPEED)
    {
      syslog(LOG_ERR, "motor_get_speed failed \n");
      return ERROR;
    }

  result = hal_ops->get_rpm(motor, &left_rpm, &right_rpm);
  if (result == OK)
    {
      if (false == speed_rpm_transfer_to_odom(left_rpm, right_rpm, vx, vz))
        {
          syslog(LOG_ERR, "speed_rpm_transfer_to_odom failed\n");
          result = ERROR;
        }
    }
  syslog(LOG_ERR, "speed:left_rpm=%f	right_rpm=%f	vx=%f	vz=%f\n", left_rpm, right_rpm, *vx, *vz);
  num++;
  if (num % 20 == 0)
    {
      enum motor_err_e motor_status;
      hal_ops->get_motor_status_code(motor, &motor_status);
      syslog(LOG_ERR, "motor_status=%d", motor_status);
    }

  return result;
}

static int motor_speed_odom_work(union motion_control_data_u data)
{
  struct motion_odom_s odom;
  int                  result;
  struct timespec      timestamp;

  /* get speed odometry */

  odom.type = ODOM_SPEED;
  clock_gettime(CLOCK_REALTIME, &timestamp);
  odom.sec = timestamp.tv_sec;
  odom.ns  = timestamp.tv_nsec;
  /* call callback */

  if (g_motor_manager.motion_odom_cb != NULL)
    {
      result = motor_get_speed_odom(&odom.x, &odom.z);
      if (OK == result)
        {
          g_motor_manager.motion_odom_cb(odom);
        }
      else
        {
          syslog(LOG_ERR, "get odom failed result=%d\n", result);
          return ERROR;
        }
    }
  return OK;
}

static int motor_driver_status_work(union motion_control_data_u data)
{
  union motion_control_data_u driver_data;

  driver_data.motor_driver_status = motor_get_status_code();
  if (driver_data.motor_driver_status != NORMAL)
    {
      /* send drv_err event to motion stat machine */
      motion_sm_event(EV_DRI_ERR, driver_data);
      syslog(LOG_ERR, "ERROR: motor driver status error \n");
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: motor_set_pid
 ****************************************************************************/

int motor_set_pid(enum control_mode_e control_mode, struct motion_pid_s pid)
{
  void *motor = g_motor_manager.motor_hal;

  return g_motor_manager.motor_ops->set_pid(motor, control_mode, pid);
}

int motor_set_speed(float vx, float vz)
{
  int16_t               left_rpm, right_rpm;
  int                   result;
  void *                motor   = g_motor_manager.motor_hal;
  struct motor_hal_ops *hal_ops = g_motor_manager.motor_ops;

  if (speed_inverse_kinematics(vx, vz, &left_rpm, &right_rpm))
    {
      /* speed control */
      result = hal_ops->set_speed(motor, left_rpm, right_rpm);
      syslog(LOG_DEBUG, "motor_set_speed l_rpm=%d,r_rpm=%d\n", left_rpm, right_rpm);
    }
  else
    {
      syslog(LOG_ERR, "motor_set_speed speed_inverse_kinematics failed\n");
      result = ERROR;
    }

  return result;
}

int motor_quick_stop(bool enable)
{
  void *motor = g_motor_manager.motor_hal;

  if (enable)
    {
      syslog(LOG_ERR, "motor_quick_stop enable = %d \n", enable);
      /* reset mode as inactive */
      g_motor_manager.mode = INACTIVE;
      return g_motor_manager.motor_ops->quick_stop(motor);
    }

  return OK;
}

/* get motor driver status */
enum motor_err_e motor_get_status_code(void)
{
  enum motor_err_e      err_code = NORMAL;
  void *                motor    = g_motor_manager.motor_hal;
  struct motor_hal_ops *hal_ops  = g_motor_manager.motor_ops;

  hal_ops->get_motor_status_code(motor, &err_code);

  return err_code;
}

int motor_switch_mode(enum control_mode_e mode)
{
  bool result = false;

  /* present, just support speed */
  if (SPEED == mode)
    {
      if (g_motor_manager.mode == mode)
        {
          result = true;
        }
      else
        {
          result = motor_management_init();
        }
      return (result == true) ? OK : ERROR;
    }
  else
    {
      syslog(LOG_ERR, "motor switch error present_mode=%d\n,"
                      "target_mode=%d",
             g_motor_manager.mode, mode);
      return ERROR;
    }
}

int motor_set_position(bool pose_type, float pose)
{
  return ERROR;
}

void motor_register_odometry_cb(motion_odom_cb odom_cb)
{
  g_motor_manager.motion_odom_cb = odom_cb;
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

void motor_set_odom_frquency(uint32_t frequency)
{
  g_motor_manager.frequency = frequency;
}

bool motor_management_init(void)
{
  bool                  result = false;
  void *                motor  = g_hal_list[MOTOR_HAL].motor;
  struct motor_hal_ops *ops    = g_hal_list[MOTOR_HAL].hal_ops;
  ;

  /* get hal */
  g_motor_manager.motor_hal = motor;
  g_motor_manager.motor_ops = ops;

  /* init hal */
  result = g_motor_manager.motor_ops->motor_hal_init(motor);
  if (result != true)
    {
      syslog(LOG_ERR, "ERROR:init_motor:  motor init failed\n");
      return false;
    }
  /* default mode is speed mode */
  g_motor_manager.mode = SPEED;
  syslog(LOG_INFO, "init_motor:  motor init done\n");
  return result;
}

/****************************************************************************
 * Name: motor_management_thread
 * function: get motor odom & status data
 ****************************************************************************/

int motor_management_thread(int argc, char *argv[])
{
  uint32_t                    frequency = g_motor_manager.frequency;
  enum motion_sm_state_e      control_state;
  union motion_control_data_u motion_data;
  uint32_t                    count      = 0;
  uint32_t                    driver_num = frequency / MOTOR_DRIVER_STATUS_CHECK_FRQUENCY;
  syslog(LOG_INFO, "motor_management_thread:  starting \n");

  while (true)
    {
      /* set frequency */
      usleep(1000000 / frequency);

      control_state = get_motion_sm_state();
      if (control_state == ST_SPEED)
        {
          motion_add_work(motor_speed_odom_work, motion_data);

          /* get motor driver status */
          count++;
          if (count % driver_num == 0)
            {
              motion_add_work(motor_driver_status_work, motion_data);
            }
        }
    }

  return ERROR;
}
