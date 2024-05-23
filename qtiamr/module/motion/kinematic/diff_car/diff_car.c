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

#include "diff_car.h"
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
static bool diff_speed_inverse_kinematics(const struct kinematic_parameter_s parameters,
                                          float vx, float vz, int16_t *rpm_l, int16_t *rpm_r);

static bool diff_rpm2odom(const struct kinematic_parameter_s parameters,
                          float rpm_left, float rpm_right, float *speed_vx,
                          float *speed_vz);

static bool diff_position_inverse_kinematics(const struct kinematic_parameter_s parameters,
                                             float pos, int pos_type, int *count_l,
                                             int *count_r);

static bool diff_count2odom(const struct kinematic_parameter_s parameters,
                            int count_left, int count_right,
                            float *pose_dist, float *pose_angle);

/****************************************************************************
 * Public Data
 ****************************************************************************/
struct kinematic_ops diff_car_ops = {
  .speed_inverse              = diff_speed_inverse_kinematics,
  .position_inverse           = diff_position_inverse_kinematics,
  .pos_count_transfer_to_odom = diff_count2odom,
  .speed_rpm_transfer_to_odom = diff_rpm2odom,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool diff_speed_inverse_kinematics(const struct kinematic_parameter_s parameters,
                                          float vx, float vz, int16_t *rpm_l, int16_t *rpm_r)
{
  float v_left, v_right;
  float max_vx            = parameters.speed_max;
  float max_vz            = parameters.angle_speed_max;
  float wheel_space       = parameters.wheel_space;
  float input_line_scale  = parameters.speed_line_scale;
  float input_angle_scale = parameters.speed_angle_scale;
  float wheel_perimeter   = parameters.wheel_perimeter;

  if ((rpm_l == NULL) || (rpm_l == NULL))
    {
      syslog(LOG_ERR, " kinematic input invalid \n");
      return false;
    }

  /* scale to solve input error  */
  vx = vx * input_line_scale;
  vz = vz * input_angle_scale;

  vx = AMP_LIMIT(vx, -max_vx, max_vx);
  vz = AMP_LIMIT(vz, -max_vz, max_vz);

  if (vx == 0)
    {
      v_right = (vz * wheel_space / 2.0);
      v_left  = (-1) * v_right;
    }
  else if (vz == 0)
    {
      v_left = v_right = vx;
    }
  else
    {
      v_left  = vx - vz * wheel_space / 2.0f;
      v_right = vx + vz * wheel_space / 2.0f;
    }

  /* motor target speed limit */

  *rpm_l = (int16_t)(v_left * 60 / wheel_perimeter);
  *rpm_r = (int16_t)(-v_right * 60 / wheel_perimeter);

  return true;
}

static bool diff_rpm2odom(const struct kinematic_parameter_s parameters,
                          float rpm_left, float rpm_right, float *speed_vx,
                          float *speed_vz)
{
  float odom_line_scale  = parameters.speed_odom_line_scale;
  float odom_angle_scale = parameters.speed_odom_angle_scale;
  float wheel_space      = parameters.wheel_space;
  float wheel_perimeter  = parameters.wheel_perimeter;
  float v_left, v_right;
  float vx, vz;

  if ((speed_vx == NULL) || (speed_vz == NULL))
    {
      syslog(LOG_ERR, " kinematic input invalid \n");
      return false;
    }

  v_left  = (rpm_left * wheel_perimeter / 60);
  v_right = (-rpm_right * wheel_perimeter / 60);
  vx      = (v_left + v_right) / 2.0f;
  vz      = (v_right - v_left) / wheel_space;

  *speed_vx = vx * odom_line_scale;
  *speed_vz = vz * odom_angle_scale;

  return true;
}

static bool diff_position_inverse_kinematics(const struct kinematic_parameter_s parameters,
                                             float pos, int pos_type, int *count_l,
                                             int *count_r)
{
  return false;
}

static bool diff_count2odom(const struct kinematic_parameter_s parameters,
                            int count_left, int count_right,
                            float *pose_dist, float *pose_angle)
{
  return false;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/