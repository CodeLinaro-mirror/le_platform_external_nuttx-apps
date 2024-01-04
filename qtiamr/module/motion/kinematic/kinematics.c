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

#include "diff_car.h"
#include "kinematic.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define  KINEMATIC_MODE DIFF_CAR

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct kinematic_s
{
  struct kinematic_parameter_s parameters;
  struct kinematic_ops *ops;

}__attribute__((aligned(4)));

struct kinematic_mode_s
{
  enum kinematic_mode_e mode;
  struct kinematic_ops *ops;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct kinematic_mode_s g_mode_list[] = {
  {DIFF_CAR, &diff_car_ops},
}

static struct kinematic_s g_kinematic_s;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kinematic_parameter_init
 * Description: get kinematic configure parameters.
 ****************************************************************************/
void kinematic_init(const struct kinematic_parameter_s *parameters)
{
  g_kinematic_s.ops = g_mode_list[KINEMATIC_MODE].ops;
  memcpy(&g_kinematic_s.parameters, parameters,sizeof(struct kinematic_parameter_s));
}

bool speed_inverse_kinematics(float vx, float vz, int16_t *rpm_l, int16_t *rpm_r)
{
  return g_kinematic_s.ops->speed_inverse(g_kinematic_s.parameters,vx,vz,rpm_l,rpm_r);
}

bool speed_rpm_transfer_to_odom(float rpm_left, float rpm_right, float *speed_vx, float *speed_vz)
{
  return g_kinematic_s.ops->speed_inverse(g_kinematic_s.parameters,rpm_left,rpm_right,speed_vx,speed_vz);
}

bool position_inverse_kinematics(float pos_left, float pos_right, int *count_l, int *count_r)
{
  return false;
}

bool pos_count_transfer_to_odom(int count_left, int count_right, float *pose_dist, float *pose_angle)
{
  return false;
}
