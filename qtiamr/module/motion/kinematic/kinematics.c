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
#include "kinematics.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define  KINEMATIC_MODE (DIFF_CAR)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct kinematic_s
{
  struct kinematic_parameter_s parameters;
  struct kinematic_ops *ops;
  bool initialized;
}__attribute__((aligned(4)));

struct kinematic_mode_s
{
  enum kinematic_model_e mode;
  struct kinematic_ops *ops;
}__attribute__((aligned(4)));

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct kinematic_mode_s g_mode_list[] =
{
  {DIFF_CAR, &diff_car_ops},
  {ACKERMAN_CAR, NULL},
};

static struct kinematic_s g_kinematic_s =
{
  .initialized = false,
};
/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kinematic_init
 * Description: get kinematic configure parameters.
 ****************************************************************************/
int kinematic_init(const struct kinematic_parameter_s *parameters)
{
  enum kinematic_model_e model;

  if (NULL == parameters)
    {
      return ERROR;
    }

   /* check kinematic model */
  if(parameters->kinematic_model< 0 ||
      parameters->kinematic_model >= KINEMATIC_MODEL_MAX)
    {
      syslog(LOG_INFO,"kinematic_init: k_model=%d is invalid \n",parameters->kinematic_model);
      return ERROR;
    }

  memcpy(&g_kinematic_s.parameters, parameters,sizeof(struct kinematic_parameter_s));
  model = g_kinematic_s.parameters.kinematic_model;
  g_kinematic_s.ops = g_mode_list[model].ops;
  g_kinematic_s.initialized = true;

  return OK;
}

bool speed_inverse_kinematics(float vx, float vz, int16_t *rpm_l, int16_t *rpm_r)
{
  return g_kinematic_s.ops->speed_inverse(g_kinematic_s.parameters,vx,vz,rpm_l,rpm_r);
}

bool speed_rpm_transfer_to_odom(float rpm_left, float rpm_right,
                                float *speed_vx, float *speed_vz)
{
  return g_kinematic_s.ops->speed_rpm_transfer_to_odom(g_kinematic_s.parameters
                                          ,rpm_left,rpm_right,speed_vx,speed_vz);
}

bool position_inverse_kinematics(float pos_left, float pos_right, int *count_l, int *count_r)
{
  return false;
}

bool pos_count_transfer_to_odom(int count_left, int count_right, float *pose_dist, float *pose_angle)
{
  return false;
}
