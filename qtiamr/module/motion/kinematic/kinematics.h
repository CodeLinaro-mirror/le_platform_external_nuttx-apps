/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/
#ifndef __KINEMATIC_H
#define __KINEMATIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AMP_LIMIT(_val_, _min_, _max_)  \
        ((_val_) < (_min_) ?  (_min_) : \
        ((_val_) > (_max_) ? (_max_) : (_val_)))


/****************************************************************************
 * Public Types
 ****************************************************************************/

/* kinematic parameter structure */

struct kinematic_parameter_s
{
  float speed_line_scale;
  float speed_angle_scale;
  float speed_odom_line_scale;
  float speed_odom_angle_scale;
  float wheel_perimeter;
  float wheel_space;
  float speed_max;
  float angle_speed_max;
} __attribute__((aligned(4)));

struct kinematic_ops {
  bool  (*speed_inverse)(const struct kinematic_parameter_s parameters,
                                float vx, float vz, int16_t *rpm_l, int16_t *rpm_r);
  bool  (*position_inverse)(const struct kinematic_parameter_s parameters,
                                float pos, int pos_type, int *count_l,
                                int *count_r);
  bool  (*pos_count_transfer_to_odom)(const struct kinematic_parameter_s parameters,
                                int count_left, int count_right,
                                float *pose_dist, float *pose_angle);
  bool  (*speed_rpm_transfer_to_odom)(const struct kinematic_parameter_s parameters,
                                float rpm_left, float rpm_right, float *speed_vx,
                                float *speed_vz);
};


/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void kinematic_init(const struct kinematic_parameter_s *parameters);
bool speed_inverse_kinematics(float vx, float vz, int16_t *rpm_l, int16_t *rpm_r);
bool speed_rpm_transfer_to_odom(float rpm_left, float rpm_right, float *speed_vx, float *speed_vz);

bool position_inverse_kinematics(float pos_left, float pos_right, int *count_l, int *count_r);
bool pos_count_transfer_to_odom(int count_left, int count_right, float *pose_dist, float *pose_angle);

#endif /* __KINEMATIC_H */
