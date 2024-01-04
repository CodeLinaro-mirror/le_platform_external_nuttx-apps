/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#ifndef __CONFIG_MSG_H
#define __CONFIG_MSG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define CONFIG_PIPE "config"

enum config_msg_type_e
{
  CAR = 0x0,
  MOTION,
  SCALE,
  SENSOR,
  REMOTE_CONTROLLER,
  OBSTACLE_AVOIDANCE,
  CONFIG_MSG_TYPE_MAX,
  APPLY,
};

struct config_apply_s
{
  // 0 is success, not 0 is failed
  uint8_t status;
  // which parameter cause error
  enum config_msg_type_e error_type;
};

enum car_model_e
{
  CYCLE_CAR,
  AMR_6040,
  CAR_MODE_MAX,
};

enum kinematic_model_e
{
  DIFF_CAR,
  ACKERMAN_CAR,
  KINEMATIC_MODEL_MAX,
};

struct config_car_s
{
  enum car_model_e car_model;
  enum kinematic_model_e kinematic_model;
  double wheel_space;
  double wheel_radius;
};

struct config_motion_s
{
  double max_speed;
  double max_position;
  double max_position_line_speed;
  double max_position_angle_speed;
  double pid_speed[3];
  double pid_position[3];
  uint32_t odom_frequency;
};

struct config_scale_s
{
  double speed_scale[2];
  double position_scale[2];
  double speed_odom_scale[2];
  double position_odom_scale[2];
};

struct config_sensor_s
{
  uint8_t imu_enable;
  uint8_t ultra_enable;
  uint8_t ultra_quantity;
};

struct config_remote_controller_s
{
  double max_speed;
};

struct config_obstacle_avoidance_s
{
  double safe_distance;
};

struct config_msg_s
{
  enum config_msg_type_e type;
  union
  {
    struct config_apply_s apply;
    struct config_car_s car;
    struct config_motion_s motion;
    struct config_scale_s scale;
    struct config_sensor_s sensor;
    struct config_remote_controller_s rc;
    struct config_obstacle_avoidance_s ob;
  } data;
} __attribute__((aligned(4)));

#ifdef __cplusplus
}
#endif

#endif  // __ROBOT_CONFIG_MSG_H
