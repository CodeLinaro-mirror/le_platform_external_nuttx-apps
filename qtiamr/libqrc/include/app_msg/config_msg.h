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
  APPLY,
  CAR,
  MOTION,
  SCALE,
  SENSOR,
  REMOTE_CONTROLLER,
  OBSTACLE_AVOIDANCE,
};

struct config_apply_s
{
  // 0 is success, not 0 is failed
  uint8_t status;
  // which parameter cause error
  enum config_msg_type_e error_type;
};

struct config_car_s
{
  uint8_t model;
  double length;
  double width;
  double height;
  double wheel_space;
  double wheel_radius;
  uint8_t kinematic_model;
};

struct config_motion_s
{
  double max_speed[2];
  double max_position[2];
  double pid_speed[3];
  double pid_position[3];
  uint32_t hz_odom;
};

struct config_scale_s
{
  double speed[2];
  double position[2];
  double odom_speed[2];
  double odom_position[2];
};

struct config_sensor_s
{
  uint8_t imu_enable;
  uint8_t ultra_enable;
  uint8_t ultra_count;
};

struct config_remote_controller_s
{
  double max_speed[2];
  double max_position[2];
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
