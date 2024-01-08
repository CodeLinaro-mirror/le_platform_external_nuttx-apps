/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#ifndef __MOTION_MSG_H
#define __MOTION_MSG_H

#include <stdio.h>
#include <stdint.h>

/* Robot motion pipe name */
#define MOTION_PIPE "motion"

/* Motion odom pipe name */
#define ODOM_PIPE "odometry"

enum control_mode_e
{
  SPEED = 0x00,
  POSITION,
  TORQUE
};

enum odom_msg_type_e
{
  ODOM_SPEED = 0x00,
  ODOM_POSITION,
};

enum control_msg_type_e
{
  SET_SPEED = 0x00,
  SWITCH_MODE,
  SET_EMERGENCY,
  SET_POSITION
};

enum motor_err_e
{
  NORMAL = 0x00,
  OVER_POWER,
  LACK_POWER,
  OVER_LOAD,
  EEPROM_ERR,
  ENCODER_ERR,
  OTHER_ERR
};

struct speed_cmd_s
{
	  float vx;
	  float vz;
};

struct position_cmd_s
{
  bool pose_type;
  float pose;
};

union motion_control_data_u
{
  struct speed_cmd_s speed_cmd;
  struct position_cmd_s position_cmd;
  bool emergency;
  enum control_mode_e mode;
} __attribute__((aligned(4)));

struct motion_control_msg_s
{
  enum control_msg_type_e msg_type;
  union motion_control_data_u data;
} __attribute__((aligned(4)));

/* motion odom structure */
struct motion_odom_s
{
  enum odom_msg_type_e type;
  struct timespec timestamp;
	float x;
	float z;
} __attribute__((aligned(4)));

#endif