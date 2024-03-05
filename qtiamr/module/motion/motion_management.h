/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#ifndef __MOTION_MANAGEMENT_H
#define __MOTION_MANAGEMENT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include "client_control_msg.h"
#include "motion_msg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

typedef bool (*emergency_speed_check_cb)(float vx, float vz);  /* emergency check speed callback */

/* Pose type, used to position control */
#define POSE_ANGLE (false)
#define POSE_DIST (true)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* control state machine state */

enum control_sm_state_e
{
  ST_ROBOT_CONTROLLING = 0x00,
  ST_CHARGER_CONTROLLING,
  ST_REMOTE_CONTROLLING
};

struct motion_pid_s
{
  float kp;
  float ki;
  float kd;
};

/* motion result */

enum motion_result_e
{
  DRV_ERR = -1,
  M_OK = 0,
  DRV_BUSY,
  CLIENT_ERR,
  SM_ERR,
};

struct motion_management_s
{
  bool initialized;
  struct motion_sm_s *motion_sm;
  struct control_sm_s *control_sm;
}__attribute__((aligned(4)));


typedef void (*control_sm_notify_cb)(enum control_sm_state_e state);  /* control sm cb for state switch */

typedef void (*motion_odom_cb)(struct motion_odom_s motion_odom);  /* motion odom cb */

typedef void (*motion_action_done_cb)(void *pipe,void * data, size_t len, bool response);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
/* Client control SM */

void client_sm_init(void);
enum control_client_e get_current_client(void);
int set_control_client(enum control_client_e client);
bool client_control_sm_register_notify_cb(control_sm_notify_cb fun_cb);

/* Motion management */
enum motion_result_e motion_speed_control(enum control_client_e client,
                                          float vx, float vz);

enum motion_result_e motion_position_control(enum control_client_e client,
                                              float pose,int pose_type,
                                              motion_action_done_cb position_done_cb,
                                              void *arg);

enum motion_result_e motion_switch_mode(enum control_mode_e mode);

enum motion_result_e motion_set_emergency(bool enable);

void register_motion_odom_cb(motion_odom_cb cb_fun);

void motor_set_odom_frquency(uint32_t frequency);

int motion_management_init(int argc, char *argv[]);

void motion_motor_stop(bool stop);
void register_emergency_check_speed_cb(emergency_speed_check_cb check_cb);

#endif /* __MOTION_MANAGEMENT_H */