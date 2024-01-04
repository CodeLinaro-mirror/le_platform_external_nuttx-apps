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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef void (*control_sm_notify_cb)(enum control_sm_e state);  /* control sm cb for state switch */

typedef void (*motion_odom_cb)(struct motion_odom_s motion_odom);  /* motion odom cb */

/* Pose type, used to position control */
#define POSE_ANGLE (false)
#define POSE_DIST (true)

struct motion_pid_s
{
  float kp;
  float ki;
  float kd;
};

/* motion result */

enum motion_result_e
{
  ERROR = -1,
  OK=0,
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
                                              float pose,int pose_type);
enum motion_result_e motion_switch_mode(enum control_mode_e mode);

enum motion_result_e motion_set_emergency(bool enable);

void register_motion_odom_cb(motion_odom_cb cb_fun);

#endif /* __MOTION_MANAGEMENT_H */