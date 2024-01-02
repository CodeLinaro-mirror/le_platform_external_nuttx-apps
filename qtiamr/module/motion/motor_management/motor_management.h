/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#ifndef __MOTOR_MANAGEMENT_H
#define __MOTOR_MANAGEMENT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* message callback function format */
typedef void (*motor_notify_cb)(void);
typedef  motion_odom_cb motor_odom_cb;

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct motor_hal_ops {
  bool  (*motor_hal_init)(void *motor);
  void  (*motor_hal_deinit)(void *motor);

  int   (*switch_mode)(void *motor, enum control_mode_e mode);
  int   (*set_pid)(void *motor, enum control_mode_e, struct motion_pid_s pid);
  int   (*set_speed)(void *motor, int16_t left_rpm, int16_t right_rpm);
  int   (*set_position)(void *motor, int left_count, int right_count);
  int   (*quick_stop)(void *motor);

  int   (*get_rpm)(void *motor, float *left, float *right);
  int   (*get_count)(void *motor, int *left, int * right);
  int   (*get_motor_status_code)(void *motor, enum motor_err_e * motor_status);
  bool  (*check_pose_reach)(void *motor);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* motor management IF */

bool init_motor(void);
int motor_set_pid(enum control_mode_e control_mode, struct motion_pid_s pid);
int motor_set_speed(float vx, float vz);
int motor_quick_stop(bool enable);
int motor_switch_mode(enum control_mode_e mode);
int motor_set_position(bool pose_type, float pose);

enum motor_err_e motor_driver_status_code(void);

/* callback register for odom */

void motor_register_odometry_cb(motor_odom_cb odom_cb);

/* register action done cb */
void motor_register_position_done_cb(motor_notify_cb pose_done_cb);
void motor_register_switching_done_cb(motor_notify_cb switch_done_cb);

void motor_management_thread(void);

#endif /* __MOTOR_MANAGEMENT_H */