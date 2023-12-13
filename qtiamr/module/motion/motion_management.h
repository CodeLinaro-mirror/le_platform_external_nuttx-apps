/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#ifndef __APP_QTIAMR_MOTION_TASK_H
#define __APP_QTIAMR_MOTION_TASK_H
/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include "canopen.h"
#include "motor_driver.h"


#ifdef CONFIG_APP_QTIAMR

/* motion task running frequency */
#define CONTROL_FREQUENCY    (50)


#define PI_MATH                   (3.1416f)

#define MAX_SPEED                 0.7         //(5 * QTIAMR1_WHEEL_PERIMETER)
#define MAX_ANGULAR_VELOCITY      2
#define SMOOTH_STEP               (0.03)      // Max speed : Max accelerate = 1:1.5



enum position_sub_mode
{
	POS_SUB_DISTANCE,
	POS_SUB_ANGLE,
};

struct motor_control_s
{
    int rpm_goal;
    int rpm_actual;
};

struct amr_motion_s
{
    struct motor_control_s motor_left;
    struct motor_control_s motor_right;

    float smooth_speed;
	uint8_t control_mode;
	bool velocity_zero;
	bool target_reached;
	uint8_t target_control_mode;
	bool control_mode_update;

	bool quick_stop_enable;
	bool motor_stop_once;
};

void smooth_speed_control(float vx, float step);
void get_motor_actual_speed(int *left, int *right);
void get_motor_goal_speed(int *left, int *right);

void inverse_kinematics(float vx, float vz, int *rpm_l, int *rpm_r);
void inverse_kinematics_pos(float vx, float vz, int sub_mode, int *count_l, int *count_r);
void get_motor_driver_velocity(float *vx, float *vz);
void rpm_transfer_to_odom(amr_motor_data_t *data);
int motion_task(int argc, char *argv[]);
amr_motor_data_t* get_motor_odom(void);
void quick_stop_status_set(bool enable);
bool motor_stop_status_get();

#endif
#endif
