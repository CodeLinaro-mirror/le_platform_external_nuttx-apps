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

#ifdef CONFIG_APP_QCOMAMR

#define MOTION_PRIORITY   110
#define MOTION_STACKSIZE  (16*2048)

/* motion task running frequency */
#define CONTROL_FREQUENCY    (100)

/* qcomamr parameters */
#define QTIAMR1_WHEEL_SPACE       (0.250f)
#define QTIAMR2_WHEEL_SPACE       (0.3302f)
#define QTIAMR1_WHEEL_DIAMETER    (0.131f)

#define PI_MATH                   (3.1416f)
#define QTIAMR1_WHEEL_PERIMETER   (QTIAMR1_WHEEL_DIAMETER * PI_MATH)

#define MAX_SPEED                 0.5         //(5 * QTIAMR1_WHEEL_PERIMETER)
#define SMOOTH_STEP               (0.03)      // Max speed : Max accelerate = 1:1.5

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
};

void smooth_speed_control(float vx, float step);
void get_motor_actual_speed(int *left, int *right);
void get_motor_goal_speed(int *left, int *right);

void inverse_kinematics(float vx, float vz, int *rpm_l, int *rpm_r);
int motion_task(int argc, char *argv[]);

#endif
#endif
