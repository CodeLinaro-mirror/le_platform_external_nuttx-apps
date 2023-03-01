
/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/


#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <string.h>
#include <inttypes.h>
#include <syslog.h>

#include <nuttx/arch.h>

#include "motion_task.h"
#include "motor_driver.h"
#include "main.h"

static struct amr_motion_s g_amr_motion;


/*****************************************************************************
* Function: Motor speed smooth control. 
* Description: The smooth control of the motor speed solves the vibration
* caused by too much acceleration when the robot is under control.
* Input: @vx, motor speed; @step,control smooth coefficient, the smaller the 
* value, the smoother.
* Return: CRC value
******************************************************************************/
void smooth_speed_control(float vx, float step)
{
    if (vx > g_amr_motion.smooth_speed)
    {
        g_amr_motion.smooth_speed += step;    
    }
        
    else if (vx < g_amr_motion.smooth_speed)
    {
        g_amr_motion.smooth_speed -= step;
    }
        
    else
    {
        g_amr_motion.smooth_speed = vx;
    }
        
    if ((abs(vx) <0.1) && g_amr_motion.smooth_speed < (2.5 * step) && g_amr_motion.smooth_speed > (-2.5 * step))
    {
        g_amr_motion.smooth_speed = vx;
    }       
}

void get_motor_actual_speed(int *left, int *right)
{
    *left = g_amr_motion.motor_left.rpm_actual;
    *right = g_amr_motion.motor_right.rpm_actual;
}

void get_motor_goal_speed(int *left, int *right)
{
    *left = g_amr_motion.motor_left.rpm_goal;
    *right = g_amr_motion.motor_right.rpm_goal;
}

/*****************************************************************************
* Function: Motor speed control task. 
* Description: Carry out kinematics analysis through the incoming robot speed
* and issue speed commands to the driver.
* value, the smoother.
* Return: ERROR NUMBER
******************************************************************************/
int motion_task(int argc, char *argv[])
{
    struct motor_control_s *motor_left;
    struct motor_control_s *motor_right;

    float vx_rc, vz_rc;

    int rpm_l = 0, rpm_r = 0;
	int count_l = 0, count_r = 0;

	int i = 0;
	
    motor_left = &g_amr_motion.motor_left;
    motor_right = &g_amr_motion.motor_right;
    
    motor_driver_init();
    printf("motion task start running! \n");
    
    while (1)
    {
        usleep(100000);

        if(OK == get_rc_goal_speed(&vx_rc, &vz_rc))
        {
	    //smooth_speed_control(vx_rc, SMOOTH_STEP);
            inverse_kinematics(vx_rc, vz_rc, &motor_left->rpm_goal, &motor_right->rpm_goal);
            //driver_set_motor_speed(motor_left->rpm_goal, motor_right->rpm_goal);
	    motor_speed_sync(motor_left->rpm_goal, motor_right->rpm_goal); 
        }

        if (i++%20 == 0)
        {
        	printf("vx %.2f, vz %.2f; rpm_l %d, rpm_r %d\n", vx_rc, vz_rc, motor_left->rpm_goal, motor_right->rpm_goal);
			motor_speed_read(&rpm_l, &rpm_r);
        }
		else if (i%10 == 0)
        {
			motor_position_read(&count_l, &count_r);
        }

    }

	return 0;
}
