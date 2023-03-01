
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
#include <math.h>

#include "main.h"

/*****************************************************************************
* Function: differential car inverse kinematics 
* Description: With diff car mode, calculate motor rotating speed.
* Input: @vx: The linear velocity of the robot in the x-axis direction;
* @vz: The linear velocity of the robot in the z-axis direction;
* Output: @rpm_l,;eft motor speed ；@rpm_r,right motor speed. 
* Return: NULL
******************************************************************************/
void inverse_kinematics(float vx, float vz, int *rpm_l, int *rpm_r)
{
    float v_left, v_right;

    int res = ERROR;
	
    if((rpm_l == NULL) || (rpm_l == NULL))
	{
		printf("rpm error !\n");

		return res;
	}

    v_left  = vx - vz*180/PI_MATH * QTIAMR1_WHEEL_SPACE / 2.0f; 
    v_right = vx + vz*180/PI_MATH * QTIAMR1_WHEEL_SPACE / 2.0f;

    /* motor target speed limit */
    v_left  = AMP_LIMIT(v_left, -MAX_SPEED, MAX_SPEED);
    v_right = AMP_LIMIT(v_right, -MAX_SPEED, MAX_SPEED);

    *rpm_l   = (int)(-v_left * 60 / QTIAMR1_WHEEL_PERIMETER);
    *rpm_r   = (int)(v_right * 60 / QTIAMR1_WHEEL_PERIMETER);

    //printf("vx %f,vz %f; v_l %f, v_r %f; rpm_l %d, rpm_r %d\n", vx, vz, v_left, v_right, *rpm_l, *rpm_r);


	return;
}
