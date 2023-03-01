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

#include "motion_task.h"


static float vz_to_akm_angle(float vx, float vz){
	float r_turning;
	float angle_turning;
	if (vx != 0 && vz != 0){
		/* check if r_turning less than minmiun turn radius */
		if (fabs(vx/vz) <= SENIOR_AKM_MIN_TURN_RADIUS){
			if (vz > 0)
				vz = fabs(vx) /(SENIOR_AKM_MIN_TURN_RADIUS);
			else
				vz = -fabs(vx) /(SENIOR_AKM_MIN_TURN_RADIUS);
		}
		r_turning = vx/vz;
		angle_turning = atan(ACKERMAN_AXLES_SPACE /(r_turning + 0.5f * ACKERMAN_WHEEL_SPACE));
	}
	else
		angle_turning = 0;

	return angle_turning;
}

void inverse_kinematics(float vx, float vz,
						float *vl, float *vr,
						struct servo_control_s *servo_cntl, bool rc_control){
	float radius_turning;
	float angle_turning;
	float temp;
	float vright_goal;
	float vleft_goal;
	float servo_goal;

	/* ackerman car need turning radius */
	if (rc_control) {
		/* for rc, vz is angle  */
		angle_turning = vz;
	}
	else {
		angle_turning = vz_to_akm_angle(vx,vz);
	}

	/*Make sure  Angle limit value to fit R=0.75 SENIOR_AKM_MIN_TURN_RADIUS*/
    angle_turning = AMP_LIMIT(angle_turning, -0.52f, 0.52f);
	radius_turning = ACKERMAN_AXLES_SPACE / tan(angle_turning) - 0.5f * ACKERMAN_WHEEL_SPACE;

	/* Inverse kinematics */
	if (fabs(angle_turning) < 0.001) { //angle_turning is zero
		vright_goal = vx;
		vleft_goal = vx;
	}
	else {
		vright_goal = vx * (radius_turning - 0.5 * ACKERMAN_WHEEL_SPACE) / radius_turning;
        vleft_goal = vx * (radius_turning + 0.5 * ACKERMAN_WHEEL_SPACE) / radius_turning;
	}

	/* The PWM value of the servo controls the steering Angle of the front wheel */
	temp =  -0.2137 * pow(angle_turning, 2) + 1.439 * angle_turning + 0.009599;
	
	servo_goal = servo_cntl->angle_zero_pwm + servo_cntl->angle_bias_pwm + temp * Slide2PWM_RATIO;
	/* motor target speed limit */
	*vr = AMP_LIMIT(vright_goal, -MAX_SPEED, MAX_SPEED);
	*vl  = AMP_LIMIT(vleft_goal, -MAX_SPEED, MAX_SPEED);
	servo_cntl->angle_goal_pwm  = AMP_LIMIT(servo_goal, MIN_SERVO_PWM, MAX_SERVO_PWM);
}

