/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#ifndef __APP_ACKERMAN_REMOTE_CONTROL_H
#define __APP_ACKERMAN_REMOTE_CONTROL_H
/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <stdio.h>
#include <stdint.h>

#ifdef CONFIG_APP_ACKERMAN

#include "car_pwm.h"
#include "car_encoder.h"


#define CAR_RC_PRIORITY  200
#define CAR_RC_STACKSIZE  (4*2048)

#define  RC_ANGLE_DEV "/dev/capture0"
#define  RC_SPEED_DEV "/dev/capture1"

#define RC_MIDDLE_VALUE     (75)   /* rc default value is 75  */
#define RC_HAVE_VALUE       (85)   /* check rc if have data */
#define RC_MAX_VALUE        (100)   
#define RC_MIN_VALUE        (50)  


#define RC_SAMPLE_TIME      (10)  /* ms */


struct rc_data_s
{
	int		speed_fd;
    int		angle_fd;
    uint8_t angle_duty;
    uint8_t speed_duty;
    float speed_x;
    float speed_z;
    uint32_t sample_time;  /* sampling interval/ms */
    bool rc_attached;      /* RC mode  */
    bool rc_data_ready;     /* false: data not ready; true: rc_data ready */
};


int rc_task(int argc, char *argv[]);
int get_rc_goal_speed(float* vx, float* vz);


#endif
#endif