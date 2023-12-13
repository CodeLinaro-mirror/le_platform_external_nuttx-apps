/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#ifndef __APP_QTIAMR_MAIN_H
#define __APP_QTIAMR_MAIN_H
/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <stdio.h>
#include <stdint.h>

#ifdef CONFIG_APP_QTIAMR

#include "rc.h"
#include "motion_task.h"

#define AMP_LIMIT(_val_, _min_, _max_)  \
	((_val_) < (_min_) ?  (_min_) : \
	((_val_) > (_max_) ? (_max_) : (_val_)))

#endif
#endif



#ifndef __AMR_TASK_CONFIG_H
#define __AMR_TASK_CONFIG_H


#define MOTION_PRIORITY   110
#define MOTION_STACKSIZE  (16*2048)



#define ROS_COM_PRIORITY  110
#define ROS_COM_STACKSIZE  (4*2048)

#define CAR_ULTRASOUND_PRIORITY   200
#define CAR_ULTRASOUND_STACKSIZE  (4*2048)

#define CAR_RC_PRIORITY   110
#define CAR_RC_STACKSIZE  (4*2048)

#define CAR_AUTO_CHARGING_PRIORITY   110
#define CAR_AUTO_CHARGING_STACKSIZE  (4*2048)


struct task_signal_info
{
    pid_t task_id; /* Id of the task */
    int   signal;  /* Signal that triggers the start */
};

enum task_list
{
    TASK_RC,
	TASK_MOTION,
	TASK_ROS_COM,
	TASK_ULTRASOUND,
	TASK_AUTO_CHARGE,
	TASK_NUM,
};

enum car_type_lsit
{
	CAR_TYPE_AMR_SMALL,
	CAR_TYPE_AMR_BIG,
	CAR_TYPE_NUM,
};

struct config_parameter_data
{
	uint8_t car_type;
	float vx_max;
	float vx_max_pos;
	float vz_max;
	float vz_max_pos;
	bool rc_enable;

};

extern volatile struct config_parameter_data parameter_data;

#endif
