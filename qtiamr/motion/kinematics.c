
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
#include "motor_driver.h"
#include "ros_com.h"


const float QTIAMR_WHEEL_SPACE_ARRAY[CAR_TYPE_NUM]     = {(0.250f), (0.3302f)};
const float QTIAMR_WHEEL_PERIMETER_ARRAY[CAR_TYPE_NUM] = {(0.334f), (0.4115f)};
const float LINEAR_SCALE_ARRAY[CAR_TYPE_NUM]           = {(0.98),   (1.00)};
const float ANGLE_SCALE_ARRAY[CAR_TYPE_NUM]            = {(1.0083), (1.00)};
const float PRE_LINEAR_SCALE_ARRAY[CAR_TYPE_NUM]       = {(1.00), (1.0396)};
const float PRE_ANGLE_SCALE_ARRAY[CAR_TYPE_NUM]        = {(1.00), (1.0406)};

#define QTIAMR_WHEEL_SPACE(x)        QTIAMR_WHEEL_SPACE_ARRAY[x]
#define QTIAMR_WHEEL_PERIMETER(x)    QTIAMR_WHEEL_PERIMETER_ARRAY[x]
#define LINEAR_SCALE(x)              LINEAR_SCALE_ARRAY[x]
#define ANGLE_SCALE(x)               ANGLE_SCALE_ARRAY[x]
#define PRE_LINEAR_SCALE(x)          PRE_LINEAR_SCALE_ARRAY[x]
#define PRE_ANGLE_SCALE(x)           PRE_ANGLE_SCALE_ARRAY[x]

#define QTIAMR_WHEEL_POSITION_COUNT_CMD 40960
#define QTIAMR_WHEEL_POSITION_COUNT     4096


void inverse_kinematics(float vx, float vz, int *rpm_l, int *rpm_r)
{
	float v_left, v_right;
	int res = ERROR;
	float max_vx, max_vz;
	uint8_t car_type;

	if((rpm_l == NULL) || (rpm_l == NULL))
	{
		printf("rpm error !\n");
		return res;
	}

	car_type = get_car_type();
	/* scale to solve input error  */
	vx = vx * PRE_LINEAR_SCALE(car_type);
	vz = vz * PRE_ANGLE_SCALE(car_type);

	max_vx = get_max_vx();
	vx = AMP_LIMIT(vx, -max_vx, max_vx);

	max_vz = get_max_vz();
	vz = AMP_LIMIT(vz, -max_vz, max_vz);

	car_type = get_car_type();
	if (vx == 0) {
		v_right = (vz * QTIAMR_WHEEL_SPACE(car_type)/2.0);
		v_left = (-1)*v_right;
	} else if (vz ==0) {
		v_left = v_right = vx;
	} else {
		v_left  = vx - vz * QTIAMR_WHEEL_SPACE(car_type) / 2.0f;
		v_right = vx + vz * QTIAMR_WHEEL_SPACE(car_type) / 2.0f;
	}
	/* motor target speed limit */

	*rpm_l   = (int16_t)(v_left * 60 / QTIAMR_WHEEL_PERIMETER(car_type));
	*rpm_r   = (int16_t)(-v_right * 60 / QTIAMR_WHEEL_PERIMETER(car_type));

	//printf("car type %d,vx_max %.3f,vz_max %.3f,vx %.3f,vz %.3f;v_l %.3f,v_r %.3f;rpm_l %d,rpm_r %d;wheel=%.3f\n", parameter_data.car_type, max_vx, max_vz, vx, vz, v_left, v_right, *rpm_l, *rpm_r,QTIAMR_WHEEL_PERIMETER(parameter_data.car_type));

	return;
}

void inverse_kinematics_pos(float x, float z, int sub_mode, int *count_l, int *count_r)
{
	uint8_t car_type;

	car_type = get_car_type();

	if (count_l == NULL || count_r == NULL)
	{
		return;
	}

	if(sub_mode == POS_SUB_DISTANCE)
	{
		*count_l = (int)(x / QTIAMR_WHEEL_PERIMETER(car_type) * QTIAMR_WHEEL_POSITION_COUNT_CMD);
		*count_r = (int)(-x / QTIAMR_WHEEL_PERIMETER(car_type) * QTIAMR_WHEEL_POSITION_COUNT_CMD);
	}

	if(sub_mode == POS_SUB_ANGLE)
	{
		*count_l = (int)(z * QTIAMR_WHEEL_SPACE(car_type) / 2.0f / QTIAMR_WHEEL_PERIMETER(car_type) * QTIAMR_WHEEL_POSITION_COUNT_CMD);
		*count_r = (int)(z * QTIAMR_WHEEL_SPACE(car_type) / 2.0f / QTIAMR_WHEEL_PERIMETER(car_type) * QTIAMR_WHEEL_POSITION_COUNT_CMD);
	}
}

void count_transfer_to_odom(amr_motor_data_t *data)
{
	float p_left, p_right;
	uint8_t car_type;
	float px, pz;

	car_type = get_car_type();

	p_left  = (data->left_counts/QTIAMR_WHEEL_POSITION_COUNT * QTIAMR_WHEEL_PERIMETER(car_type));
	p_right = (- data->right_counts/QTIAMR_WHEEL_POSITION_COUNT * QTIAMR_WHEEL_PERIMETER(car_type));

	px = (p_left + p_right) / 2.0f;
	pz = (p_right - p_left) / QTIAMR_WHEEL_SPACE(car_type);

	if (data->mode_switch)
	{
		data->px_last = px;
		data->pz_last = pz;
		data->mode_switch = false;
	}

	data->px = px - data->px_last;
	data->pz = pz - data->pz_last;
	data->px_last = px;
	data->pz_last = pz;

}

void rpm_transfer_to_odom(amr_motor_data_t *data)
{
	float v_left, v_right;
	uint8_t car_type;

	car_type = get_car_type();

	v_left  = (data->left_rpm * QTIAMR_WHEEL_PERIMETER(car_type) / 60) * 0.1;
	v_right = (- data->right_rpm * QTIAMR_WHEEL_PERIMETER(car_type) / 60) * 0.1;

	data->vx = (v_left + v_right) / 2.0f;
	//data->vz = (v_right - v_left) / QTIAMR1_WHEEL_SPACE;

	data->vz = (v_right - v_left) / QTIAMR_WHEEL_SPACE(car_type);

	//filter
	if (abs(v_left) > 0.2 || abs(v_right) > 0.2 ){
		data->vx = data->vx*LINEAR_SCALE(car_type);
		data->vz = data->vz*ANGLE_SCALE(car_type);
	}

	//syslog(LOG_DEBUG, " read rpm:%d,%d vx:%.3f vz:%.3f\n", data->left_rpm, data->right_rpm, data->vx, data->vz);
}
