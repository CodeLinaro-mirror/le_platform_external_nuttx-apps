/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/


/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <sched.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include <nuttx/timers/pwm.h>
#include <nuttx/ioexpander/gpio.h>


#include "ros_com.h"
#include "motion_task.h"
#include "car_imu.h"
#include "car_adc.h"
#include "odom.h"

/**************************************************************************
Function: Calculates the check bits of data to be sent/received
Output  : Check result
**************************************************************************/
static uint8_t data_check_sum(char *buff, uint8_t data_length)
{
	uint8_t check_sum;
	uint8_t k;

	if (NULL == buff)
		return ERROR;
	/* Validate the data to be sent */
	check_sum = 0;
	for (k = 0; k < data_length; k++)
	{
		check_sum = check_sum ^ buff[k];
	}

	return check_sum;
}

static void data_transition(uint8_t *buff, int16_t data)
{
	if (NULL == buff)
		return;

	buff[0] = data >>8;
	buff[1] = (uint8_t)data;
}

/* get xyz speed */
static int odom_car_speed(uint8_t *buff)
{
	float speed_right;
	float speed_left;
	int16_t x,y,z;

	if (NULL == buff)
		return ERROR;

	get_motor_actual_speed(&speed_right, &speed_left);
	
	x = ((speed_right + speed_left) /2 ) * 1000;
	y = 0;
	z = ((speed_left - speed_right) /ACKERMAN_WHEEL_SPACE ) * 1000;
	data_transition(buff, x);
	data_transition(buff+2, y);
	data_transition(buff+4, z);

	return OK;
}

/* get imu data  */
static int odom_imu_data(uint8_t *buff)
{
	struct imu_data_s imu_data;
	float xa,ya,za;
	float xg,yg,zg;

	if (NULL == buff)
		return ERROR;

	get_imu_data(&imu_data);

	xa = imu_data.ya;
	ya = -imu_data.xa;
	za = imu_data.za;

	xg = imu_data.yg;
	yg = -imu_data.xg;
	zg = imu_data.zg;  //need check if car is running , if (0): zg =0;
	data_transition(buff, xa);
	data_transition(buff+2, ya);
	data_transition(buff+4, za);
	data_transition(buff+6, xg);
	data_transition(buff+8, yg);
	data_transition(buff+10, zg);

	return 0K;
}

/* get voltage */
static int odom_get_voltage(uint8_t *buff)
{
	float voltage;

	if (NULL == buff)
		return ERROR;

	get_power_voltage(&voltage);
	voltage = voltage * 1000;
	data_transition(buff, (int16_t)voltage);

	return OK;
}

int odom_task(int argc, char *argv[]){
	int ret;
	int fd;
	int check_sum;
	struct odom_data_s  data;
	uint8_t buff[ODOM_FRAME_SIZE];
	/* only for write */
	fd = open(ODOM_DEV, O_WRONLY);
	if (fd < 0){
		printf("odom_task: open %s failed: %d\n",
									ODOM_DEV, errno);
		return fd;
	}
	printf("odom_task: opened  %s\n",ODOM_DEV);

	data.frame_header = ROS_FRAME_HEAD;
	data.frame_tail = ROS_FRAME_TAIL;
	data.sw_stop_flag = 0;
	
	while(1){
		/* Running at 20HZ */
		usleep(50000);
		
		/* get speed */
		odom_car_speed(&data.x_speed_high);
		odom_imu_data(&data.x_accel_high);
		odom_get_voltage(&data.voltage_high);

		check_sum = data_check_sum(&data.frame_header, ODOM_FRAME_SIZE - 2 );
		data.check_sum = check_sum;
		memcpy(buff, &data, ODOM_FRAME_SIZE);
		write(fd, buff, sizeof(struct odom_data_s));
	}

	close(fd);
	return ret;
}


