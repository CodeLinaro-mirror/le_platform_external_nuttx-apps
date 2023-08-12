/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#ifndef __APP_ACKERMAN_ROS_COM_H
#define __APP_ACKERMAN_ROS_COM_H
/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <stdio.h>
#include <stdint.h>

#ifdef CONFIG_APP_ACKERMAN

#include "car_pwm.h"
#include "car_encoder.h"


#define CAR_ROS_COM_PRIORITY  200
#define CAR_ROS_COM_STACKSIZE  (4*2048)

#define ROS_COM_DEV    "/dev/ttyS1"   /* ttyS0 is console for debug better, actual is ttyS2(usart3)  */

#define RECEIVE_BUFF_SIZE    11     /*  fixed received data size  */

#define ROS_FRAME_HEAD    0X7B //Frame_header
#define ROS_FRAME_TAIL	  0X7D //Frame_tail
#define ROS_FRAME_CMD     0xFD  // Frame for command

/** serial data structure  **/
struct ros_send_data_s
{
	unsigned char frame_header; /* 1 byte */
	short X_speed;	            /* 2 bytes */
	short Y_speed;              /* 2 bytes */
	short Z_speed;              /* 2 bytes */
	short power_voltage;        /* 2 bytes */
	//Mpu6050_Data Accelerometer; /* 6 bytes */
	//Mpu6050_Data Gyroscope;     /* 6 bytes */ 
	unsigned char frame_tail;   /* 1 bytes */
};

struct ros_receive_data_s 
{
	uint8_t frame_header; //1 byte
	uint8_t cmd;       //1 byte
	uint8_t reserve;      //1 byte
	uint8_t x_high;       //1 byte
	uint8_t x_low;        //1 byte
	uint8_t y_high;       //1 byte
	uint8_t y_low;        //1 byte
	uint8_t z_high;       //1 byte
	uint8_t z_low;        //1 byte
	uint8_t check_sum;    //1 byte
	uint8_t frame_tail;   //1 bytes 
};


/** ROS communication structure **/
enum ros_command{
	CMD_SPEED_CONFIGURE = 0,
	CMD_ANGLE_CONFIGURE,
	CMD_PID,
	CMD_DEBUG,
};

struct car_ros_com_s
{
	bool	initialized;
	int		ros_fd;
	struct ros_receive_data_s 	receive_data;
	uint8_t current_receive_len;
	union {
		struct {
			float  x_speed;
			float  z_speed;
		};
		struct {
			float  kp;
			float  ki;
		};
	}parse_data;
	uint8_t current_send_len;
	bool	tx_ready;
	bool	speed_ready;
	bool	pid_ready;	

};

int ros_com_task(int argc, char *argv[]);
int get_ros_goal_speed(float* vx, float* vz);
int get_ros_pid(float* kp, float* ki);
#endif
#endif