/***************************************************************************
* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
****************************************************************************/
#ifndef __APP_QTIAMR_ROS_COM_H
#define __APP_QTIAMR_ROS_COM_H

#include <stdio.h>
#include <stdint.h>
#include "motor_driver.h"

#ifdef CONFIG_APP_QTIAMR

/* roscom task running frequency */
#define ROSCOM_FREQUENCY	(50)
#define ROS_CONNECT_TIMEOUT	(6)


#define ROS_COM_DEV	"/dev/ttyS2"

#define RECEIVE_BUFF_SIZE	11     /*  fixed received data size  */

#define STANDARD_FRAME_LEN	(sizeof(struct standard_frame_s))
#define EXTEND_FRAME_LEN	(sizeof(struct extend_frame_s))
#define STANDARD_FRAME_HEADER	(0X7B)
#define STANDARD_FRAME_TRAILER	(0X7D)
#define EXTEND_FRAME_HEADER	(0XFB)
#define EXTEND_FRAME_TRAILER	(0XFD)

#define ROS_FRAME_HEAD	0x7b //Frame_header
#define ROS_FRAME_TAIL	0x7d //Frame_tail

enum parameter_download_state
{
	PAR_COM_INIT,
	PAR_DOWN_LOAD,
	PAR_LOAD_DONE,
};

enum ros_ack_code
{
	ACK_SUCCESS,
	ACK_FAILED,
	ACK_OUT_RANGE,
	ACK_REPEAT,
	ACK_STATUS_ERR,
};



/** qrc data structure  **/
#pragma pack(1)
struct ros_receive_data_s
{
	uint8_t frame_header;
	uint8_t cmd;
	uint8_t reserved;
	uint8_t data1;
	uint8_t data2;
	uint8_t data3;
	uint8_t data4;
	uint8_t data5;
	uint8_t data6;
	uint8_t check_sum;
	uint8_t frame_tail;
};



struct odom_data_s
{
	uint8_t frame_header;		//1 byte
	uint8_t sw_stop_flag;		//1 byte

	uint8_t x_speed_high;		//1 byte
	uint8_t x_speed_low;		//1 byte
	uint8_t y_speed_high;		//1 byte
	uint8_t y_speed_low;		//1 byte
	uint8_t z_speed_high;		//1 byte
	uint8_t z_speed_low;		//1 byte

	/* 8~13: acceleration */
	uint8_t x_accel_high;		//1 byte
	uint8_t x_accel_low;		//1 byte
	uint8_t y_accel_high;		//1 byte
	uint8_t y_accel_low;		//1 byte
	uint8_t z_accel_high;		//1 byte
	uint8_t z_accel_low;		//1 byte
	/* 14~19 gyro */
	uint8_t x_gyro_high;		//1 byte
	uint8_t x_gyro_low;			//1 byte
	uint8_t y_gyro_high;		//1 byte
	uint8_t y_gyro_low;			//1 byte
	uint8_t z_gyro_high;		//1 byte
	uint8_t z_gyro_low;			//1 byte

	uint8_t voltage_high;		//1 byte
	uint8_t voltage_low;		//1 byte
	uint8_t check_sum;			//1 byte
	uint8_t frame_tail;			//1 bytes
};

/** ROS communication structure **/
enum ros_command
{
	CMD_SPEED_CONTROL = 0x01,
	CMD_TIME_LOOP,
	CMD_TIME_SET,
	CMD_TIME_SYNC_REQUEST,
    CMD_PARAMETER_DOWNLOAD,
    CMD_CONTROL_MODE_SWITCH,
    CMD_POSITION_SET,
    CMD_ANGLE_SET,
    CMD_QUICK_STOP_ENABLE,
	CMD_TIME_GET = 0x41,
	CMD_CHARG_STATUS,
	CMD_EXCEPTION_STATUS,
	CMD_CAR_TYPE_GET,
	CMD_RESET_STATUS,
	CMD_CONTROL_MODE_STATUS,
	CMD_IMU = 0xF1,
	CMD_ODOM,
	CMD_POSITION_ODOM,
};

enum ros_subcommand
{
	CMD_CAR_TYPE_SET,
	CMD_MAX_VX_SET,
	CMD_MAX_VZ_SET,
	CMD_POS_MAX_VX_SET,
	CMD_POS_MAX_VZ_SET,
	CMD_ENABLE_RC,
	CMD_PARAMETER_DONE = 0xFF,

};

struct amr_ros_com_s
{
	bool	initialized;
	int		ros_fd;
	struct ros_receive_data_s 	ros_receive_data;
	struct odom_data_s odom_data;
	uint8_t current_receive_len;
	union {
		struct {
			float  x_speed;
			float  z_speed;
		};
		struct {
			float  x;
			float  z;
		};
	}parse_data;
	uint8_t current_send_len;
	bool	tx_ready;
	bool	speed_ready;
	bool	pid_ready;
	bool    velocity_enable;
	bool    position_enable;
	bool    position_update;
	uint8_t pos_sub_mode;
	bool    notify_mode_switch;
	struct timespec tp_connect;
	bool    connected;

};

struct standard_frame_s {
	uint8_t frame_header;
	uint8_t cmd;
	uint8_t reserved;
	uint8_t data1;
	uint8_t data2;
	uint8_t data3;
	uint8_t data4;
	uint8_t data5;
	uint8_t data6;
	uint8_t check_sum;
	uint8_t frame_tail;
};

struct extend_frame_s {
	uint8_t frame_header;
	uint8_t cmd;
	uint8_t data1;
	uint8_t data2;
	uint8_t data3;
	uint8_t data4;
	uint8_t data5;
	uint8_t data6;
	uint8_t data7;
	uint8_t data8;
	uint8_t data9;
	uint8_t data10;
	uint8_t data11;
	uint8_t data12;
	uint8_t data13;
	uint8_t data14;
	uint8_t data15;
	uint8_t data16;
	uint8_t data17;
	uint8_t data18;
	uint8_t check_sum;
	uint8_t frame_tail;
};

int ros_com_task(int argc, char *argv[]);
int get_ros_goal_speed(float* vx, float* vz);
int publish_odom_to_ros(void);
float get_max_vx(void);
float get_max_vz(void);
uint8_t get_car_type(void);

#endif
#endif
