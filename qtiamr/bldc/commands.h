/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/


#ifndef __APP_QTIAMR_COMMANDS_H
#define __APP_QTIAMR_COMMANDS_H
/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <stdio.h>
#include <stdint.h>

#ifdef CONFIG_APP_QTIAMR

#define RS485    /* Controller RS485 bus */
#define RS485_MSG_LEN (16) /* 8 bytes message len & 8 for extern */
#define RS485_RECEIVE_TIME_MS (2000)

#define MULTI_REG_LEN	(2)	/* 最多连续写两个16bits 的register */
#define SINGLE_FRAME_LEN (6)	/* No CRC frame*/
#define SINGLE_FRAME_LEN (8)	/* No CRC frame*/

#define CONTROLLER_ADDR   (0x01)
#define R_MULTI_REG		(0x03)
#define W_MULTI_REG		(0x10)
#define W_SINGLE_REG	(0x06)

/* MC register (int16_t) */
#define MCREG_CONTROL_MODE		(0X200E)	/* 0x08: enable，0x07:disable，0x06:warning release ， 0x05: emergency stop */
#define CONTROL_MODE_EN			(0X08)
#define CONTROL_MODE_DISABLE	(0X07)
#define CONTROL_MODE_CLEAR_WARN	(0X06)
#define CONTROL_MODE_STOP		(0X05)

#define MCREG_RUNNING_MODE		(0X200D)	/* 0x3:speed mode，0x4:  topque mode，0x1&0x2: location mode */
#define RUNNING_MODE_SPEED		(0X03)


#define MCREG_LEFT_GET_SPEED	(0X20AB)	/* 0.1r/min */
#define MCREG_RIGHT_GET_SPEED	(0X20AC)
#define MCREG_LEFT_SET_SPEED	(0X2088)	/* -3000~3000r/min*/
#define MCREG_RIGHT_SET_SPEED	(0X2089)


#define READ_SPEED_UNIT		(0.1)	/* r/min */


#define RETURN_RAME_WRITE_SIZE	(8)	/* Write a single register to return the frame length */
#define RETURN_RAME_READ_SINGLE_SIZE	(7)	/* Reading a single register returns the frame length */
#define RETURN_RAME_READ_MULTI_SIZE		(9)	/* Reading a single register returns the frame length */


enum mc_err_code {
	ERR_FUNCTION_CODE = 0x01,
	ERR_DATA_ADDR,
	ERR_DATA_VALUE,
};

struct mc_cmd_buffer_s {
	char 		mc_addr;
	char 		mc_cmd_id;
	int16_t 	mc_register;
	union {
		int16_t		mc_reg_len;  //read  frame or write multi reg return frame. 
		int16_t		reg_data;	 //write single frame
	};
	uint16_t	crc;
};

/* Read multiple registers return frame data structure */
struct mc_cmd_multi_buffer_read_s {
	char 		mc_addr;
	char 		mc_cmd_id;
	int16_t		mc_reg_len;
	int16_t 	reg_data[MULTI_REG_LEN];
	uint16_t	crc;
};

/* Write multiple register frame data structure */
struct mc_cmd_multi_buffer_write_s {
	char 		mc_addr;
	char 		mc_cmd_id;
	int16_t		mc_register;
	char		mc_reg_len;
	int16_t 	reg_data[MULTI_REG_LEN];
	uint16_t	crc;
};

struct motor_controller_cmd_s {
	const char  *name;
	struct mc_cmd_buffer_s command;
};

enum mc_cmd_e {
	MC_SET_SPEED_MODE = 0,		/*	speed control mode	*/

};



int mc_init(int fd);
int mc_set_speed_100(int fd);
void mc_disable(int fd);

int mc_set_speed(int16_t speed_left, int16_t speed_right, int fd);


#endif
#endif
