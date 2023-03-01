/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/


#ifndef __APP_ACKERMAN_ODOM_H
#define __APP_ACKERMAN_ODOM_H
/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <stdio.h>
#include <stdint.h>

#ifdef CONFIG_APP_ACKERMAN

#define CAR_ODOM_PRIORITY	(200)
#define CAR_ODOM_STACKSIZE  (2048)

/* ttyS0 is console for debug better;tty1 for odom template, actual is ttyS2(usart3)  */
#define ODOM_DEV  "/dev/ttyS1"

#define ODOM_FRAME_SIZE (24)

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

int odom_task(int argc, char *argv[]);
#endif
#endif