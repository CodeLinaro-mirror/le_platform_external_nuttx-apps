/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/
#ifndef __INCLUDE_QTIAMR_MOTOR_DRIVER_H
#define __INCLUDE_QTIAMR_MOTOR_DRIVER_H

#include <nuttx/config.h>
#include <stdio.h>
#include <stdio.h>
#include <stdint.h>
#include <nuttx/fs/fs.h>

#define MOTOR_DRIVER_DEV  "/dev/can0"

#define STATUS_BIT_TARGET_REACHED ((1<<10)&0xFFFF)
#define STATUS_BIT_VELOCITY_ZERO  ((1<<12)&0xFFFF)

enum car_control_mode_
{
	CAR_VELOCITY_MODE,
	CAR_POSITION_MODE,
	CAR_MODE_NUM,
};

enum dri_om              //Driver operating mode
{
	DRI_NOT_DEFINED   = 0x00,  //Not defined
	DRI_POSITION_MODE = 0x01,  //Position mode
	DRI_VELOCITY_MODE = 0x03,  //Velocity mode
	DRI_TORQUE_MODE   = 0x04,  //Torque mode
};

enum driver_bus_mode
{
	BUS_CANOPEN = 0x0,
	BUS_RS485,
};

struct driver_error_data
{
	uint8_t left_motor_l;
	uint8_t left_motor_h;
	uint8_t right_motor_l;
	uint8_t right_motor_h;
	uint8_t nused[4];
};

struct driver_sdo_data
{
	uint8_t sdo_cmd;
	uint8_t index_l;
	uint8_t index_h;
	uint8_t sub_index;
	uint8_t data1_l;
	uint8_t data1_h;
	uint8_t data2_l;
	uint8_t data2_h;
};

struct motor_driver_s
{
	bool initialized;
	int mc_fd;
	uint8_t bus_mode;
	struct driver_error_data error_data;
};

typedef struct amr_motor_data
{
	int16_t left_rpm;
	int16_t right_rpm;
	float vx;
	float vy;
	float vz;
	int left_counts;
	int right_counts;
	float px;
	float py;
	float pz;
	float px_last;
	float py_last;
	float pz_last;
	bool mode_switch;
	bool target_reached;
	struct timespec tp;
}amr_motor_data_t;

int motor_driver_init(void);
void motor_driver_deinit(void);
void driver_set_motor_speed(int left_speed_rpm, int right_speed_rpm);
void motor_speed_sync(int left_speed_rpm, int right_speed_rpm, uint32_t time);
int motor_speed_read(amr_motor_data_t *data);
void motor_position_set(int left_rpm, int right_rpm);
bool motor_velocity_zero();
bool motor_target_reached();
int motor_control_mode_switch(uint8_t mode);
void motor_position_read(amr_motor_data_t *data);
void motor_quick_stop();
void motor_resume_enable(uint8_t mode);

#endif
