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



/* Define motor driver control command, the value is obtained through the upper layer*/
struct driver_command_s
{
	uint16_t mdc_debug_read;
    uint16_t mdc_async_set;
    uint16_t mdc_sync_set;
    uint16_t mdc_enable_set;
    uint16_t mdc_velocity_mode_set;
    uint16_t mdc_left_speed_set;
	uint16_t mdc_right_speed_set;
	uint16_t mdc_left_acc_set;
	uint16_t mdc_right_acc_set;
	uint16_t mdc_left_dec_set;
	uint16_t mdc_right_dec_set;
	uint16_t mdc_speed_sync_mode_set;
	uint16_t mdc_speed_read;
	uint16_t mdc_left_position_read;
	uint16_t mdc_right_position_read;	
};


enum dri_om              //Driver operating mode
{
    NOT_DEFINED   = 0x00,  //Not defined
    POSITION_MODE = 0x01,  //Position mode
    VELOCITY_MODE = 0x03,  //Velocity mode
    TORQUE_MODE   = 0x04,  //Torque mode
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

int motor_driver_init(void);
void motor_driver_deinit(void);
void driver_set_motor_speed(int left_speed_rpm, int right_speed_rpm);
void motor_speed_sync(int left_speed_rpm, int right_speed_rpm);
void motor_speed_read(int * left_speed_rpm, int * right_speed_rpm);
void motor_position_read(int *left_counts, int *right_counts);

#endif
