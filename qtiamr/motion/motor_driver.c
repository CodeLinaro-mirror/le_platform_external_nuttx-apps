/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/


#include <fcntl.h>
#include <stdio.h>
#include <inttypes.h>
#include <nuttx/can/can.h>

#include "canopen.h"
#include "motor_driver.h"

static struct motor_driver_s g_motor_driver;
static struct driver_command_s  mdc;

#define CAN_STD_SIZE 8

struct mc_cmd_s {
	const char  *name;
	struct driver_sdo_data command;
};

enum mc_control_code_e {
	CODE_DEBUG_CC = 0,
	CODE_CAN_ASYNC,
	CODE_CAN_SYNC,
	CODE_CAN_ENABLE_1,
	CODE_CAN_ENABLE_2,
	CODE_CAN_ENABLE_3,
	CODE_SPEED_MODE,
	CODE_SET_SPEED_LEFT,
	CODE_SET_SPEED_RIGHT,
	CODE_SET_SPEED_LEFT_ACC,
	CODE_SET_SPEED_RIGHT_ACC,
	CODE_SET_SPEED_LEFT_DEC,
	CODE_SET_SPEED_RIGHT_DEC,
	CODE_SYNC_SPEED,
	CODE_SPEED_READ,
	CODE_LEFT_COUNT,
	CODE_RIGHT_COUNT,
};


/* Read/write frame */
const struct mc_cmd_s g_command_list[] = {
	{{"debug"},{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	{{"can_async_set"},{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{{"can_sync_set"},{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{{"can_enable_1"},{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{{"can_enable_2"},{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{{"can_enable_3"},{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{{"can_velocity_set"},{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{{"set_speed_left_rpm"},{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{{"set_speed_right_rpm"}, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{{"set_speed_left_acc"},{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{{"set_speed_right_acc"}, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{{"set_speed_left_dec"},{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{{"set_speed_right_dec"}, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},	
	{{"speed_sync"}, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{{"speed_read"}, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{{"position_left"}, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{{"position_right"}, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
};

void zlac8015d_driver_error_check(void)
{
        struct driver_sdo_data can_data;
        size_t data_len = sizeof(can_data);
        int fd = g_motor_driver.mc_fd;
        char buffer[CAN_STD_SIZE];

        if (g_motor_driver.initialized != true)
    {
                printf("motor_driver not init \n");
                return;
        }

        //send err code register
        printf("CODE_DEBUG_CC send debug request \n");
        can_data = g_command_list[CODE_DEBUG_CC].command;
        canopen_send(fd, &can_data, data_len);
        
	printf("CODE_DEBUG_CC read debug info \n");
	usleep(50000);
	canopen_receive(fd, buffer, CAN_STD_SIZE);

        if(buffer[4] == 0x01)
        {
                printf("over voltage! \n");
        }

        switch(buffer[4])
        {
                case 0x01:
                    printf("over voltage! \n");
                        break;

                case 0x02:
                    printf("lack voltage! \n");
                        break;

                case 0x04:
                    printf("left motor over current! \n");
                        break;

                case 0x08:
                    printf("left motor overload! \n");
                        break;

                case 0x10:
                    printf("left motor over voltage! \n");
                        break;

                case 0x20:
                    printf("encoder over proof! \n");
                        break;

                case 0x40:
                    printf("speed over proof! \n");

                case 0x80:
                    printf("reference voltage error! \n");
                    break;

            default:
                break;
        }

        switch(buffer[5])
        {
                case 0x01:
                    printf("EEPROM read-write error! \n");
                        break;

                case 0x02:
                    printf("Hall sensor error! \n");
                        break;

                default:
                    break;
        }

        switch(buffer[6])
        {
                case 0x04:
                    printf("right motor over current! \n");
                        break;

                case 0x08:
                    printf("right motor over load! \n");
                        break;

                case 0x10:
                    printf("right motor over voltage! \n");
                        break;

                case 0x20:
                    printf("encoder over proof");
                        break;

                case 0x40:
                    printf("speed over proof! \n");
                    break;

                case 0x80:
                    printf("reference voltage error! \n");
                        break;

                default:
                    break;

        }

        if(buffer[7] == 0x02)
        {
                printf("hall sensor error! \n");
				
		}
		if((buffer[4] == 0) && (buffer[5] == 0) && (buffer[6] == 0) && (buffer[7] == 0))
        {
                printf("motor driver ok! \n");
        }
}

/*****************************************************************************
* Function: Set motor speed with canopen frame. 
* Description: Set motor speed to motor driver.
* Input: @left_speed_rpm,left motor speed; @right_speed_rpm,right motor speed.
* Return: ERROR NUMBER
******************************************************************************/
void driver_set_motor_speed(int left_speed_rpm, int right_speed_rpm)
{
	struct driver_sdo_data left_data,right_data;
	size_t data_len = sizeof(left_data);
	int fd = g_motor_driver.mc_fd;

	left_data = g_command_list[CODE_SET_SPEED_LEFT].command;
	right_data = g_command_list[CODE_SET_SPEED_RIGHT].command;
	
	left_data.data1_l=left_speed_rpm&0xff;
	left_data.data1_h=left_speed_rpm>>8;
		
	right_data.data1_l=right_speed_rpm&0xff;
	right_data.data1_h=right_speed_rpm>>8;

	
	//处理速度值。
	printf("set speed \n");
	canopen_send(fd, &left_data, data_len);
	usleep(200000);
	canopen_send(fd, &right_data, data_len);
	usleep(200000);
}

/*****************************************************************************
* Function: Motor speed sync 
* Description: Send speed as sync mode.
* Input: @left_speed_rpm,left motor speed; @right_speed_rpm,right motor speed.
* Return: ERROR NUMBER
******************************************************************************/
void motor_speed_sync(int left_speed_rpm, int right_speed_rpm)
{
	struct driver_sdo_data motor_speed;
	int i = 0;
	size_t data_len = sizeof(motor_speed);
        int fd = g_motor_driver.mc_fd;
	char buffer[8]={0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};

	motor_speed = g_command_list[CODE_SYNC_SPEED].command;

	motor_speed.data1_l = left_speed_rpm&0xff;
	motor_speed.data1_h = left_speed_rpm>>8;

	motor_speed.data2_l = right_speed_rpm&0xff;
        motor_speed.data2_h = right_speed_rpm>>8;

    if (i++ % 10 == 0)
    {
		//printf("speed sync %d, %d \n", motor_speed.data1_h<<8|motor_speed.data1_l, motor_speed.data2_h<<8|motor_speed.data2_l);
    }
	canopen_send(fd, &motor_speed, data_len);
	usleep(50000);
	canopen_receive(fd, buffer, CAN_STD_SIZE);
}

/*****************************************************************************
* Function: Motor speed sync 
* Description: Send speed as sync mode.
* Input: @left_speed_rpm,left motor speed; @right_speed_rpm,right motor speed.
* Return: ERROR NUMBER
******************************************************************************/
void motor_speed_read(int *left_speed_rpm, int *right_speed_rpm)
{
	struct driver_sdo_data motor_speed = {0};
	size_t data_len = sizeof(motor_speed);
    int fd = g_motor_driver.mc_fd;
	int ret = 0;

	canopen_send(fd, &g_command_list[CODE_SPEED_READ].command, data_len);
	usleep(50000);

	ret = canopen_receive(fd, &motor_speed, CAN_STD_SIZE);

    if (ret > 0 && motor_speed.index_l ==  0x6C && motor_speed.index_h == 0x60)
    {
		printf("speed read rpm %d,%d \n", motor_speed.data1_h<<8|motor_speed.data1_l, motor_speed.data2_h<<8|motor_speed.data2_l);		
    }
}

/*****************************************************************************
* Function: Read motor position.
* Description: Read motor position from motor driver.
* Output: @left_counts,left motor position count; @right_counts,right motor position count.
* Return: ERROR NUMBER
******************************************************************************/
void motor_position_read(int *left_counts, int *right_counts)
{
	struct driver_sdo_data motor_position_left = {0};
	struct driver_sdo_data motor_position_right = {0};
	size_t data_len = sizeof(motor_position_left);
    int fd = g_motor_driver.mc_fd;
	int ret = 0;

	canopen_send(fd, &g_command_list[CODE_LEFT_COUNT].command, data_len);
	usleep(50000);

	ret = canopen_receive(fd, &motor_position_left, CAN_STD_SIZE);
	usleep(50000);

	canopen_send(fd, &g_command_list[CODE_RIGHT_COUNT].command, data_len);
	usleep(50000);

	ret = canopen_receive(fd, &motor_position_right, CAN_STD_SIZE);

    if (ret > 0 && motor_position_left.index_l ==  0x64 && motor_position_left.index_h == 0x60)
    {
		printf("position read conuts %d,%d \n", (int)motor_position_left.data1_h<<8|motor_position_left.data1_l|motor_position_left.data2_h<<24|motor_position_left.data2_l<<16,\
		(int)motor_position_right.data1_h<<8|motor_position_right.data1_l|motor_position_right.data2_h<<24|motor_position_right.data2_l<<16);		
    }	
}


static void zlac8015d_driver_init(void)
{
	struct driver_sdo_data can_data;
	size_t data_len = sizeof(can_data);
	int fd = g_motor_driver.mc_fd;
	char buffer[8]={0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
	
	if (g_motor_driver.initialized != true){
		printf("motor_driver not init \n");
		return;
	}

	//ASYNC_SET
	//printf("ASYNC_SET\n");
	//usleep(200000);
	//can_data = g_command_list[CODE_CAN_ASYNC].command;
        //canopen_send(fd, &can_data, data_len);
	//SYNC_SET
	printf("SYNC_SET\n");
	usleep(200000);
	can_data = g_command_list[CODE_CAN_SYNC].command;
	canopen_send(fd, &can_data, data_len);
	//ENABLE
	printf("ENABLE\n");
	usleep(2000);
	canopen_receive(fd, buffer, CAN_STD_SIZE);
	usleep(2000);
	printf("SPEED MODE\n");
        can_data = g_command_list[CODE_SPEED_MODE].command;
        canopen_send(fd, &can_data, data_len);
   	usleep(2000);
	canopen_receive(fd, buffer, CAN_STD_SIZE);
	usleep(2000);

	can_data = g_command_list[CODE_SET_SPEED_LEFT_ACC].command;
    canopen_send(fd, &can_data, data_len);
	usleep(2000);
	canopen_receive(fd, buffer, CAN_STD_SIZE);
	usleep(2000);


	can_data = g_command_list[CODE_SET_SPEED_RIGHT_ACC].command;
    canopen_send(fd, &can_data, data_len);
		usleep(2000);
	canopen_receive(fd, buffer, CAN_STD_SIZE);
	usleep(2000);

	can_data = g_command_list[CODE_SET_SPEED_LEFT_DEC].command;
    canopen_send(fd, &can_data, data_len);
		usleep(2000);
	canopen_receive(fd, buffer, CAN_STD_SIZE);
	usleep(2000);

	can_data = g_command_list[CODE_SET_SPEED_RIGHT_DEC].command;
    canopen_send(fd, &can_data, data_len);
		usleep(2000);
	canopen_receive(fd, buffer, CAN_STD_SIZE);
	usleep(2000);

	can_data = g_command_list[CODE_CAN_ENABLE_1].command;
    canopen_send(fd, &can_data, data_len);
		usleep(2000);
	canopen_receive(fd, buffer, CAN_STD_SIZE);
	usleep(2000);
	
	can_data = g_command_list[CODE_CAN_ENABLE_2].command;
    canopen_send(fd, &can_data, data_len);
		usleep(2000);
	canopen_receive(fd, buffer, CAN_STD_SIZE);
	usleep(2000);
	
	can_data = g_command_list[CODE_CAN_ENABLE_3].command;
    canopen_send(fd, &can_data, data_len);
		usleep(2000);
	canopen_receive(fd, buffer, CAN_STD_SIZE);
	usleep(2000);
	//SPEED MODE
	//printf("SPEED MODE\n");
	//can_data = g_command_list[CODE_SPEED_MODE].command;
        //canopen_send(fd, &can_data, data_len);
	//usleep(200000);
}

int motor_driver_init(void)
{
	int fd;
	//open fd
	fd = open(MOTOR_DRIVER_DEV, O_RDWR);
    if (fd < 0)
    {
        printf("ERROR: open %s failed: %d\n", MOTOR_DRIVER_DEV, errno);
		close(fd);
		return ERROR;
    }
	g_motor_driver.initialized = true;
	g_motor_driver.mc_fd = fd;
	g_motor_driver.bus_mode = BUS_CANOPEN;
	//init driver hardware
	zlac8015d_driver_init();
	return OK;
}

void motor_driver_deinit(void)
{
	//close fd
	close(g_motor_driver.mc_fd);
	
}







