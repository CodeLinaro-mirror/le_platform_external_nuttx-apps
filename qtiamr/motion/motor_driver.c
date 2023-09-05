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
#include <syslog.h>

#include "canopen.h"
#include "motor_driver.h"
#include "ros_com.h"

static struct motor_driver_s g_motor_driver;
#define CAN_STD_SIZE 8

#define KI  (200)
#define KP  (400)
#define M_POLOE (10)


enum mc_control_code_e {
	CODE_DEBUG_CC = 0,
	CODE_MOTOR_STATUS,
	CODE_CAN_ASYNC,
	CODE_CAN_SYNC,
	CODE_CAN_ENABLE_1,
	CODE_CAN_ENABLE_2,
	CODE_CAN_ENABLE_3,
	CODE_QUICK_STOP,
	CODE_SPEED_MODE,
	CODE_POSITION_MODE,
	CODE_SET_POS_LEFT,
	CODE_SET_POS_RIGHT,
	CODE_SET_POS_REL_START_PREPARE,
	CODE_SET_POS_REL_START_RUN,
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
	CODE_LEFT_KP,
	CODE_RIGHT_KP,
	CODE_LEFT_KI,
	CODE_RIGHT_KI,
	CODE_LEFT_POLE,
	CODE_RIGHT_POLE,
	CODE_LEFT_ENCODER_RANGE,
	CODE_RIGHT_ENCODER_RANGE,
};

struct mc_cmd_s {
	enum mc_control_code_e cmd_enum;
	struct driver_sdo_data command;
};


/* read or write data frame */
const struct mc_cmd_s g_command_list[] = {
	{CODE_DEBUG_CC,                  {0x43, 0x3f, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{CODE_MOTOR_STATUS,              {0x43, 0x41, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{CODE_CAN_ASYNC,                 {0x2b, 0x0f, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{CODE_CAN_SYNC,                  {0x2b, 0x0f, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00}},
	{CODE_CAN_ENABLE_1,              {0x2b, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00}},
	{CODE_CAN_ENABLE_2,              {0x2b, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00}},
	{CODE_CAN_ENABLE_3,              {0x2b, 0x40, 0x60, 0x00, 0x0f, 0x00, 0x00, 0x00}},
	{CODE_QUICK_STOP,                {0x2b, 0x40, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{CODE_SPEED_MODE,                {0x2f, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00}},
	{CODE_POSITION_MODE,             {0x2f, 0x60, 0x60, 0x00, 0x01, 0x00, 0x00, 0x00}},
	{CODE_SET_POS_LEFT,              {0x23, 0x7a, 0x60, 0x01, 0x00, 0x00, 0x00, 0x00}},
	{CODE_SET_POS_RIGHT,             {0x23, 0x7a, 0x60, 0x02, 0x00, 0x00, 0x00, 0x00}},
	{CODE_SET_POS_REL_START_PREPARE, {0x2b, 0x40, 0x60, 0x00, 0x4f, 0x00, 0x00, 0x00}},
	{CODE_SET_POS_REL_START_RUN,     {0x2b, 0x40, 0x60, 0x00, 0x5f, 0x00, 0x00, 0x00}},
	{CODE_SET_SPEED_LEFT,            {0x23, 0xFF, 0x60, 0x01, 0x64, 0x00, 0x00, 0x00}},
	{CODE_SET_SPEED_RIGHT,           {0x23, 0xFF, 0x60, 0x02, 0x64, 0x00, 0x00, 0x00}},
	{CODE_SET_SPEED_LEFT_ACC,        {0x23, 0x83, 0x60, 0x01, 0xC8, 0x00, 0x00, 0x00}},
	{CODE_SET_SPEED_RIGHT_ACC,       {0x23, 0x83, 0x60, 0x02, 0xC8, 0x00, 0x00, 0x00}},
	{CODE_SET_SPEED_LEFT_DEC,        {0x23, 0x84, 0x60, 0x01, 0xF4, 0x01, 0x00, 0x00}},
	{CODE_SET_SPEED_RIGHT_DEC,       {0x23, 0x84, 0x60, 0x02, 0xF4, 0x01, 0x00, 0x00}},	
	{CODE_SYNC_SPEED,                {0x23, 0xFF, 0x60, 0x03, 0x64, 0x00, 0x64, 0x00}},
	{CODE_SPEED_READ,                {0x43, 0x6C, 0x60, 0x03, 0x00, 0x00, 0x00, 0x00}},
	{CODE_LEFT_COUNT,                {0x43, 0x64, 0x60, 0x01, 0x00, 0x00, 0x00, 0x00}},
	{CODE_RIGHT_COUNT,               {0x43, 0x64, 0x60, 0x02, 0x00, 0x00, 0x00, 0x00}},
	{CODE_LEFT_KP,                   {0x2b, 0x1d, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00}},
	{CODE_RIGHT_KP,                  {0x2b, 0x1d, 0x20, 0x02, 0x00, 0x00, 0x00, 0x00}},
	{CODE_LEFT_KI,                   {0x2b, 0x1e, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00}},
	{CODE_RIGHT_KI,                  {0x2b, 0x1e, 0x20, 0x02, 0x00, 0x00, 0x00, 0x00}},
	{CODE_LEFT_POLE,                 {0x2b, 0x0c, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00}},
	{CODE_RIGHT_POLE,                {0x2b, 0x0c, 0x20, 0x02, 0x00, 0x00, 0x00, 0x00}},
	{CODE_LEFT_ENCODER_RANGE,        {0x2b, 0x0E, 0x20, 0x01, 0x00, 0x01, 0x00, 0x00}},
	{CODE_RIGHT_ENCODER_RANGE,       {0x2b, 0x0E, 0x20, 0x02, 0x00, 0x01, 0x00, 0x00}},
};

void motor_quick_stop()
{
	struct driver_sdo_data can_data;
	size_t data_len = sizeof(can_data);
	int fd=g_motor_driver.mc_fd;
	char buffer[8]={0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};

	if (g_motor_driver.initialized != true){
		syslog(LOG_ERR,"motor_driver not init \n");
		return;
	}

	syslog(LOG_DEBUG,"motor quick stop\n");
	usleep(2000);
	can_data = g_command_list[CODE_QUICK_STOP].command;
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

}

void motor_resume_enable(uint8_t mode)
{
	struct driver_sdo_data can_data;
	size_t data_len = sizeof(can_data);
	int fd=g_motor_driver.mc_fd;
	char buffer[8]={0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};

	if (g_motor_driver.initialized != true){
		syslog(LOG_ERR,"motor_driver not init \n");
		return;
	}

	syslog(LOG_INFO,"motor resume enable\n");
	usleep(2000);


	can_data = g_command_list[CODE_CAN_ENABLE_3].command;
	canopen_send(fd, &can_data, data_len);
	usleep(2000);
	canopen_receive(fd, buffer, CAN_STD_SIZE);
	usleep(2000);		

	if (mode == CAR_POSITION_MODE)
	{
		motor_position_set(0, 0); // update target reached flag for next pos cmd
	}
}

static void set_motor_pid(uint16_t kp,uint16_t ki)
{
	struct driver_sdo_data can_data;
	size_t data_len = sizeof(can_data);
	int fd=g_motor_driver.mc_fd;
	char buffer[8]={0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
	if(kp > 30000) 
		kp =30000;
	if(ki > 30000)
	    ki = 30000;

	if (g_motor_driver.initialized != true){
		syslog(LOG_ERR,"motor_driver not init \n");
		return;
	}

	syslog(LOG_INFO,"PID_SET\n");
	usleep(2000);
	can_data = g_command_list[CODE_LEFT_KP].command;
	can_data.data1_l = kp && 0xff;
	can_data.data1_h = (kp >> 8)&& 0xff;
	canopen_send(fd, &can_data, data_len);
	usleep(2000);
	canopen_receive(fd, buffer, CAN_STD_SIZE);
	usleep(2000);

	can_data = g_command_list[CODE_RIGHT_KP].command;
	can_data.data1_l = kp && 0xff;
	can_data.data1_h = (kp >> 8)&& 0xff;
	canopen_send(fd, &can_data, data_len);
	usleep(2000);
	canopen_receive(fd, buffer, CAN_STD_SIZE);
	usleep(2000);

	can_data = g_command_list[CODE_LEFT_KI].command;
	can_data.data1_l = ki && 0xff;
	can_data.data1_h = (ki >> 8)&& 0xff;
	canopen_send(fd, &can_data, data_len);
	usleep(2000);
	canopen_receive(fd, buffer, CAN_STD_SIZE);
	usleep(2000);

	can_data = g_command_list[CODE_RIGHT_KI].command;
	can_data.data1_l = ki && 0xff;
	can_data.data1_h = (ki >> 8)&& 0xff;
	canopen_send(fd, &can_data, data_len);
	usleep(2000);
	canopen_receive(fd, buffer, CAN_STD_SIZE);
	usleep(2000);

}

void zlac8015d_driver_error_check(void)
{
        struct driver_sdo_data can_data;
        size_t data_len = sizeof(can_data);
        int fd = g_motor_driver.mc_fd;
        char buffer[CAN_STD_SIZE];

        if (g_motor_driver.initialized != true)
	{
                syslog(LOG_ERR,"motor_driver not init \n");
                return;
        }

        //send err code register
        can_data = g_command_list[CODE_DEBUG_CC].command;
        canopen_send(fd, &can_data, data_len);
	usleep(5000);
	canopen_receive(fd, buffer, CAN_STD_SIZE);
	//printf("Read debug error data %x, %x, %x, %x \n",buffer[4],buffer[5],buffer[6],buffer[7]);

	if(buffer[4] == 0x01)
	{
		syslog(LOG_ERR,"over voltage! \n");
	}

	switch(buffer[4])
	{
		case 0x01:
                    syslog(LOG_ERR,"over voltage! \n");
                        break;

                case 0x02:
                    syslog(LOG_ERR,"lack voltage! \n");
                        break;

                case 0x04:
                    syslog(LOG_ERR,"left motor over current! \n");
                        break;

                case 0x08:
                    syslog(LOG_ERR,"left motor overload! \n");
                        break;

                case 0x10:
                    syslog(LOG_ERR,"left motor over voltage! \n");
                        break;

                case 0x20:
                    syslog(LOG_ERR,"encoder over proof! \n");
                        break;

                case 0x40:
                    syslog(LOG_ERR,"speed over proof! \n");

                case 0x80:
                    syslog(LOG_ERR,"reference voltage error! \n");
                    break;

                default:
                    break;
        }

        switch(buffer[5])
        {
                case 0x01:
                    syslog(LOG_ERR,"EEPROM read-write error! \n");
                    break;

                case 0x02:
                    syslog(LOG_ERR,"Hall sensor error! \n");
                    break;

                default:
                    break;
        }

        switch(buffer[6])
        {
                case 0x04:
                    syslog(LOG_ERR,"right motor over current! \n");
                    break;

                case 0x08:
                    syslog(LOG_ERR,"right motor over load! \n");
                    break;

                case 0x10:
                    syslog(LOG_ERR,"right motor over voltage! \n");
                    break;

                case 0x20:
                    syslog(LOG_ERR,"encoder over proof");
                    break;

                case 0x40:
                    syslog(LOG_ERR,"speed over proof! \n");
                    break;

                case 0x80:
                    syslog(LOG_ERR,"reference voltage error! \n");
                    break;

                default:
                    break;

        }

        if(buffer[7] == 0x02)
        {
                syslog(LOG_ERR,"hall sensor error! \n");
	}
	if((buffer[4] == 0) && (buffer[5] == 0) && (buffer[6] == 0) && (buffer[7] == 0))
        {
                syslog(LOG_DEBUG,"motor driver ok! \n");
        }
}

/*set wheel speed. input: rotation: range -300 ~ 300. it must be integer*/
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

	//process speed value
	syslog(LOG_INFO,"set speed \n");
	canopen_send(fd, &left_data, data_len);
	usleep(2000);
	canopen_send(fd, &right_data, data_len);
	usleep(2000);
}

void motor_speed_sync(int left_speed_rpm, int right_speed_rpm, uint32_t time)
{
	struct driver_sdo_data motor_speed;
	int i = 0;
	uint32_t delta = 0;
	size_t data_len = sizeof(motor_speed);
        int fd = g_motor_driver.mc_fd;
	char buffer[8]={0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};

	motor_speed = g_command_list[CODE_SYNC_SPEED].command;

	motor_speed.data1_l = left_speed_rpm&0xff;
	motor_speed.data1_h = left_speed_rpm>>8;

	motor_speed.data2_l = right_speed_rpm&0xff;
        motor_speed.data2_h = right_speed_rpm>>8;

	delta = clock_systime_ticks() - time;

	if (i++ % 10 == 0)
	{
		//syslog("cmd delay %dms \n", delta*10);
	}

	//syslog(LOG_INFO, "cmd delay %dms \n", delta*10);
	canopen_send(fd, &motor_speed, data_len);
	usleep(1000);
	canopen_receive(fd, buffer, CAN_STD_SIZE);
}

int motor_speed_read(amr_motor_data_t *data)
{
	struct driver_sdo_data motor_speed = {0};
	size_t data_len = sizeof(motor_speed);
	int fd = g_motor_driver.mc_fd;
	int ret = OK;
	int16_t rpm_right;
	int16_t rpm_left;
	uint8_t car_type;

	canopen_send(fd, &g_command_list[CODE_SPEED_READ].command, data_len);
	usleep(1000);

	car_type = get_car_type();

	ret = canopen_receive(fd, &motor_speed, CAN_STD_SIZE);

	if (ret > 0 && motor_speed.index_l ==  0x6C && motor_speed.index_h == 0x60)
	{
		rpm_left = motor_speed.data1_h<<8 | motor_speed.data1_l;
		rpm_right = motor_speed.data2_h<<8 | motor_speed.data2_l;

                if(car_type == 0)
		{
                    data->left_rpm =rpm_left;
		    data->right_rpm = rpm_right;
		}

		else
                {
		    data->left_rpm = - rpm_left;
                    data->right_rpm = - rpm_right;
		}

		ret = clock_gettime(CLOCK_REALTIME, &data->tp);
		if (ret < 0)
		{
			syslog(LOG_WARNING, "speed read timestamp failed \n");
		}
	}
	return ret;
}

void motor_position_read(amr_motor_data_t *data)
{
	struct driver_sdo_data motor_position_left = {0};
	struct driver_sdo_data motor_position_right = {0};
	size_t data_len = sizeof(motor_position_left);
	int fd = g_motor_driver.mc_fd;
	int ret = 0;

	canopen_send(fd, &g_command_list[CODE_LEFT_COUNT].command, data_len);
	usleep(5000);

	ret = canopen_receive(fd, &motor_position_left, CAN_STD_SIZE);
	usleep(5000);

	canopen_send(fd, &g_command_list[CODE_RIGHT_COUNT].command, data_len);
	usleep(5000);

	ret = canopen_receive(fd, &motor_position_right, CAN_STD_SIZE);

	if (ret > 0 && motor_position_left.index_l ==  0x64 && motor_position_left.index_h == 0x60)
	{
		//printf("position read conuts %d,%d \n", (int)motor_position_left.data1_h<<8|motor_position_left.data1_l|motor_position_left.data2_h<<24|motor_position_left.data2_l<<16,\
		(int)motor_position_right.data1_h<<8|motor_position_right.data1_l|motor_position_right.data2_h<<24|motor_position_right.data2_l<<16);	
		data->left_counts = (int)motor_position_left.data1_h<<8|motor_position_left.data1_l|motor_position_left.data2_h<<24|motor_position_left.data2_l<<16;
		data->right_counts = (int)motor_position_right.data1_h<<8|motor_position_right.data1_l|motor_position_right.data2_h<<24|motor_position_right.data2_l<<16;

		ret = clock_gettime(CLOCK_REALTIME, &data->tp);
		if (ret < 0)
		{
			syslog(LOG_WARNING, "speed read timestamp failed \n");
		}
	}
}


uint16_t motor_status_read(void)
{
	struct driver_sdo_data can_data;
	struct driver_sdo_data motor_status = {0};
	size_t data_len = sizeof(can_data);
	int fd = g_motor_driver.mc_fd;
	uint16_t left_status = 0, right_status = 0;
	uint8_t target_reached_left = 0, target_reached_right = 0; 

	if (g_motor_driver.initialized != true)
	{
        	//printf("motor_driver not init \n");
        	return 0;
	}

	can_data = g_command_list[CODE_MOTOR_STATUS].command;
	canopen_send(fd, &can_data, data_len);
	usleep(1000);
	canopen_receive(fd, &motor_status, CAN_STD_SIZE);

	left_status = motor_status.data1_h << 8 | motor_status.data1_l;
	right_status = motor_status.data2_h << 8 | motor_status.data2_l;

	return left_status & right_status;
}

/* only valid in position mode */
bool motor_target_reached()
{
	uint16_t target_reached = 0; 

	target_reached = motor_status_read() & STATUS_BIT_TARGET_REACHED;

	//printf("target reached %d\n", target_reached);

	return !!target_reached;
}

/* only valid in velocity mode */
bool motor_velocity_zero()
{
	uint16_t velocity_zero = 0; 

	velocity_zero = motor_status_read() & STATUS_BIT_VELOCITY_ZERO;

	//printf("velocity zero %d\n", velocity_zero);

	return !!velocity_zero;
}

void motor_position_set(int left_cnt, int right_cnt)
{
	struct driver_sdo_data can_data;
	struct driver_sdo_data motor_pos;
	size_t data_len = sizeof(motor_pos);
	int fd = g_motor_driver.mc_fd;
	char buffer[8]={0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};

	if (g_motor_driver.initialized != true){
		//printf("motor_driver not init \n");
		return;
	}

	motor_pos = g_command_list[CODE_SET_POS_LEFT].command;
	motor_pos.data1_l = left_cnt&0xff;
	motor_pos.data1_h = (left_cnt>>8) & 0xff;
	motor_pos.data2_l = (left_cnt>>16) & 0xff;
	motor_pos.data2_h = (left_cnt>>24) & 0xff;
	canopen_send(fd, &motor_pos, data_len);
	usleep(1000);
	canopen_receive(fd, buffer, CAN_STD_SIZE);
	usleep(1000);

	motor_pos = g_command_list[CODE_SET_POS_RIGHT].command;
	motor_pos.data1_l = right_cnt&0xff;
	motor_pos.data1_h = (right_cnt>>8) & 0xff;
	motor_pos.data2_l = (right_cnt>>16) & 0xff;
	motor_pos.data2_h = (right_cnt>>24) & 0xff;
	canopen_send(fd, &motor_pos, data_len);
	usleep(1000);
	canopen_receive(fd, buffer, CAN_STD_SIZE);
	usleep(1000);

	can_data = g_command_list[CODE_SET_POS_REL_START_PREPARE].command;
	canopen_send(fd, &can_data, data_len);
	usleep(2000);
	canopen_receive(fd, buffer, CAN_STD_SIZE);
	usleep(2000);

	can_data = g_command_list[CODE_SET_POS_REL_START_RUN].command;
	canopen_send(fd, &can_data, data_len);
	usleep(2000);
	canopen_receive(fd, buffer, CAN_STD_SIZE);
	usleep(2000);

	syslog(LOG_INFO, "position set left %d right %d\n", left_cnt, right_cnt);

}

int motor_control_mode_switch(uint8_t mode)
{
	struct driver_sdo_data can_data;
	size_t data_len = sizeof(can_data);
	int fd = g_motor_driver.mc_fd;
	char buffer[8]={0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};

	if (g_motor_driver.initialized != true){
		//printf("motor_driver not init \n");
		return ERROR;
	}

	if (mode == CAR_POSITION_MODE)
	{
		can_data = g_command_list[CODE_POSITION_MODE].command;
		canopen_send(fd, &can_data, data_len);
		usleep(2000);
		canopen_receive(fd, buffer, CAN_STD_SIZE);
		usleep(2000);
	}
	else if(mode == CAR_VELOCITY_MODE)
	{
		can_data = g_command_list[CODE_SPEED_MODE].command;
		canopen_send(fd, &can_data, data_len);
		usleep(2000);
		canopen_receive(fd, buffer, CAN_STD_SIZE);
		usleep(2000);
	}
	else
	{
		syslog(LOG_ERR,"control mode not exist! \n");
		return ERROR;
	}

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

	if (mode == CAR_POSITION_MODE)
	{
		motor_position_set(0, 0); // update target reached flag for next mode switch
	}

	syslog(LOG_INFO, "mode switch to %d \n", mode);

	return OK;

}

static void zlac8015d_driver_init(void)
{
	struct driver_sdo_data can_data;
	size_t data_len = sizeof(can_data);
	int fd = g_motor_driver.mc_fd;
	char buffer[8]={0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};

	if (g_motor_driver.initialized != true){
		syslog(LOG_ERR,"motor_driver not init \n");
		return;
	}

	//ASYNC_SET
	//printf("ASYNC_SET\n");
	//usleep(200000);
	//can_data = g_command_list[CODE_CAN_ASYNC].command;
        //canopen_send(fd, &can_data, data_len);
	//SYNC_SET
	syslog(LOG_INFO,"SYNC_SET\n");
	usleep(200000);
	can_data = g_command_list[CODE_CAN_SYNC].command;
	canopen_send(fd, &can_data, data_len);
	//ENABLE
	syslog(LOG_INFO,"ENABLE\n");
	usleep(2000);
	canopen_receive(fd, buffer, CAN_STD_SIZE);
	usleep(2000);
	syslog(LOG_INFO,"SPEED MODE\n");
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

	/* init poloe */
	can_data = g_command_list[CODE_LEFT_POLE].command;
	can_data.data1_l = M_POLOE && 0xff;
	can_data.data1_h = (M_POLOE >> 8)&& 0xff;
	canopen_send(fd, &can_data, data_len);
	usleep(2000);
	canopen_receive(fd, buffer, CAN_STD_SIZE);
	usleep(2000);

	can_data = g_command_list[CODE_RIGHT_POLE].command;
	can_data.data1_l = M_POLOE && 0xff;
	can_data.data1_h = (M_POLOE >> 8)&& 0xff;
	canopen_send(fd, &can_data, data_len);
	usleep(2000);
	canopen_receive(fd, buffer, CAN_STD_SIZE);
	usleep(2000);

	/* init PID */
	set_motor_pid(KP,KI);
}

int motor_driver_init(void)
{
	int fd;
	//open fd
	fd = open(MOTOR_DRIVER_DEV, O_RDWR|O_NONBLOCK);
	//fd = open(MOTOR_DRIVER_DEV, O_RDWR);
	if (fd < 0)
	{
	syslog(LOG_ERR,"ERROR: open %s failed: %d\n", MOTOR_DRIVER_DEV, errno);
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
