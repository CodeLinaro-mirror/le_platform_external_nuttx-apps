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
#include "car_main.h"
#include "motion_task.h"

static struct car_ros_com_s g_car_ros;

float xyz_target_speed_transition(uint8_t high, uint8_t low)
{
	short transition;

	transition = ((high << 8) + low);
	return
		(float) (transition / 1000 + (transition % 1000) * 0.001); //Unit conversion, mm/s->m/s
}


static void parse_speed_data(void){
	struct ros_receive_data_s *p_data;

	p_data = &g_car_ros.receive_data;
	
	g_car_ros.parse_data.x_speed = xyz_target_speed_transition(p_data->x_high, p_data->x_low);
	g_car_ros.parse_data.z_speed = xyz_target_speed_transition(p_data->z_high, p_data->z_low);
	g_car_ros.speed_ready = true;
	printf("parse_speed_data: get speed_x = %f speed_z:%f\n",g_car_ros.parse_data.x_speed,g_car_ros.parse_data.z_speed);
}

static void parse_pid_data(void){
	struct ros_receive_data_s *p_data;

	p_data = &g_car_ros.receive_data;

	g_car_ros.parse_data.kp = xyz_target_speed_transition(p_data->x_high, p_data->x_low) * 10;
	g_car_ros.parse_data.ki = xyz_target_speed_transition(p_data->z_high, p_data->z_low) * 10;
	g_car_ros.pid_ready = true;
	printf("parse_pid_data: get kp = %f,ki= %f \n",g_car_ros.parse_data.kp,g_car_ros.parse_data.ki);

}

static void parse_ros_data(void){
	switch (g_car_ros.receive_data.cmd){
		case(CMD_SPEED_CONFIGURE):
			//
			parse_speed_data();
			break;
		case(CMD_PID):
			/* pid configure */
			parse_pid_data();
			break;
		default:
			printf("parse_ros_data: cmd invalid \n");
			break;
	}
}

/* ros com  task  */
static uint8_t  ros_com_get_data(void){
	int ret;
	FAR char *buffer;
	int fd;
	struct car_ros_com_s *ros_com;
	uint8_t *current_len;
	ssize_t n;

	ros_com = &g_car_ros;
	fd = ros_com-> ros_fd;
	buffer = (char *)&ros_com->receive_data;
	current_len = &ros_com->current_receive_len;

	if (buffer[0] != ROS_FRAME_HEAD){
		*current_len = 0;
	}

	n = read(fd, buffer+*current_len, 1);

	up_udelay(1000);
	if (n < 0) {
		printf("read failed: %d\n", errno);
		fflush(stdout);
	}

	if ((*current_len) == (RECEIVE_BUFF_SIZE-1)){
		printf("read data full frame ");
		/* have received full frame */
		if (buffer[*current_len] == ROS_FRAME_TAIL) {
			parse_ros_data();
		}
		*current_len = 0;
	}
	else{
		*current_len = (*current_len) +1;
	}

	return OK;
}
	

int get_ros_goal_speed(float* vx, float* vz){

	if (g_car_ros.speed_ready)
	{
		*vx = g_car_ros.parse_data.x_speed;
		*vz = g_car_ros.parse_data.z_speed;
		g_car_ros.speed_ready = false;
		return OK;
	}
	else
		return ERROR;
}

int get_ros_pid(float* kp, float* ki){

	if (g_car_ros.pid_ready)
	{
		*kp = g_car_ros.parse_data.kp;
		*ki = g_car_ros.parse_data.ki;
		g_car_ros.pid_ready = false;
		return OK;
	}
	else
		return ERROR;
}


/* Init ros communication */
static int ros_com_init(void){

	g_car_ros.ros_fd = open(ROS_COM_DEV, O_RDWR);
	printf("ros_com_init: open   %s\n",ROS_COM_DEV);
	if (g_car_ros.ros_fd < 0){
		printf("ros_com_init: open %s failed: %d\n",
									ROS_COM_DEV, errno);
		return g_car_ros.ros_fd;
	}

	g_car_ros.tx_ready = false;
	g_car_ros.speed_ready = false;
	g_car_ros.pid_ready = false;
	
	g_car_ros.current_receive_len = 0;
	g_car_ros.current_send_len = 0;
	g_car_ros.parse_data.x_speed = 0.0;
	g_car_ros.parse_data.z_speed = 0.0;

	return OK;
}
void ros_com_deinit(void)
{
	close(g_car_ros.ros_fd);
}

int ros_com_task(int argc, char *argv[]){
	int ret;
	
	ret = ros_com_init();
	if (ret != OK){
		printf("ros_com_task: init failed\n");
	}
	
	while(1){
		/* read serial data*/
		if (OK != ros_com_get_data()){
			printf("ros_com_get_data:get data failed\n");
		}
	}

	ros_com_deinit();
	return ret;
}
