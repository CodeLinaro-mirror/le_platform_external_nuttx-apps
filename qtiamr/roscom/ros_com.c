/****************************************************************************
* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
****************************************************************************/
#include <nuttx/config.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <sched.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <syslog.h>

#include "ros_com.h"
#include "main.h"
#include "motion_task.h"
#include "motor_driver.h"

extern volatile struct amr_motion_s g_amr_motion;

#define READ_BUFF_LEN  (EXTEND_FRAME_LEN+STANDARD_FRAME_LEN)
volatile struct amr_ros_com_s g_amr_ros;
uint8_t g_read_buffer[READ_BUFF_LEN];
uint8_t g_read_buffer_availabe_len;
enum parameter_download_state parameter_stat = PAR_COM_INIT;
extern struct task_signal_info task_info[TASK_NUM];
volatile struct config_parameter_data parameter_data = {
	CAR_TYPE_AMR_SMALL,
	MAX_SPEED,
	MAX_SPEED,
	MAX_ANGULAR_VELOCITY,
	MAX_ANGULAR_VELOCITY,
	true,
};

static int ros_cmd_send(void *buf, size_t len);
static void ros_cmd_receive(void);

static uint8_t data_check_sum(char *buff, uint8_t data_length)
{
	uint8_t check_sum;
	uint8_t k;

	if (NULL == buff)
	{
		return ERROR;
	}
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
	{
		return;
	}

	buff[0] = data >>8;
	buff[1] = (uint8_t)data;
}

/* get xyz speed */

float xyz_target_speed_transition(uint8_t high, uint8_t low)
{

	short transition;

	transition = ((high << 8) + low);
	return (float) (transition / 1000 + (transition % 1000) * 0.001); //Unit conversion, mm/s->m/s
}

static void parse_speed_data(void)
{
	uint8_t car_type;
	struct ros_receive_data_s *p_data;

	if(g_amr_ros.velocity_enable == false)
	{
		return;
	}

	p_data = &g_amr_ros.ros_receive_data;
	car_type = get_car_type();

	if(car_type == 0)
	{
		g_amr_ros.parse_data.x_speed = xyz_target_speed_transition(p_data->data1, p_data->data2);
		g_amr_ros.parse_data.z_speed = xyz_target_speed_transition(p_data->data5, p_data->data6);
	}

	else
	{
		g_amr_ros.parse_data.x_speed = - xyz_target_speed_transition(p_data->data1, p_data->data2);
		g_amr_ros.parse_data.z_speed = - xyz_target_speed_transition(p_data->data5, p_data->data6);
	}
	g_amr_ros.speed_ready = true;
}

void parse_position_data(struct ros_receive_data_s p_data)
{
	g_amr_ros.parse_data.x = xyz_target_speed_transition(p_data.data1, p_data.data2);
	g_amr_ros.position_enable = false;
	g_amr_ros.position_update = true;
	g_amr_ros.pos_sub_mode = POS_SUB_DISTANCE;
	syslog(LOG_INFO, "distance is %.3f \n", g_amr_ros.parse_data.x);
}

void parse_angle_data(struct ros_receive_data_s p_data)
{
	g_amr_ros.parse_data.z = xyz_target_speed_transition(p_data.data1, p_data.data2);
	g_amr_ros.position_enable = false;
	g_amr_ros.position_update = true;
	g_amr_ros.pos_sub_mode = POS_SUB_ANGLE;
	syslog(LOG_INFO, "angle is %.3f \n", g_amr_ros.parse_data.z);
}
void target_control_mode_set(uint8_t mode);
void control_update_state_set(bool update);

int control_mode_switch(uint8_t control_mode)
{
	if (control_mode >= CAR_MODE_NUM)
	{
		syslog(LOG_ERR, "ACK_OUT_RANGE\n");
		return ACK_OUT_RANGE;
	}

	if (control_mode == g_amr_motion.control_mode)
	{
		syslog(LOG_ERR, "ACK_REPEAT\n");
		return ACK_REPEAT;
	}

	if (g_amr_motion.control_mode == CAR_VELOCITY_MODE && g_amr_motion.velocity_zero == 0)
	{
		syslog(LOG_ERR, "ACK_STATUS_ERR not stop\n");
		return ACK_STATUS_ERR;
	}

	if (g_amr_motion.control_mode == CAR_POSITION_MODE && g_amr_motion.target_reached == 0)
	{
		syslog(LOG_ERR, "ACK_STATUS_ERR not target reach\n");
		return ACK_STATUS_ERR;
	}

	if (control_mode == CAR_POSITION_MODE)
	{
		g_amr_ros.velocity_enable = false;
	}
	else if(control_mode == CAR_VELOCITY_MODE)
	{
		g_amr_ros.position_enable = false;
	}

	target_control_mode_set(control_mode);
	control_update_state_set(true);

	syslog(LOG_INFO, "target mode %d update %d\n", g_amr_motion.target_control_mode, g_amr_motion.control_mode_update);

	return ACK_SUCCESS;

}

float get_max_vx(void)
{
	return parameter_data.vx_max;
}

float get_max_vz(void)
{
	return parameter_data.vz_max;
}

uint8_t get_car_type(void)
{
	return parameter_data.car_type;
}


void parameter_download_config(struct ros_receive_data_s data)
{
	switch (data.reserved)
	{
		case (CMD_CAR_TYPE_SET):
			if (data.data1 < CAR_TYPE_NUM)
			{
				parameter_data.car_type = data.data1;
				syslog(LOG_INFO, "car type is %d \n", parameter_data.car_type);
			}
			else
			{
				syslog(LOG_INFO, "car type error %d \n", parameter_data.car_type);
			}
			break;
		case (CMD_MAX_VX_SET):
			parameter_data.vx_max = xyz_target_speed_transition(data.data1, data.data2);
			syslog(LOG_INFO, "vx max speed %.3f \n", parameter_data.vx_max);
			break;
		case (CMD_POS_MAX_VX_SET):
			parameter_data.vx_max_pos = xyz_target_speed_transition(data.data1, data.data2);
			syslog(LOG_INFO, "vx pos max speed %.3f \n", parameter_data.vx_max_pos);
			break;
		case (CMD_MAX_VZ_SET):
			parameter_data.vz_max = xyz_target_speed_transition(data.data1, data.data2);
			syslog(LOG_INFO, "vz max speed %.3f \n", parameter_data.vz_max);
			break;
		case (CMD_POS_MAX_VZ_SET):
			parameter_data.vz_max_pos = xyz_target_speed_transition(data.data1, data.data2);
			syslog(LOG_INFO, "vz pos max speed %.3f \n", parameter_data.vz_max_pos);
			break;

		case (CMD_ENABLE_RC):
			parameter_data.rc_enable = data.data1;
			syslog(LOG_INFO, "rc enable is %d \n", parameter_data.rc_enable);
			break;
		case(CMD_PARAMETER_DONE):
			parameter_stat = PAR_LOAD_DONE;
			syslog(LOG_INFO, "parameter download done \n");
			trigger_time_sync();
			notify_pending_task();
			break;
	}
}

int notify_pending_task(void)
{
	int ret = 0;

	ret = kill(task_info[TASK_MOTION].task_id, task_info[TASK_MOTION].signal);
	syslog(LOG_INFO, "pid %d send car signal %d ret %d \n", task_info[TASK_MOTION].task_id, task_info[TASK_MOTION].signal, ret);
	ret = kill(task_info[TASK_ULTRASOUND].task_id, task_info[TASK_ULTRASOUND].signal);
	syslog(LOG_INFO, "pid %d send car signal %d ret %d \n", task_info[TASK_ULTRASOUND].task_id, task_info[TASK_ULTRASOUND].signal, ret);

}

void mode_switch_notify(void)
{
	struct ros_receive_data_s ack_data;

	ack_data.data1 = ACK_SUCCESS;
	ack_data.data2 = g_amr_motion.control_mode;
	ack_data.cmd = CMD_CONTROL_MODE_STATUS; //mode to set: 0 vel, 1 pos; status:0 success, 1 failed.
	ack_data.frame_header = ROS_FRAME_HEAD;
	ack_data.frame_tail = ROS_FRAME_TAIL;
	ack_data.check_sum = data_check_sum(&ack_data, sizeof(struct ros_receive_data_s) - 2 );
	ros_cmd_send(&ack_data, sizeof(struct ros_receive_data_s));
}

static void parse_ros_data(void)
{
	struct timespec tp;
	struct ros_receive_data_s ack_data;
	int ret = OK;
	uint8_t control_mode = 0;
    uint8_t quick_stop_enable = 0;
    
    switch (g_amr_ros.ros_receive_data.cmd)
    {
        case(CMD_SPEED_CONTROL):
	    parse_speed_data();
	    break;

		case(CMD_TIME_LOOP):
			ros_cmd_send(&g_amr_ros.ros_receive_data, sizeof(struct ros_receive_data_s));
			syslog(LOG_INFO, "ack ping\n");
			break;

		case(CMD_CONTROL_MODE_SWITCH):
			control_mode = g_amr_ros.ros_receive_data.data1;
			syslog(LOG_INFO, "control mode %d\n", control_mode);
			ret = control_mode_switch(control_mode);
			if (ret)
			{
				ack_data.data1 = ret;
				ack_data.data2 = control_mode;
				ack_data.cmd = CMD_CONTROL_MODE_STATUS; //mode to set: 0 vel, 1 pos; status:0 success, 1 failed.
				ack_data.frame_header = ROS_FRAME_HEAD;
				ack_data.frame_tail = ROS_FRAME_TAIL;
				ack_data.check_sum = data_check_sum(&ack_data, sizeof(struct ros_receive_data_s) - 2 );
				ros_cmd_send(&ack_data, sizeof(struct ros_receive_data_s));
			}
			break;

		case(CMD_POSITION_SET):
			if (g_amr_ros.position_enable == true)
			{
				parse_position_data(g_amr_ros.ros_receive_data);
			}
			syslog(LOG_INFO, "receive pos cmd \n");
			break;

		case(CMD_ANGLE_SET):
			if (g_amr_ros.position_enable == true)
			{
				parse_angle_data(g_amr_ros.ros_receive_data);
			}
			syslog(LOG_INFO, "receive angle cmd \n");
			break;

		case(CMD_RESET_STATUS):
			parameter_stat = PAR_DOWN_LOAD;
			syslog(LOG_INFO, "receive reset ack\n");
			break;

		case(CMD_PARAMETER_DOWNLOAD):
			parameter_stat = PAR_DOWN_LOAD;
			parameter_download_config(g_amr_ros.ros_receive_data);
			ros_cmd_send(&g_amr_ros.ros_receive_data, sizeof(struct ros_receive_data_s));
			break;

		case(CMD_TIME_SET):
			tp.tv_sec = (g_amr_ros.ros_receive_data.data1<<24)|(g_amr_ros.ros_receive_data.data2<<16) | (g_amr_ros.ros_receive_data.data3<<8)|g_amr_ros.ros_receive_data.data4;
			tp.tv_nsec = ((g_amr_ros.ros_receive_data.data5 << 8) | g_amr_ros.ros_receive_data.data6) * 1000000;
			ret = clock_settime(CLOCK_REALTIME, &tp);
			syslog(LOG_INFO, "set time %d: %ds %dns\n", ret, tp.tv_sec, tp.tv_nsec);
			break;

		case(CMD_TIME_GET):
			ret = clock_gettime(CLOCK_REALTIME, &tp);
			ack_data.data1 = (tp.tv_sec >> 24) & 0xff;
			ack_data.data2 = (tp.tv_sec >> 16) & 0xff;
			ack_data.data3 = (tp.tv_sec >> 8) & 0xff;
			ack_data.data4 = (tp.tv_sec) & 0xff;
			ack_data.data5 = ((tp.tv_nsec/1000000) >> 8) & 0xff;
			ack_data.data6 = (tp.tv_nsec/1000000) & 0xff;
		    
		    ack_data.cmd = CMD_TIME_GET;
		    ack_data.frame_header = ROS_FRAME_HEAD;
	    	    ack_data.frame_tail = ROS_FRAME_TAIL;
		    ack_data.check_sum = data_check_sum(&ack_data, sizeof(struct ros_receive_data_s) - 2 );
		    ros_cmd_send(&ack_data, sizeof(struct ros_receive_data_s));
		    syslog(LOG_INFO, "get time %d: %ds %dns\n", ret, tp.tv_sec, tp.tv_nsec);
		    break;

		case(CMD_QUICK_STOP_ENABLE):
			quick_stop_enable = g_amr_ros.ros_receive_data.data1;
			syslog(LOG_INFO, "quick_stop_enable %d\n", quick_stop_enable);
			quick_stop_status_set(quick_stop_enable);
			break;
		default:
			syslog(LOG_INFO,"parse_ros_data: cmd invalid 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n", g_amr_ros.ros_receive_data.frame_header, \
				g_amr_ros.ros_receive_data.cmd, g_amr_ros.ros_receive_data.data1<<24|g_amr_ros.ros_receive_data.data2<<16 |
				g_amr_ros.ros_receive_data.data3<<8|g_amr_ros.ros_receive_data.data4,\
				(g_amr_ros.ros_receive_data.data5 << 8 | g_amr_ros.ros_receive_data.data6),\
				g_amr_ros.ros_receive_data.check_sum, g_amr_ros.ros_receive_data.frame_tail);
		    break;
	}
}

/* get roscom frame */
static void ros_com_get_frame(void)
{
	uint8_t index =0;
	uint8_t data_len = READ_BUFF_LEN-g_read_buffer_availabe_len; //Effective data length
	uint8_t i;

	while (index < data_len) {
		if (g_read_buffer[index] != STANDARD_FRAME_HEADER) {
			index = index + 1;
			continue;
		}
		/* get right header */
		if ((data_len - index) >= STANDARD_FRAME_LEN) {
			//Get a complete frame of data
			memcpy(&g_amr_ros.ros_receive_data, &g_read_buffer[index],STANDARD_FRAME_LEN);
			//need to add checksum
			if (g_amr_ros.ros_receive_data.check_sum == data_check_sum(&g_amr_ros.ros_receive_data, sizeof(struct ros_receive_data_s) - 2))
			{
				clock_gettime(CLOCK_MONOTONIC, &g_amr_ros.tp_connect);
				parse_ros_data();
			}
			else
			{
				syslog(LOG_WARNING, "roscom checksum failed\n");
			}
			index = index + STANDARD_FRAME_LEN;
			continue;
		}
		else //stop check. frame has no  received not fully.
			break;
	}
	/*Clear the data in front of the index and keep the data behind */
	if (index > 0)
	{
		for(i=0;i++;i<(data_len-index))
		{
			g_read_buffer[i] = g_read_buffer[index+i];
		}
	}
	g_read_buffer_availabe_len = g_read_buffer_availabe_len + index;
}

int get_ros_goal_speed(float* vx, float* vz)
{
	if (g_amr_ros.speed_ready && parameter_data.rc_enable == false)
	{
		*vx = g_amr_ros.parse_data.x_speed;
		*vz = g_amr_ros.parse_data.z_speed;
		g_amr_ros.speed_ready = false;
		return OK;
	}
	else
	{
		return ERROR;
	}
}

int get_ros_goal_position(float* x, float* z, int *mode)
{
	if (g_amr_ros.position_update == false || parameter_data.rc_enable == true)
	{
		return ERROR;
	}

	if (g_amr_ros.pos_sub_mode == POS_SUB_DISTANCE)
	{
		*x = g_amr_ros.parse_data.x;
		*mode = POS_SUB_DISTANCE;
		g_amr_ros.position_update = false;
	}
	else if (g_amr_ros.pos_sub_mode == POS_SUB_ANGLE)
	{
		*z = g_amr_ros.parse_data.z;
		*mode = POS_SUB_ANGLE;
		g_amr_ros.position_update = false;
	}
	else
	{
		syslog(LOG_ERR, "get pos failed sub mode %d\n", g_amr_ros.pos_sub_mode);
		return ERROR;
	}
	syslog(LOG_ERR, "get pos sub mode %d x:%.3f z:%.3f\n", g_amr_ros.pos_sub_mode, *x, *z);

	return OK;
}

int publish_odom_to_ros(void)
{
	int ret = 0;
	struct extend_frame_s send_data = {0};
	amr_motor_data_t *data = NULL;
	static struct timespec tp_last;
	int16_t vx_16,vz_16;
	data = get_motor_odom();

	if (data == NULL)
	{
		return ERROR;
	}

	if (tp_last.tv_sec == data->tp.tv_sec && tp_last.tv_nsec == data->tp.tv_nsec)
	{
		return ERROR;
	}
	send_data.frame_header = EXTEND_FRAME_HEADER;
	send_data.cmd = CMD_ODOM;
	send_data.frame_tail = EXTEND_FRAME_TRAILER;

	send_data.data1 = (data->tp.tv_sec >> 24) & 0xff;
	send_data.data2 = (data->tp.tv_sec >> 16) & 0xff;
	send_data.data3 = (data->tp.tv_sec >> 8) & 0xff;
	send_data.data4 = (data->tp.tv_sec) & 0xff;
	send_data.data5 = ((data->tp.tv_nsec/1000000) >> 8) & 0xff;
	send_data.data6 = (data->tp.tv_nsec/1000000) & 0xff;

	vx_16 = data->vx *1000;
	vz_16 = data->vz*1000;
	send_data.data7 = (vx_16 >> 8)&0xff;
	send_data.data8 = (vx_16)&0xff;
	send_data.data11 = (vz_16 >> 8)&0xff;
	send_data.data12 = (vz_16)&0xff;

	send_data.check_sum = data_check_sum(&send_data, sizeof(struct extend_frame_s) - 2 );
	tp_last.tv_sec = data->tp.tv_sec;
	tp_last.tv_nsec = data->tp.tv_nsec;

	ret = ros_cmd_send(&send_data, sizeof(struct extend_frame_s));
	if (ret < 0)
	{
		syslog(LOG_WARNING, "publish odom failed!\n");
		return ERROR;
	}
	//syslog(LOG_DEBUG, "publish odom tp %d %d,vx %.3f,vz %.3f\n", data->tp.tv_sec, data->tp.tv_nsec/1000000, data->vx, data->vz);


	return OK;

}

int publish_pos_odom_to_ros(void)
{
	int ret = 0;
	struct extend_frame_s send_data = {0};
	amr_motor_data_t *data = NULL;
	static struct timespec tp_last;
	int px,pz;
	data = get_motor_odom();

	if (data == NULL)
	{
		return ERROR;
	}

	if (tp_last.tv_sec == data->tp.tv_sec && tp_last.tv_nsec == data->tp.tv_nsec)
	{
		return ERROR;
	}
	send_data.frame_header = EXTEND_FRAME_HEADER;
	send_data.cmd = CMD_POSITION_ODOM;
	send_data.frame_tail = EXTEND_FRAME_TRAILER;

	send_data.data1 = (data->tp.tv_sec >> 24) & 0xff;
	send_data.data2 = (data->tp.tv_sec >> 16) & 0xff;
	send_data.data3 = (data->tp.tv_sec >> 8) & 0xff;
	send_data.data4 = (data->tp.tv_sec) & 0xff;
	send_data.data5 = ((data->tp.tv_nsec/1000000) >> 8) & 0xff;
	send_data.data6 = (data->tp.tv_nsec/1000000) & 0xff;

	px = data->px *1000;
	pz = data->pz *1000;
	send_data.data7 = (px >> 24) & 0xff;
	send_data.data8 = (px >> 16) & 0xff;
	send_data.data9 = (px >> 8) & 0xff;
	send_data.data10 = (px) & 0xff;
	send_data.data11 = (pz >> 24) & 0xff;
	send_data.data12 = (pz >> 16) & 0xff;
	send_data.data13 = (pz >> 8) & 0xff;
	send_data.data14 = (pz) & 0xff;

	send_data.data15 = (data->target_reached) & 0x01;

	send_data.check_sum = data_check_sum(&send_data, sizeof(struct extend_frame_s) - 2 );
	tp_last.tv_sec = data->tp.tv_sec;
	tp_last.tv_nsec = data->tp.tv_nsec;

	ret = ros_cmd_send(&send_data, sizeof(struct extend_frame_s));
	if (ret < 0)
	{
		syslog(LOG_WARNING, "publish odom failed!\n");
		return ERROR;
	}
	//syslog(LOG_DEBUG, "publish odom tp %d %d,vx %.3f,vz %.3f\n", data->tp.tv_sec, data->tp.tv_nsec/1000000, data->vx, data->vz);


	return OK;

}

int publish_status_to_ros(void)
{
	int ret = 0;
	struct standard_frame_s send_data = {0};

	send_data.frame_header = STANDARD_FRAME_HEADER;
	send_data.cmd = CMD_EXCEPTION_STATUS;
	send_data.frame_tail = STANDARD_FRAME_TRAILER;

	send_data.data1 = 0xff;
	send_data.data2 = 0xff;
	send_data.data3 = 0xff;
	send_data.data4 = motor_stop_status_get() & 0xff;
	send_data.data5 = 0xff;
	send_data.data6 = 0xff;

	send_data.check_sum = data_check_sum(&send_data, sizeof(struct standard_frame_s) - 2 );

	ret = ros_cmd_send(&send_data, sizeof(struct standard_frame_s));
	if (ret < 0)
	{
		syslog(LOG_WARNING, "publish status failed!\n");
		return ERROR;
	}

	return OK;
}


int trigger_time_sync(void)
{
	int ret = 0;
	struct ros_receive_data_s send_data = {0};

	send_data.frame_header = ROS_FRAME_HEAD;
	send_data.cmd = CMD_TIME_SYNC_REQUEST;
	send_data.check_sum = data_check_sum(&send_data, sizeof(struct ros_receive_data_s) - 2 );;
	send_data.frame_tail = ROS_FRAME_TAIL;

	ret = ros_cmd_send(&send_data, sizeof(struct ros_receive_data_s));
	if (ret < 0)
	{
		return ERROR;
	}
	syslog(LOG_INFO, "trigger time sync!\n");

	return OK;
}

static int ros_cmd_send(void *buf, size_t len)
{
	int ret = 0;

	ret = write(g_amr_ros.ros_fd, buf, len);

	return ret;
}

/* Read data of a specified length from the underlying serial */
static void ros_cmd_receive(void)
{
	int ret = 0;
	uint8_t data_len =0;
	int fd = g_amr_ros.ros_fd;
	int n;

	if (g_amr_ros.initialized) {
		/* check serial read buffer available data length, it's nonblock function */
		ret = ioctl(fd, FIONREAD, (unsigned long)&data_len);
		if (data_len == 0)
			return;

		if (data_len < g_read_buffer_availabe_len) {
			/* read all available data*/
			n = read(fd, g_read_buffer+(READ_BUFF_LEN-g_read_buffer_availabe_len), data_len);
			//syslog(LOG_INFO, "roscom read data1 len = %d \n",n);
			g_read_buffer_availabe_len = g_read_buffer_availabe_len -n;
		}
		else {
			n = read(fd, g_read_buffer+(READ_BUFF_LEN-g_read_buffer_availabe_len), g_read_buffer_availabe_len);
			//syslog(LOG_INFO, "roscom read data2 len = %d \n",n);
			g_read_buffer_availabe_len =0;
		}
		//Parse the received data
		ros_com_get_frame();
	}
	else
		syslog(LOG_INFO, "amr ros not initialized \n");

}


/* Init ros communication */
static int ros_com_init(void)
{
	int ret = 0;

	//g_amr_ros.ros_fd = open(ROS_COM_DEV, O_RDWR|O_NONBLOCK);
	g_amr_ros.ros_fd = open(ROS_COM_DEV, O_RDWR);
	syslog(LOG_INFO,"ros_com_init: open   %s\n", ROS_COM_DEV);
	if (g_amr_ros.ros_fd < 0) {
		g_amr_ros.initialized = false;
		syslog(LOG_INFO,"ros_com_init: open %s failed: %d\n", ROS_COM_DEV, errno);
		return g_amr_ros.ros_fd;
	}

	g_amr_ros.initialized = true;
	g_read_buffer_availabe_len = READ_BUFF_LEN;

	g_amr_ros.tx_ready = false;
	g_amr_ros.speed_ready = false;
	g_amr_ros.pid_ready = false;

	g_amr_ros.current_receive_len = 0;
	g_amr_ros.current_send_len = 0;
	g_amr_ros.parse_data.x_speed = 0.0;
	g_amr_ros.parse_data.z_speed = 0.0;
	g_amr_ros.velocity_enable = true;
	g_amr_ros.pos_sub_mode = POS_SUB_DISTANCE;
	g_amr_ros.position_enable = false;
	g_amr_ros.position_update = false;
	g_amr_ros.notify_mode_switch = false;

    return OK;
}

void ros_com_deinit(void)
{
	close(g_amr_ros.ros_fd);
}

int broadcast_reset_state(void)
{

	int ret = 0;
	struct ros_receive_data_s send_data = {0};

	send_data.frame_header = ROS_FRAME_HEAD;
	send_data.cmd = CMD_RESET_STATUS;
	send_data.check_sum = data_check_sum(&send_data, sizeof(struct ros_receive_data_s) - 2 );;
	send_data.frame_tail = ROS_FRAME_TAIL;

	ret = ros_cmd_send(&send_data, sizeof(struct ros_receive_data_s));
	if (ret < 0)
	{
		return ERROR;
	}
	syslog(LOG_INFO, "broadcast reset state\n");

	return OK;
}
bool ros_connection_check(void)
{
	struct timespec tp;
	int32_t delta = 0;

	clock_gettime(CLOCK_MONOTONIC, &tp);
	delta = tp.tv_sec - g_amr_ros.tp_connect.tv_sec;

	if (delta > ROS_CONNECT_TIMEOUT)
	{
		g_amr_ros.connected = false;
	}
	else
	{
		g_amr_ros.connected = true;
	}

	return g_amr_ros.connected;
}

/* roscom task */
int ros_com_task(int argc, char *argv[])
{
    int ret;
	int i = 0, j = 0;
	
    ret = ros_com_init();
    if (ret != OK)
    {
        syslog(LOG_INFO,"ros_com_task: init failed\n");
    }

    while(1)
    {
		usleep(1000000/ROSCOM_FREQUENCY);
		/* read qrc data*/
		ros_cmd_receive();

		if (i++ % ROSCOM_FREQUENCY == 0 && parameter_stat == PAR_COM_INIT && i < ROSCOM_FREQUENCY*60)
		{
			broadcast_reset_state();
		}

		if (parameter_stat == PAR_LOAD_DONE)
		{
			if (ros_connection_check())
			{
				if(g_amr_motion.control_mode == CAR_VELOCITY_MODE)
				{
					publish_odom_to_ros();
				}
				else
				{
					publish_pos_odom_to_ros();
				}

				if (j++ % ROSCOM_FREQUENCY == 0)
				{
					publish_status_to_ros();
				}
			}
			if (g_amr_ros.notify_mode_switch)
			{
				mode_switch_notify();
				g_amr_ros.notify_mode_switch = false;
			}
		}

	}

	ros_com_deinit();
	return ret;
}
