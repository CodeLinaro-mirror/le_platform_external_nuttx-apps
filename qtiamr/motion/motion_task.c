
/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <string.h>
#include <inttypes.h>
#include <syslog.h>

#include <nuttx/arch.h>

#include "motion_task.h"
#include "main.h"
#include "ros_com.h"
#include "ultrasound.h"

extern struct amr_ros_com_s g_amr_ros;

volatile struct amr_motion_s g_amr_motion;
static amr_motor_data_t amr_motor_data;

void smooth_speed_control(float vx, float step)
{
	if (vx > g_amr_motion.smooth_speed)
	{
		g_amr_motion.smooth_speed += step;
	}
	else if (vx < g_amr_motion.smooth_speed)
	{
		g_amr_motion.smooth_speed -= step;
	}
	else
	{
		g_amr_motion.smooth_speed = vx;
	}

	if ((abs(vx) <0.1) && g_amr_motion.smooth_speed < (2.5 * step) && g_amr_motion.smooth_speed > (-2.5 * step))
	{
		g_amr_motion.smooth_speed = vx;
	}
}

void get_motor_actual_speed(int *left, int *right)
{
	*left = g_amr_motion.motor_left.rpm_actual;
	*right = g_amr_motion.motor_right.rpm_actual;
}

void get_motor_goal_speed(int *left, int *right)
{
	*left = g_amr_motion.motor_left.rpm_goal;
	*right = g_amr_motion.motor_right.rpm_goal;
}

amr_motor_data_t* get_motor_odom(void)
{
	return &amr_motor_data;
}

void target_control_mode_set(uint8_t mode)
{
	g_amr_motion.target_control_mode = mode;
}

void control_update_state_set(bool update)
{
	g_amr_motion.control_mode_update = update;
}

void quick_stop_status_set(bool enable)
{
	g_amr_motion.quick_stop_enable = enable;
}

bool motor_stop_status_get()
{
	return g_amr_motion.motor_stop_once;
}

int motion_task(int argc, char *argv[])
{
	struct motor_control_s *motor_left;
	struct motor_control_s *motor_right;
	struct timespec tp_start, tp_end;
	long delta = 0;
	float vx_rc = 0, vz_rc = 0;
	float vx_ros = 0, vz_ros = 0;
	float x_pos = 0, z_pos = 0;
	int left_goal = 0, right_goal = 0;
	int left_cnt = 0, right_cnt = 0;
	int sub_mode = 0;
	bool last_target_reached = false;
    uint32_t v_time = 0;
	g_amr_motion.motor_stop_once = false;

	int ret = OK;

	int i = 0;
	struct timespec tp = {60, 0};
	sigset_t set;
	struct siginfo value;

	sigemptyset(&set);
	sigaddset(&set, SIGUSR1);

	syslog(LOG_INFO,"motion task wait signal \n");
	sigtimedwait(&set, &value, &tp);
	syslog(LOG_INFO,"motion task receive signal %d code %d err %d \n",\
		value.si_signo,value.si_code, value.si_errno);

	g_amr_motion.control_mode = CAR_VELOCITY_MODE;
	g_amr_motion.velocity_zero = false;
	g_amr_motion.target_reached = false;
	g_amr_motion.target_control_mode = CAR_VELOCITY_MODE;
	g_amr_motion.control_mode_update = false;
	g_amr_motion.quick_stop_enable = true;
	syslog(LOG_DEBUG, "mode update:%d\n", g_amr_motion.control_mode_update);
	motor_left = &g_amr_motion.motor_left;
	motor_right = &g_amr_motion.motor_right;
	motor_driver_init();
	syslog(LOG_INFO,"motion task start running! \n");
	while (1)
	{
		usleep(1000000/CONTROL_FREQUENCY);
		clock_gettime(CLOCK_MONOTONIC, &tp_start);

		if(OK == get_rc_goal_speed(&vx_rc, &vz_rc, &v_time)) {
			//smooth_speed_control(vx_rc, SMOOTH_STEP);
			inverse_kinematics(vx_rc, vz_rc, &motor_left->rpm_goal, &motor_right->rpm_goal);
			//driver_set_motor_speed(motor_left->rpm_goal, motor_right->rpm_goal);
			motor_speed_sync(motor_left->rpm_goal, motor_right->rpm_goal, v_time);
		}

		if(OK == get_ros_goal_speed(&vx_ros, &vz_ros)) {
			inverse_kinematics(vx_ros, vz_ros, &motor_left->rpm_goal, &motor_right->rpm_goal);
			motor_speed_sync(motor_left->rpm_goal, motor_right->rpm_goal, v_time);
		}

		if(OK == get_ros_goal_position(&x_pos, &z_pos, &sub_mode))
		{
			inverse_kinematics_pos(x_pos, z_pos, sub_mode, &left_goal, &right_goal);
			motor_position_set(left_goal, right_goal);
		}
		if(g_amr_motion.control_mode == CAR_VELOCITY_MODE)
		{
			g_amr_motion.velocity_zero = motor_velocity_zero();
			ret = motor_speed_read(&amr_motor_data);
			if (ret < 0) {
				syslog(LOG_WARNING, "speed read failed\n");
			}
			else {
				rpm_transfer_to_odom(&amr_motor_data);
			}
		}
		else if (g_amr_motion.control_mode == CAR_POSITION_MODE)
		{
			g_amr_motion.target_reached = motor_target_reached();
			motor_position_read(&amr_motor_data);
			count_transfer_to_odom(&amr_motor_data);
			amr_motor_data.target_reached = g_amr_motion.target_reached;
			if (g_amr_motion.target_reached && last_target_reached != g_amr_motion.target_reached)
			{
				g_amr_ros.position_enable = true;
			}
			last_target_reached = g_amr_motion.target_reached;
		}

		if (g_amr_motion.control_mode_update == true && g_amr_motion.target_control_mode != g_amr_motion.control_mode)
		{
			ret = motor_control_mode_switch(g_amr_motion.target_control_mode);
			if (ret == OK)
			{
				g_amr_motion.control_mode = g_amr_motion.target_control_mode;
				if (g_amr_motion.control_mode == CAR_POSITION_MODE)
				{
					g_amr_ros.position_enable = true;
				}
				if (g_amr_motion.control_mode == CAR_VELOCITY_MODE)
				{
					g_amr_ros.velocity_enable = true;
				}
				g_amr_motion.control_mode_update = false;
				g_amr_ros.notify_mode_switch = true;
				amr_motor_data.mode_switch = true;

			}
		}

		if (i++%20 == 0) {
			zlac8015d_driver_error_check();
			syslog(LOG_DEBUG, "mode:%d update:%d target:%d x:%.2f z:%.2f reach:%d vx:%.2f vz:%.2f stop:%d rpm_l:%d rpm_r:%d\n",
					g_amr_motion.control_mode, g_amr_motion.control_mode_update, g_amr_motion.target_control_mode, x_pos, z_pos,
					g_amr_motion.target_reached, vx_ros, vz_ros, g_amr_motion.velocity_zero, motor_left->rpm_goal, motor_right->rpm_goal);
			//syslog(LOG_INFO,"motor task runing: %d ns\n", delta);
		}
		clock_gettime(CLOCK_MONOTONIC, &tp_end);
		delta = tp_end.tv_nsec - tp_start.tv_nsec;
	    
		if (motor_need_stop() && g_amr_motion.quick_stop_enable)
		{
			if (g_amr_motion.motor_stop_once == false)
			{
				motor_quick_stop();
			}
			g_amr_motion.motor_stop_once = true;
		}
		else if (g_amr_motion.motor_stop_once == true)
		{
			motor_resume_enable(g_amr_motion.control_mode);
			g_amr_motion.motor_stop_once = false;
		}
    }

    return 0;
}
