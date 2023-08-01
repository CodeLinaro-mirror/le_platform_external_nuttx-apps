/****************************************************************************
* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
****************************************************************************/
#include <syslog.h>

/* roscom task */
int ros_com_task(int argc, char *argv[])
{
	int ret;
	int i = 0;


	ret = ros_com_init();
	if (ret != OK){
		syslog(LOG_INFO,"ros_com_task: init failed\n");
	}
	while(1){
		usleep(1000000/ROSCOM_FREQUENCY);
		/* read qrc data*/
		ros_cmd_receive();
		if (i++ % ROSCOM_FREQUENCY == 0 && parameter_stat == PAR_COM_INIT && i < ROSCOM_FREQUENCY*60){
			broadcast_reset_state();
		}

		if (parameter_stat == PAR_LOAD_DONE){
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
