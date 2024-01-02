/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/
#include <stdio.h>
#include <syslog.h>

#include "avoidance.h"

 int get_ultra_sensor_params()
{
	return 0;
}


 int avoidance_main(int argc, char *argv[])
 {
 	ret = 0;
	get_ultra_sensor_params();
	
	syslog(LOG_INFO,"ultrasound management init done\n");
	return ret;
 }

