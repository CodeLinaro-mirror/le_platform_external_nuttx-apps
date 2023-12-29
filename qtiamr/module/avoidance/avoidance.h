/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/
 #ifnded _MODULE_AVOIDANCE_H
 #define _MODULE_AVOIDANCE_H

 
 #include <stdio.h>
 #include <stdint.h>
 #include <syslog.h>

 struct ultra_client {
	uint8_t thres_front;
	uint8_t thres_side;
	uint8_t thres_bottom;
	int (*cb)(uint8_t type);
 }

 int get_ultra_sensor_params(void);
 int register_ultra_client(struct ultra_client * client);

 #endif

