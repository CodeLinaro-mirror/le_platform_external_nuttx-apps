/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/


#ifndef __AVOID_MANAGEMENT_H
#define __AVOID_MANAGEMENT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>
#include <syslog.h>

#define SENSOR_MAX 7

enum sensor_type {
	FRONT,
	SIDE,
	BUTTOM,
	MAX
};

struct ultra_sensor_ops
{
	int(*read)(void *buff, uint8_t len, int fd);
	int(*write)(void *data, uint8_t len, int fd);
};

struct ultra_sensor
{
	enum sensor_type type;
	uint8_t addr;
	uint8_t threshold; /*unit:cm*/
	struct ultra_sensor_ops ops;
};


#endif

