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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define ULTRASOUND_DEV  "/dev/ttyS3"
#define SENSOR_MAX (7)
#define AVOID_WAKEUP (21)


/****************************************************************************
 * Public types
 ****************************************************************************/
enum sensor_type {
	FRONT,
	SIDE,
	BOTTOM,
};

struct avoid_sensor
{
	enum sensor_type type;
	uint8_t addr;
	uint8_t count;
}__attribute__((aligned(4)));

/****************************************************************************
 * Public functon prototypes
 ****************************************************************************/
struct avoid_sensor* get_ultra_sensor_list(void);

int avoid_init(void);
int avoid_management_thread(int argc, char *argv[]);

#endif

