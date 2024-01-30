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
	SENSOR_TYPE_MAX,
};

struct avoid_sensor
{
	enum sensor_type type;
	uint8_t addr;
}__attribute__((aligned(4)));


/****************************************************************************
 * Public data
 ****************************************************************************/
extern uint8_t ultras_num;
extern const struct avoid_sensor g_sensor_list[SENSOR_MAX];

/****************************************************************************
 * Public functon prototypes
 ****************************************************************************/

int avoid_init(void);
int avoid_management_thread(int argc, char *argv[]);

#endif

