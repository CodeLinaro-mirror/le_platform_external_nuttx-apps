/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#ifndef __EMERGENCY_AVOIDANCE_H
#define __EMERGENCY_AVOIDANCE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <stdint.h>

/********************************************************************************
 * Public Datas
 ********************************************************************************/
enum move_direction
{
	MOTIONLESS,
	FORWARD,
	BACKWARD,
	TURN_LEFT,
	TURN_RIGHT,
};

/********************************************************************************
 * Public Function Prototypes
 ********************************************************************************/
int emergency_main(int argc, char *argv[]);

#endif

