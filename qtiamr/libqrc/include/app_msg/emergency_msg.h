/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/
#ifndef _LIBQRC_EMERGENCY_MSG_H
#define _LIBQRC_EMERGENCY_MSG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define EMERG_PIPE "emerg"

/****************************************************************************
 * Public types
 ****************************************************************************/
enum emerg_msg_type
{
	ENABLEMENT,
	T_SENSOR,
};

struct emerg_msg_s
{
	enum emerg_msg_type msg_type;
	int value;
} __attribute__((aligned(4)));


#endif

