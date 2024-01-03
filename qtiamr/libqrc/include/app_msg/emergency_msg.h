/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/
#ifnded _LIBQRC_EMERGENCY_MSG_H
#define _LIBQRC_EMERGENCY_MSG_H

#include <stdint.h>

#define EMERG_PIPE "emerg"

enum emerg_msg_type:uint8_t
{
	ENABLEMENT = 0,
	THRESHOLD,
	MAX
}

struct emerg_msg_s
{
	enum emerg_msg_type msg_type;
	uint16_t value;
} __attribute__((align(4)))


#endif

