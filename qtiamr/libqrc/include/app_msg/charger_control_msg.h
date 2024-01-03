/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/


#ifndef __APP_CHARGER_CONTROL_MSG_H
#define __APP_CHARGER_CONTROL_MSG_H
/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <stdio.h>
#include <stdint.h>

/********************************************************************************
 * Pre-processor Definitions
 ********************************************************************************/

#define CHARGER_MSG_PIPE "motion"

enum charger_ctl_cmd_e
{
	GET_VOLTAGE 	= 	0x01,
	GET_FULL_BATT_VOLT	=	0x02,
	GET_CURRENT 	= 	0x03,
	GET_PILE_STATS 	= 	0x04,
	GET_SM_STATS	=	0x05,
	START_CHARGING	=	0x06,
	STOP_CHARGING	=	0x07,
	GET_BATT_CAP	=	0x08
};


union cmd_val
{
  float		voltage;
  float		current;
  uint32_t	pile_stats;
  uint32_t	sm_stats;
};



struct charger_ctl_cmd_s
{
  uint32_t cmd_type;
  union cmd_val cmd_value;
}__attribute__((align(4)));


#endif /* __APP_CHARGER_CONTROL_MSG_H */

