/***************************************************************************
* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
****************************************************************************/

#ifndef __APP_CHARGER_EC130_H
#define __APP_CHARGER_EC130_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <sched.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <stdbool.h>
#include <syslog.h>

#include <nuttx/can/can.h>

/********************************************************************************
 * Pre-processor Definitions
 ********************************************************************************/


#define CONFIG_EXAMPLES_CAN_DEVPATH "/dev/can1"
#define PRI_CAN_ID PRIu16
#define AUTO_CHARGER_MSG_ID 0x182
#define CHARGING_CURR_UNIT 0.033
#define WHELL_SPEED_VX_DIV	1500
#define WHELL_SPEED_VZ_DIV	800


typedef struct ec130_raw_data_s
{
    uint8_t vx_h;
    uint8_t vx_l;
    uint8_t vy_h;
    uint8_t vy_l;
    uint8_t vz_h;
    uint8_t vz_l;
    uint8_t flags;
    uint8_t current;

}ec130_raw_data_t;


typedef struct ec130_data_s {
    float vx;
    float vz;
    float current;
    bool If_infrared;
    bool If_charging;
    uint32_t timestamp;
}ec130_data_t;


typedef struct ec130_device_s {
	bool initialized;
	int fd;
	//charger_dev_t *charger_dev;
	ec130_data_t ec130_data;
}ec130_dev_t;

bool ec130_driver_init(charger_dev_t *charger_dev);


#endif /* __APP_CHARGER_EC130_H */

