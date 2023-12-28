/****************************************************************************
* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
****************************************************************************/

#ifndef __APP_QTIAMR_AUTO_CHARGING_H
#define __APP_QTIAMR_AUTO_CHARGING_H
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
#include "amr_adc.h"

#ifdef CONFIG_APP_QTIAMR

typedef struct
{
    uint8_t vx_h;
    uint8_t vx_l;
    uint8_t vy_h;
    uint8_t vy_l;
    uint8_t vz_h;
    uint8_t vz_l;
    uint8_t flags;
    uint8_t current;

}infrared_dock_cmd_s;

typedef struct
{
    float vx;
    float vz;
    float current;
    bool If_infrared;
    bool If_charging;
    uint32_t timestamp;
}audo_charging_data_s;

int auto_charging_task(int argc, char *argv[]);
int get_charging_goal_speed(float* vx, float* vz, uint32_t* vt);
#endif
#endif
