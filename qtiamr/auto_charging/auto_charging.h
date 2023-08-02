/****************************************************************************
* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
****************************************************************************/

#ifndef __APP_AUTO_CHARGING_H
#define __APP_AUTO_CHARGING_H

#include <nuttx/config.h>
#include <stdio.h>
#include <errno.h>

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
