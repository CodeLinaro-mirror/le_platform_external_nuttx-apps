/***************************************************************************
* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
****************************************************************************/

#include "auto_charging.h"

int auto_charging_task(int argc, char *argv[]){

    g_i =0;
    auto_charging_init();
    amr_adc_init();

    while(1){

        auto_charging_sync_power_voltage();
        auto_chanrging_get_data();
        usleep(100);
    }

    return 0;
}
