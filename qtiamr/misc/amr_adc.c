/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#include <stdio.h>
#include <errno.h>
#include <stdlib.h>

#include "amr_adc.h"

struct car_adc_s g_adc_power;

/* Init adc */
int amr_adc_init(void) {

    g_adc_power.adc_devpath = strdup(DEV_VOLTAGE);
    g_adc_power.initialized = true;

    return OK;
}

void amr_adc_deinit(void)
{
    return;
}
