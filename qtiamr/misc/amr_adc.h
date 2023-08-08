/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#ifndef __APP_ADC_H
#define __APP_ADC_H

#include <stdio.h>
#include <stdint.h>

struct car_adc_s
{
    bool initialized;
    FAR char *adc_devpath;
};

int amr_adc_init(void);
void amr_adc_deinit(void);

#endif

