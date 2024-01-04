/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#ifndef __APP_CHARGER_VOLT_ADC_H
#define __APP_CHARGER_VOLT_ADC_H
/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <nuttx/fs/fs.h>

#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>

/********************************************************************************
 * Pre-processor Definitions
 ********************************************************************************/

#define ADC_VOLTAGE_CHANNEL			(10)
#define ADC_MAX_RANGE				(4096)
#define ADC_MAX_RANGE_HALF			(2048)
#define ADC_VOLT_PATH				"/dev/adc_power"
/* Actual 1 adc controller have 16 channels */
#define ADC_MAX_GROUPSIZE			(1)
#define ADC_BASE_REF_VOLT			(3.3)
#define ADC_VOLT_PER_COUNT			ADC_BASE_REF_VOLT/ADC_MAX_RANGE
#define ADC_VOTL_DIV				(11.0)

typedef struct charger_voltage_adc_dev_s
{
	bool        initialized;
	FAR char    *adc_devpath;
	uint8_t 	channel;
	int			fd;
	struct		adc_msg_s samples[ADC_MAX_GROUPSIZE];   /* adc data */
}volt_adc_dev_t;

bool adc_get_voltage(float *voltage);




#endif /* __APP_CHARGER_VOLT_ADC_H */

