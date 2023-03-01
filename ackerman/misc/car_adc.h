/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#ifndef __APP_ACKERMAN_ADC_H
#define __APP_ACKERMAN_ADC_H
/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <nuttx/fs/fs.h>

#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>


#ifdef CONFIG_APP_ACKERMAN

/* channel 1: SERVO slides; channel 2: power voltage  */


#define ADC_SERVO_POWER			CAR_ADC1
#define ADC_CAR_MODE			CAR_ADC2

#define ADC_POWER_CHANNEL		(5)
#define ADC_SERVO_BIAS_CHANNEL	(1)

#define ADC_CAR_MODE_CHANNEL	(13)
#define ADC_MAX_RANGE			(4096)
#define ADC_MAX_RANGE_HALF			(2048)

#define DEV_SERVO_POWER   "/dev/adc_servo"
#define DEV_ADC_CAR       "/dev/adc_car_mode"

#define ADC_MAX_GROUPSIZE		(3) /* Actual 1 adc controller have 16 channels */  

enum enum_car_adc
{
  CAR_ADC1 = 1,
  CAR_ADC2
};

struct car_adc_s
{
  bool	initialized;
  FAR char *adc_devpath;
  struct adc_msg_s samples[ADC_MAX_GROUPSIZE];   /* adc data */
};

int get_power_voltage(float * voltage);
int get_servo_zero_bias(float *bias);
uint8_t get_car_mode(void);

int car_adc_init(uint8_t adc_num);
void car_adc_deinit(void);

#endif
#endif