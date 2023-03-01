/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

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

#include "car_adc.h"


#ifdef ADC_SERVO_POWER
struct car_adc_s g_adc_servo;
#endif
#ifdef ADC_CAR_MODE
struct car_adc_s g_adc_carmode;
#endif

/* Get adc sample */
static int get_adc_sample(uint16_t *pdata, struct car_adc_s *adc, uint8_t channel){
	struct adc_msg_s *sample;
	size_t readsize;
	uint8_t groups;
	ssize_t nbytes;
	int fd;
	int ret, i;

	ret = ERROR;
	if (NULL == pdata && NULL == adc)
		return ERROR;

	if (adc->initialized){
		sample = &adc->samples;\
		fd = open(adc->adc_devpath, O_RDONLY);
		if (fd < 0){
			printf("adc: open %s failed: %d\n", DEV_SERVO_POWER, errno);
			return errno;
		}
		/* Issue the software trigger to start ADC conversion */
		ret = ioctl(fd, ANIOC_TRIGGER, 0);
		if (ret < 0) {
			printf("get_adc_sample: ANIOC_TRIGGER ioctl failed: %d\n", errno);
		}

		readsize = ADC_MAX_GROUPSIZE * sizeof(struct adc_msg_s);
		nbytes = read(fd, sample, readsize);
		/* Handle unexpected return values */
		if (nbytes < 0){
			printf("get_adc_sample: read adc failed: %d\n", errno);
		}
		else if (nbytes == 0) {
			printf("get_adc_sample: No data read, Ignoring\n");
		}
		else {/* get right adc data */
			if (0 == (nbytes % sizeof(struct adc_msg_s))) {
				groups = nbytes / sizeof(struct adc_msg_s);
				for (i = 0; i < groups; i++) {
					if (channel == sample[i].am_channel) {
						*pdata = sample[i].am_data;
						break;
					}
                }
				if (i == groups)
					printf("ERROR: adc channel not matched /n/n");
					
			}
			else
				printf("get_adc_sample: read(size:%d) data invalid\n ", nbytes);
		}
    }
	close(fd);
	return OK;
}

int get_power_voltage(float * voltage) {
	uint16_t data;
	int ret;

	if (g_adc_servo.initialized) {
		ret = get_adc_sample(&data, &g_adc_servo, ADC_POWER_CHANNEL);
		if (OK == ret){
			*voltage = ((float)data / ADC_MAX_RANGE)* 3.3 * 11.0;
			return ret;
		}
	}
	return ERROR;
}

int get_servo_zero_bias(float *bias) {
	uint16_t data;
	int ret;

	if (g_adc_servo.initialized) {
		ret = get_adc_sample(&data, &g_adc_servo, ADC_SERVO_BIAS_CHANNEL);
		if (OK == ret){
			*bias = ((float)data - ADC_MAX_RANGE_HALF) / 5;
			return ret;
		}
	}
	return ERROR;
}

uint8_t get_car_mode(void)
{
	return 1;
}

/* Init adc */
int car_adc_init(uint8_t adc_num) {

	switch(adc_num){
		case ADC_SERVO_POWER:
			/** servo has 2 adc channels for servo bias and power voltage **/
			g_adc_servo.adc_devpath = strdup(DEV_SERVO_POWER);
			g_adc_servo.initialized = true;
			break;
		case ADC_CAR_MODE:
			/** servo has 1 pwm channel **/
			g_adc_carmode.adc_devpath = strdup(DEV_ADC_CAR);
			g_adc_carmode.initialized = true;
			break;
		default:
			printf("car_adc_init: invalid adc number\n");
			break;
	}

	return OK;
}

void car_adc_deinit(void)
{
	return;
}
