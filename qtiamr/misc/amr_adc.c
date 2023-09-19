/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <sched.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include "amr_adc.h"

struct car_adc_s g_adc_power;

/* Get adc sample */
static int get_adc_sample(uint32_t *pdata, struct car_adc_s *adc, uint8_t channel){
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
		sample = &adc->samples;
		fd = open(adc->adc_devpath, O_RDONLY);
		if (fd < 0){
			printf("adc: open %s failed: %d\n", adc->adc_devpath, errno);
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

int get_power_voltage(float *voltage) {
	uint32_t data;
	int ret;

	if (g_adc_power.initialized) {
		ret = get_adc_sample(&data, &g_adc_power, ADC_POWER_CHANNEL);
		if (OK == ret){
			*voltage = ( (3.3 * (float)data) / ADC_MAX_RANGE)* 11.0;
			return ret;
		}
	}
	return ERROR;
}


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
