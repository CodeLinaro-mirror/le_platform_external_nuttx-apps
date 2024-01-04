/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

volt_adc_dev_t g_volt_adc_dev;


/* Get adc sample */
static int get_adc_sample(uint32_t *pdata, volt_adc_dev_t adc_dev_p){

	struct adc_msg_s *sample;
	size_t readsize;
	uint8_t groups;
	ssize_t nbytes;
	int fd;
	int i;
	int ret;

	if (NULL == pdata){
		goto err_out;
	}
	if (adc_dev_p->initialized){
		fd = adc_dev_p->fd;
		sample = &adc_dev_p->samples;
	
		/* Issue the software trigger to start ADC conversion */
		ret = ioctl(fd, ANIOC_TRIGGER, 0);
		if (ret < 0) {
			printf("get_adc_sample: ANIOC_TRIGGER ioctl failed: %d\n", errno);
			goto err_out;
		}
		readsize = ADC_MAX_GROUPSIZE * sizeof(struct adc_msg_s);
		nbytes = read(fd, sample, readsize);
		/* Handle unexpected return values */
		if (nbytes < 0){
			printf("get_adc_sample: read adc failed: %d\n", errno);
			goto err_out;
		}
		else if (nbytes == 0) {
			printf("get_adc_sample: No data read, Ignoring\n");
		}
		else {/* get right adc data */
			if (0 == (nbytes % sizeof(struct adc_msg_s))){
				groups = nbytes / sizeof(struct adc_msg_s);
				
				for (i = 0; i < groups; i++){
					if (adc_dev_p->channel == sample[i].am_channel){
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
		return OK;
	}
err_out:
	return ERROR;
}

bool adc_get_voltage(float *voltage) {
	volt_adc_dev_t * adc_dev_p = &g_volt_adc_dev;
	uint32_t data;

	if (adc_dev_p->initialized){
		if(get_adc_sample(&data, adc_dev_p) == OK){
			*voltage = ( ((float)data) * ADC_VOLT_PER_COUNT)* ADC_VOTL_DIV;
			return OK;
		}
	}
	return ERROR;
}


bool charger_voltage_adc_init(void)
{
	volt_adc_dev_t * adc_dev_p = &g_volt_adc_dev;
	if(adc_dev_p->initialized == TRUE){
    syslog(LOG_ERR, "CHARGER: charger_voltage_adc_init: has been Initialized!\n");
		return ERROR;
	}
	memset(adc_dev_p, 0, sizeof(volt_adc_dev_t));
	adc_dev_p->adc_devpath = strdup(ADC_VOLT_PATH);
	adc_dev_p->channel = ADC_VOLTAGE_CHANNEL;
	adc_dev_p->fd = open(adc_dev_p->adc_devpath, O_RDONLY);
	if (adc_dev_p->fd < 0){
		syslog(LOG_ERR,"CHARGER: charger_voltage_adc_init: open %s failed: %d\n", adc_dev_p->adc_devpath, errno);
		return ERROR;
	}
    adc_dev_p->initialized = TRUE;
    syslog(LOG_DEBUG,"CHARGER: charger_voltage_adc_init: open %s successfully!\n", adc_dev_p->adc_devpath);
	return OK;
}

