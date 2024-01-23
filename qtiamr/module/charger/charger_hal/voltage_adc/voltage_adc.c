/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#include <stdio.h>
#include <pthread.h>
#include <syslog.h>
#include <signal.h>
#include <assert.h>
#include <nuttx/config.h>
#include <errno.h>
#include <stdlib.h>
#include <sched.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <nuttx/fs/fs.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>

#include "voltage_adc.h"


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ADC_VOLTAGE_CHANNEL  (10)
#define ADC_MAX_RANGE        (4096)
#define ADC_MAX_RANGE_HALF   (2048)
#define ADC_VOLT_PATH        "/dev/adc_power"
/* Actual 1 adc controller have 16 channels */
#define ADC_MAX_GROUPSIZE    (1)
#define ADC_BASE_REF_VOLT    (3.3)
#define ADC_VOLT_PER_COUNT   (ADC_BASE_REF_VOLT/ADC_MAX_RANGE)
#define ADC_VOTL_DIV         (11.0)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct volt_adc_dev_s
{
  bool              initialized;
  FAR char          *adc_devpath;
  uint8_t           channel;
  pthread_mutex_t   adc_mutex;
  struct            adc_msg_s samples[ADC_MAX_GROUPSIZE];   /* adc data */
}__attribute__((aligned(4)));

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int32_t get_adc_sample(uint32_t *pdata, struct volt_adc_dev_s *adc_dev_p);

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct volt_adc_dev_s g_volt_adc_dev;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Get adc sample */

static int32_t get_adc_sample(uint32_t *pdata, struct volt_adc_dev_s *adc_dev_p)
{

  struct adc_msg_s *sample;
  size_t readsize;
  uint8_t groups;
  ssize_t nbytes;
  int32_t fd = 0;
  int32_t i;
  int32_t ret;

  if (NULL == pdata)
  {
    goto err_out;
  }
  if (adc_dev_p->initialized)
  {

    fd = open(adc_dev_p->adc_devpath, O_RDONLY);

    sample = (struct adc_msg_s *)&adc_dev_p->samples;

    /* Issue the software trigger to start ADC conversion */
    ret = ioctl(fd, ANIOC_TRIGGER, 0);
    if (ret < 0)
    {
      printf("get_adc_sample: ANIOC_TRIGGER ioctl failed: %d\n", errno);
      goto err_out;
    }
    readsize = ADC_MAX_GROUPSIZE * sizeof(struct adc_msg_s);
    nbytes = read(fd, sample, readsize);
    /* Handle unexpected return values */
    if (nbytes < 0)
    {
      printf("get_adc_sample: read adc failed: %d\n", errno);
      goto err_out;
    }
    else if (nbytes == 0)
    {
      printf("get_adc_sample: No data read, Ignoring\n");
      goto err_out;
    }
    else
    { /* get right adc data */
      if (0 == (nbytes % sizeof(struct adc_msg_s)))
      {
        groups = nbytes / sizeof(struct adc_msg_s);

        for (i = 0; i < groups; i++)
        {
          if (adc_dev_p->channel == sample[i].am_channel)
          {
            *pdata = sample[i].am_data;
            break;
          }
        }
        if (i == groups)
        {
          printf("ERROR: adc channel not matched /n/n");
        }
      }
      else
          {
            printf("get_adc_sample: read(size:%d) data invalid\n ", nbytes);
        }
    }
    close(fd);
    return OK;
  }
err_out:
  close(fd);
  return ERROR;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int32_t adc_get_voltage(float *voltage)
{
  struct volt_adc_dev_s *adc_dev_p = &g_volt_adc_dev;
  pthread_mutex_t *adc_mutex = &g_volt_adc_dev.adc_mutex;
  int32_t status;
  uint32_t volt_data = 0;
  int32_t ret;

  if (adc_dev_p->initialized != TRUE)
    {
      syslog(LOG_ERR, "CHARGER: adc_get_voltage: has not been Initialized!\n");
      return ERROR;
    }

  status = pthread_mutex_lock(adc_mutex);
   if (status != 0)
     {
       syslog(LOG_ERR,"CHARGER: adc_get_voltage: ERROR pthread_mutex_lock failed, status=%ld\n", status);
       ASSERT(false);
     }
  if (get_adc_sample(&volt_data, adc_dev_p) == OK)
  {
    *voltage = (((float)volt_data) * ADC_VOLT_PER_COUNT) * ADC_VOTL_DIV;
    ret = OK;
  }
  else
  {
    ret = ERROR;
  }
  status = pthread_mutex_unlock(adc_mutex);
 if (status != 0)
   {
     syslog(LOG_ERR,"CHARGER: adc_get_voltage: ERROR pthread_mutex_unlock failed, status=%ld\n", status);
     ASSERT(false);
   }

  return ret;
}

int32_t charger_voltage_adc_init(void)
{
  struct volt_adc_dev_s *adc_dev_p = &g_volt_adc_dev;
  pthread_mutex_t *adc_mutex = &g_volt_adc_dev.adc_mutex;
  int status;

  if (adc_dev_p->initialized == TRUE)
  {
    syslog(LOG_ERR, "CHARGER: charger_voltage_adc_init: has been Initialized!\n");
    return ERROR;
  }
  memset(adc_dev_p, 0, sizeof(struct volt_adc_dev_s));

  status = pthread_mutex_init(adc_mutex, NULL);
   if (status != 0)
     {
       syslog(LOG_ERR,"CHARGER: charger_voltage_adc_init: ERROR pthread_mutex_init failed, status=%d\n", status);
       ASSERT(false);
     }

  adc_dev_p->adc_devpath = strdup(ADC_VOLT_PATH);
  adc_dev_p->channel = ADC_VOLTAGE_CHANNEL;

  adc_dev_p->initialized = TRUE;
  syslog(LOG_DEBUG, "CHARGER: charger_voltage_adc_init: voltage adc init successfully!\n");
  return OK;
}
