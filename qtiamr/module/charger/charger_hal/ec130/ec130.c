/***************************************************************************
* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <sched.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include <nuttx/can/can.h>
#include <syslog.h>

#include "charger_hal.h"
#include "ec130.h"
#include "voltage_adc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CONFIG_EXAMPLES_CAN_DEVPATH "/dev/can1"
#define AUTO_CHARGER_MSG_ID         0x182
#define CHARGING_CURR_UNIT          0.033
#define WHELL_SPEED_VX_DIV          0.67
#define WHELL_SPEED_VZ_DIV          0.67
#define CRITICAL_ERROR_DELAY        2000000
#define WHEEL_SPEED_UNIT_TRANS      0.001

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ec130_raw_data_s
{
  uint8_t vx_h;
  uint8_t vx_l;
  uint8_t vy_h;
  uint8_t vy_l;
  uint8_t vz_h;
  uint8_t vz_l;
  uint8_t flags;
  uint8_t current;
};

struct ec130_data_s
{
  float vx;
  float vz;
  float current;
  bool  is_infrared;
  bool  is_charging;
} __attribute__((aligned(4)));

struct ec130_dev_s
{
  bool                initialized;
  pthread_mutex_t     ec_mutex;
  struct ec130_data_s ec130_data;
} __attribute__((aligned(4)));

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void    ec130_mutex_init(pthread_mutex_t *ec_mutex);
static void    ec130_mutex_lock(pthread_mutex_t *ec_mutex);
static void    ec130_mutex_unlock(pthread_mutex_t *ec_mutex);
static void    ec130_raw_data_parse(struct ec130_raw_data_s *buff);
static int32_t ec130_get_can_data(void);
static int32_t ec130_get_wheel_speed(float *vx, float *vz);
static int32_t ec130_get_current(float *current);
static int32_t ec130_infrared_signal(bool *stats);
static int32_t ec130_is_charging(bool *stats);
static int32_t ec130_get_all_stat(float *voltage, float *current, bool *infrared_stat, bool *is_charging);
static int32_t ec130_driver_init(void);

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct ec130_dev_s g_ec130_dev;

/****************************************************************************
 * Private Data
 ****************************************************************************/

int32_t g_print_count;

static struct charger_drv_ops_s ec130_drv_ops = {
  .get_voltage           = adc_get_voltage,
  .get_current           = ec130_get_current,
  .get_speed             = ec130_get_wheel_speed,
  .get_is_charging_stats = ec130_is_charging,
  .get_pile_signal_stats = ec130_infrared_signal,
  .get_all_stats         = ec130_get_all_stat,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void ec130_mutex_init(pthread_mutex_t *ec_mutex)
{
  int status;
  status = pthread_mutex_init(ec_mutex, NULL);
  if (status != 0)
    {
      syslog(LOG_ERR, "CHARGER: ec130_mutex_init: ERROR pthread_mutex_init failed, status=%d\n", status);
      ASSERT(false);
    }
}

static void ec130_mutex_lock(pthread_mutex_t *ec_mutex)
{
  int status;
  status = pthread_mutex_lock(ec_mutex);
  if (status != 0)
    {
      syslog(LOG_ERR, "CHARGER: ec130_mutex_lock: ERROR pthread_mutex_lock failed, status=%d\n", status);
      ASSERT(false);
    }
}

static void ec130_mutex_unlock(pthread_mutex_t *ec_mutex)
{
  int status;
  status = pthread_mutex_unlock(ec_mutex);
  if (status != 0)
    {
      syslog(LOG_ERR, "CHARGER: ec130_mutex_unlock: ERROR pthread_mutex_unlock failed, status=%d\n", status);
      ASSERT(false);
    }
}

static void ec130_raw_data_parse(struct ec130_raw_data_s *buff)
{

  struct ec130_data_s *ec130_data_p = &g_ec130_dev.ec130_data;

  /* 0.001m/s */
  ec130_data_p->vx = (((buff->vx_h << 8) + buff->vx_l) - ((buff->vx_h >> 7) * (0xFFFF + 1)));
  /* 0.001rad/s */
  ec130_data_p->vz          = (((buff->vz_h << 8) + buff->vz_l) - ((buff->vz_h >> 7) * (0xFFFF + 1)));
  ec130_data_p->is_infrared = buff->flags & (0x01 << 1);
  ec130_data_p->is_charging = buff->flags & (0x01);
  ec130_data_p->current     = (buff->current - ((buff->current >> 7) * (0xFF + 1))) * CHARGING_CURR_UNIT;

  if (ec130_data_p->is_infrared == 0)
    {
      ec130_data_p->vx = 0;
      ec130_data_p->vz = 0;
    }

  if (ec130_data_p->is_charging == 1)
    {
      ec130_data_p->vx = 0;
      ec130_data_p->vz = 0;
    }
  if ((g_print_count++ % 10000) == 0)
    {
      g_print_count = 1;
      syslog(LOG_DEBUG, "receive data: \n");
      syslog(LOG_DEBUG, "vx_h = 0x%02x ", buff->vx_h);
      syslog(LOG_DEBUG, "vx_l = 0x%02x \n", buff->vx_l);
      syslog(LOG_DEBUG, "vz_h = 0x%02x ", buff->vz_h);
      syslog(LOG_DEBUG, "vz_l = 0x%02x \n", buff->vz_l);
      syslog(LOG_DEBUG, "flags = 0x%02x \n", buff->flags);
      syslog(LOG_DEBUG, "current = 0x%02x \n", buff->current);
      syslog(LOG_DEBUG, "parse data: \n");
      syslog(LOG_DEBUG, "vx = %.2fmm/s, vz = %.2frad/s\n", ec130_data_p->vx, ec130_data_p->vz);
      syslog(LOG_DEBUG, "If_infrared = %d, If_charging = %d\n", ec130_data_p->is_infrared, ec130_data_p->is_charging);
      syslog(LOG_DEBUG, "current = %.2fA\n", ec130_data_p->current);
    }
  return;
}

static int32_t ec130_get_can_data(void)
{
  struct ec130_dev_s *ec130_dev_p = &g_ec130_dev;
  int                 fd;
  ssize_t             nbytes;
  struct can_msg_s    rxmsg;
  long                nmsgs = 5;
  long                msgno;

  if (ec130_dev_p->initialized != TRUE)
    {
      syslog(LOG_ERR, "ec130_get_can_data: ERROR: driver NOT initialized!\n");
      usleep(CRITICAL_ERROR_DELAY);
      return ERROR;
    }

  fd = open(CONFIG_EXAMPLES_CAN_DEVPATH, O_RDONLY);
  if (fd < 0)
    {
      syslog(LOG_ERR, "ERROR: open %s failed: %d\n", CONFIG_EXAMPLES_CAN_DEVPATH, errno);
      return ERROR;
    }
  for (msgno = 0; !nmsgs || msgno < nmsgs; msgno++)
    {
      /* Flush any output before the loop entered or from the previous pass through the loop. */
      fflush(stdout);
      nbytes = read(fd, &rxmsg, sizeof(struct can_msg_s));
      if (nbytes < CAN_MSGLEN(0) || nbytes > sizeof(struct can_msg_s))
        {
          syslog(LOG_ERR, "ec130_get_can_data:ERROR: read(%ld) returned %ld\n", (long)sizeof(struct can_msg_s), (long)nbytes);
          close(fd);
          return ERROR;
        }
      //syslog(LOG_ERR, "  ID: %4u DLC: %u\n", rxmsg.cm_hdr.ch_id, rxmsg.cm_hdr.ch_dlc);
      if (rxmsg.cm_hdr.ch_id == AUTO_CHARGER_MSG_ID)
        {
          ec130_raw_data_parse((struct ec130_raw_data_s *)&rxmsg.cm_data);
          goto out;
        }
    }
out:
  close(fd);
  return OK;
}

static int32_t ec130_get_wheel_speed(float *vx, float *vz)
{
  struct ec130_data_s *ec130_data_p = &g_ec130_dev.ec130_data;
  pthread_mutex_t *    ec_mutex     = &g_ec130_dev.ec_mutex;
  if (g_ec130_dev.initialized != TRUE)
    {
      syslog(LOG_ERR, "ec130_get_wheel_speed:ERROR: driver has not been initialized\n");
      return ERROR;
    }

  ec130_mutex_lock(ec_mutex);
  if (ec130_get_can_data() < 0)
    {
      syslog(LOG_ERR, "ec130_get_wheel_speed:ERROR: get can data error\n");
      return ERROR;
    }
  /*mm/s to m/s*/
  *vx = (ec130_data_p->vx * WHEEL_SPEED_UNIT_TRANS) * WHELL_SPEED_VX_DIV;
  /*0.001rad/s to 1rad/s*/
  *vz = (ec130_data_p->vz * WHEEL_SPEED_UNIT_TRANS) * WHELL_SPEED_VZ_DIV;

  ec130_mutex_unlock(ec_mutex);
  return OK;
}

static int32_t ec130_get_current(float *current)
{
  struct ec130_data_s *ec130_data_p = &g_ec130_dev.ec130_data;
  pthread_mutex_t *    ec_mutex     = &g_ec130_dev.ec_mutex;

  if (g_ec130_dev.initialized != TRUE)
    {
      syslog(LOG_ERR, "ec130_get_current:ERROR: driver has not been initialized\n");
      return ERROR;
    }

  ec130_mutex_lock(ec_mutex);
  if (ec130_get_can_data() < 0)
    {
      syslog(LOG_ERR, "ec130_get_current:ERROR: get can data error\n");
      return ERROR;
    }
  *current = ec130_data_p->current;
  ec130_mutex_unlock(ec_mutex);
  return OK;
}

static int32_t ec130_infrared_signal(bool *stats)
{
  struct ec130_data_s *ec130_data_p = &g_ec130_dev.ec130_data;
  pthread_mutex_t *    ec_mutex     = &g_ec130_dev.ec_mutex;
  if (g_ec130_dev.initialized != TRUE)
    {
      syslog(LOG_ERR, "ec130_infrared_signal:ERROR: driver has not been initialized\n");
      return ERROR;
    }

  ec130_mutex_lock(ec_mutex);
  if (ec130_get_can_data() < 0)
    {
      syslog(LOG_ERR, "ec130_infrared_sig_stat:ERROR: get can data error\n");
      return ERROR;
    }
  *stats = ec130_data_p->is_infrared;
  ec130_mutex_unlock(ec_mutex);
  return OK;
}

static int32_t ec130_is_charging(bool *stats)
{
  struct ec130_data_s *ec130_data_p = &g_ec130_dev.ec130_data;
  pthread_mutex_t *    ec_mutex     = &g_ec130_dev.ec_mutex;
  if (g_ec130_dev.initialized != TRUE)
    {
      syslog(LOG_ERR, "ec130_is_charging:ERROR: driver has not been initialized\n");
      return ERROR;
    }

  ec130_mutex_lock(ec_mutex);
  if (ec130_get_can_data() < 0)
    {
      syslog(LOG_ERR, "ec130_is_charging_stat:ERROR: get can data error\n");
      return ERROR;
    }
  *stats = ec130_data_p->is_charging;
  ec130_mutex_unlock(ec_mutex);
  return OK;
}

static int32_t ec130_get_all_stat(float *voltage, float *current, bool *infrared_stat, bool *is_charging)
{
  struct ec130_data_s *ec130_data_p = &g_ec130_dev.ec130_data;
  pthread_mutex_t *    ec_mutex     = &g_ec130_dev.ec_mutex;
  if (g_ec130_dev.initialized != TRUE)
    {
      syslog(LOG_ERR, "ec130_get_all_stat:ERROR: driver has not been initialized\n");
      return ERROR;
    }
  if (voltage != NULL)
    {
      if (adc_get_voltage(voltage) != OK)
        {
          syslog(LOG_ERR, "ec130_get_all_stat:ERROR: get voltagte error\n");
          return ERROR;
        }
    }
  ec130_mutex_lock(ec_mutex);
  if (ec130_get_can_data() < 0)
    {
      syslog(LOG_ERR, "ec130_get_all_stat:ERROR: get can data error\n");
      return ERROR;
    }
  if (current != NULL)
    {
      *current = ec130_data_p->current;
    }
  if (infrared_stat != NULL)
    {
      *infrared_stat = ec130_data_p->is_infrared;
    }
  if (is_charging != NULL)
    {
      *is_charging = ec130_data_p->is_charging;
    }
  ec130_mutex_unlock(ec_mutex);

  return OK;
}

static int32_t ec130_driver_init(void)
{
  if (g_ec130_dev.initialized == TRUE)
    {
      syslog(LOG_ERR, "CHARGER: ec130_driver_init: has been Initialized!\n");
      return ERROR;
    }
  memset(&g_ec130_dev, 0, sizeof(struct ec130_dev_s));
  ec130_mutex_init(&g_ec130_dev.ec_mutex);

  charger_dev_ops_cb_register(&ec130_drv_ops);

  g_ec130_dev.initialized = TRUE;
  syslog(LOG_DEBUG, "CHARGER: ec130_driver_init: ec130 Initialize successfully!\n");

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int32_t ec130_and_adc_driver_init(void)
{

  if (charger_voltage_adc_init() != OK)
    {
      syslog(LOG_ERR, "CHARGER: ec130_and_adc_driver_init: charger_voltage_adc_init Initialize failed!\n");
      return ERROR;
    }
  if (ec130_driver_init() != OK)
    {
      syslog(LOG_ERR, "CHARGER: ec130_driver_init: charger_voltage_adc_init Initialize failed!\n");
      return ERROR;
    }
  return OK;
}
