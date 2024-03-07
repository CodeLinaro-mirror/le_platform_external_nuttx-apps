/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <stdint.h>
#include <nuttx/fs/fs.h>
#include <nuttx/sensors/icm42688.h>

#include "imu.h"
#include "qrc_msg_management.h"
#include "main.h"
#include "config_msg.h"


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define IMU_BIT(n)  (1 << (n))
#define IMU_READ_FREQ   50 //default 50HZ
#define AMR_IMU_PRIORITY   200
#define AMR_IMU_STACKSIZE  (2048)
#define IMU_DEV  "/dev/icm"
#define MAX_IMU_DATA_ROW     (32767)
#define MAX_IMU_GYRO     (1000)   /* ± 1000 deg/sec */
#define MAX_IMU_ACCEL    (8*9.8) /* ± 8g */
#define DEVIATION_COUNT  (100)  /* the first 100 data used for deviation */
#define IMU_ENABLED      (1)

#define GRAVITY_ACC    (9.78)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static int amr_imu_init(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/
static struct imu_pkg_s  g_amr_imu;
static struct qrc_pipe_s *g_imu_pipe = NULL;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/*  Sensor driver have configured:
 *  pwr_mgmt reset,
 *  gyro[ ± 1000 deg/sec ],
 *  set accel LPF at 184 Hz, gyro LPF at 188 Hz
 *  accel[ ± 8g ]
 */
int amr_imu_init(void)
{
  int fd;

  /* open imu fd */
  fd = open(IMU_DEV, O_RDWR);
  if (fd < 0)
    {
      syslog(LOG_INFO, "amr_imu_init: open %s failed: %d\n", IMU_DEV, errno);
      return fd;
    }

  g_amr_imu.fd_imu = fd;

  return OK;
}

void amr_imu_deinit(void)
{
  close(g_amr_imu.fd_imu);
}

/*
 * first convert imu data from raw to actual acceleration & gyro;
 * x: amr's left & right(+) , y: amr's front & rear(+)
 * then send data to upper system
 */
static int send_imu_data(int16_t *raw_data, uint8_t len, struct timespec ts)
{
  struct imu_data_s *data = &g_amr_imu.data;
  struct imu_data_s *d_data = &g_amr_imu.deviation_data;
  struct imu_msg_s imu_msg;
  static int deviation_count = DEVIATION_COUNT;

  if (NULL == raw_data)
    {
      return ERROR;
    }

  if (len != IMU_DATA_LEN_7)
    {
      return ERROR;
    }

  data->xa = ((float)raw_data[1]) / MAX_IMU_DATA_ROW * MAX_IMU_ACCEL;
  data->ya = ((float)raw_data[2]) / MAX_IMU_DATA_ROW * MAX_IMU_ACCEL;
  data->za = ((float)raw_data[3]) / MAX_IMU_DATA_ROW * MAX_IMU_ACCEL;

  data->xg = ((float)raw_data[4]) / MAX_IMU_DATA_ROW * MAX_IMU_GYRO;
  data->yg = ((float)raw_data[5]) / MAX_IMU_DATA_ROW * MAX_IMU_GYRO;
  data->zg = ((float)raw_data[6]) / MAX_IMU_DATA_ROW * MAX_IMU_GYRO;

  /*
  syslog(LOG_INFO,"imu:acc x:%.3f, y:%.3f,z:%.3f\n",
                    data->xa, data->ya, data->za);
  syslog(LOG_INFO,"imu:gyr x:%.3f, y:%.3f,z:%.3f\n",
                    data->xg, data->yg, data->zg);
  */

  if (deviation_count == 0)
  {
    imu_msg.data.xa = data->xa - d_data->xa;
    imu_msg.data.ya = data->ya - d_data->ya;
    imu_msg.data.za = data->za - d_data->za + GRAVITY_ACC;
    imu_msg.data.xg = data->xg - d_data->xg;
    imu_msg.data.yg = data->yg - d_data->yg;
    imu_msg.data.zg = data->zg - d_data->zg;
    imu_msg.sec = ts.tv_sec;
    imu_msg.ns = ts.tv_nsec;
    qrc_write(g_imu_pipe, (void *)&imu_msg, sizeof(struct imu_msg_s), false);
  }
  else
  {
    d_data->xa += data->xa;
    d_data->ya += data->ya;
    d_data->za += data->za;
    d_data->xg += data->xg;
    d_data->yg += data->yg;
    d_data->zg += data->zg;

    deviation_count--;
    if (deviation_count == 0)
    {
      d_data->xa /= DEVIATION_COUNT;
      d_data->ya /= DEVIATION_COUNT;
      d_data->za /= DEVIATION_COUNT;
      d_data->xg /= DEVIATION_COUNT;
      d_data->yg /= DEVIATION_COUNT;
      d_data->zg /= DEVIATION_COUNT;
    }
  }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imu task function
 ****************************************************************************/
int imu_task(int argc, char *argv[])
{
  char pipe_name[] = IMU_PIPE;
  int ret, i;
  uint8_t buf_len = IMU_DATA_LEN_7 *2;
  int16_t tempbuff[IMU_DATA_LEN_7] = {0}; /* 0 temp;1-3 acc; 4-6 gyro; */
  struct timespec ts;
  struct config_sensor_s config;
  int result;

  /* get qrc pipe */
  g_imu_pipe =  qrc_get_pipe(pipe_name);
  if (g_imu_pipe == NULL)
    {
      /* notify error */
      config_notify_completed(false);
      return -1;
    }

  /* get IMU config */
  result = get_configuration_parameters(SENSOR, (void *)&config);
  if (result != OK)
    {
      syslog(LOG_INFO,"IMU: get config Failed \n");
      config_notify_completed(false);
      return result;
    }

  if (config.imu_enable != IMU_ENABLED)
    {
      config_notify_completed(true);
      syslog(LOG_INFO,"IMU disabled\n");
      return 0;
    }

  sleep(5);
  ret = amr_imu_init();
  if (ret != OK)
    {
      config_notify_completed(false);
      return -1;
    }

  /* notify init done */
  config_notify_completed(true);

  while (true)
    {
      /* read imu data*/
      usleep(1000000/IMU_READ_FREQ);

      ret = read(g_amr_imu.fd_imu, g_amr_imu.raw_data, buf_len);
      clock_gettime(CLOCK_REALTIME, &ts);
      if (ret != buf_len)
        {
          syslog(LOG_INFO,"amr_imu_task: read data failed : %u\n",ret);
          continue;
        }
      else
        {
          for(i = 0; i < IMU_DATA_LEN_7; i++)
            tempbuff[i] = (int16_t) ((g_amr_imu.raw_data[ 2 * i ] << 8) | g_amr_imu.raw_data[ 2 * i + 1 ]);
        }

      send_imu_data(tempbuff, IMU_DATA_LEN_7, ts);
    }

  amr_imu_deinit();
  return 0;
}