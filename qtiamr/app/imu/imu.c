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
#include <syslog.h>

#include <nuttx/timers/pwm.h>
#include <nuttx/ioexpander/gpio.h>
#include "imu.h"

#define IMU_BIT(n)  (1 << (n))

static struct amr_imu_s  g_amr_imu;

static uint8_t deviation_count = DEVIATION_COUNT;
static uint8_t log_ctl;

/*  Sensor driver have configured:
 *  pwr_mgmt reset,
 *  gyro[ ± 1000 deg/sec ],
 *  set accel LPF at 184 Hz, gyro LPF at 188 Hz
 *  accel[ ± 8g ]
 */
int amr_imu_init(void)
{
	int ret;
	int fd;

	/* open imu fd */
	fd = open(IMU_DEV, O_RDWR);
	if (fd < 0){
		syslog(LOG_INFO, "amr_imu_init: open %s failed: %d\n", IMU_DEV, errno);
		return fd;
	}
	syslog(LOG_INFO,"amr_imu_init: open   %s\n",IMU_DEV);
	g_amr_imu.fd_imu = fd;

	return OK;
}

void amr_imu_deinit(void)
{
	close(g_amr_imu.fd_imu);
}

int amr_imu_check(void)
{
	if(g_amr_imu.fd_imu)
		return OK;
	else
		return ERROR;
}

void get_imu_gyro_z(float *gyro_z)
{
	*gyro_z = g_amr_imu.data.zg;
}

uint8_t* get_imu_data(void)
{
	return &g_amr_imu.raw_data[2];
}

/* change imu data from row data to actual acceleration & gyro;
 * x: amr's left & right(+) , y: amr's front & rear(+) */
static int imu_data_transform(int16_t *raw_imu_data, uint8_t len){

	struct imu_data_s *data;
	struct imu_data_s *d_data;

	if (NULL == raw_imu_data)
	{
		return ERROR;
	}
	data = &g_amr_imu.data;
	if (len != IMU_DATA_LEN_7)
	{
		return ERROR;
	}
	data->xa = ((float)raw_imu_data[1]/MAX_IMU_DATA_ROW)*MAX_IMU_ACCEL;
	data->ya = ((float)raw_imu_data[2]/MAX_IMU_DATA_ROW)*MAX_IMU_ACCEL;
	data->za = ((float)raw_imu_data[3]/MAX_IMU_DATA_ROW)*MAX_IMU_ACCEL;

	data->xg = ((float)raw_imu_data[4]/MAX_IMU_DATA_ROW)*MAX_IMU_GYRO;
	data->yg = ((float)raw_imu_data[5]/MAX_IMU_DATA_ROW)*MAX_IMU_GYRO;
	data->zg = ((float)raw_imu_data[6]/MAX_IMU_DATA_ROW)*MAX_IMU_GYRO;

	if (++log_ctl % 100 == 0) {
	    syslog(LOG_DEBUG, "imu data: accel speed x:%.2f, y:%.2f, z %.2f \n",data->xa, data->ya, data->za);
	    syslog(LOG_DEBUG, "imu data: gyro speed x:%.2f, y:%.2f, z %.2f \n\n",data->xg, data->yg, data->zg);
		log_ctl = 0;
	}
	/* zero bias*/
	if (deviation_count == 0){ 
		d_data = &g_amr_imu.deviation_data;
		data->xa = data->xa - d_data->xa;
		data->ya = data->ya - d_data->ya;
		data->za = data->za - d_data->za;
		data->xg = data->xg - d_data->xg;
		data->yg = data->yg - d_data->yg;
		data->zg = data->zg - d_data->zg;
		//syslog(LOG_INFO, "imu data: actual speed x:%f, y:%f,z %f \n\n", data->xa, data->ya, data->za);
	}

	return OK;
}

int imu_task(int argc, char *argv[]){
	int ret, i;
	int fd;
	
	size_t imu_data_size;
	struct imu_data_s *data;
	struct imu_data_s *deviation_data;
	uint8_t buf_len = IMU_DATA_LEN_7 *2;
	uint16_t tempbuff[IMU_DATA_LEN_7] = {0}; /* 1*2 temp; 3*2 acceleration; 3*2 gyro; */
	ret = amr_imu_init();
	if (ret != OK){
		syslog(LOG_INFO,"amr_imu_task: init failed\n");
	}
	fd = g_amr_imu.fd_imu;
	data = &g_amr_imu.data;
	deviation_data = &g_amr_imu.deviation_data;

    while(true)
    {
        /* read imu data*/
        usleep(1000000/IMU_READ_FREQ);  //frequency is 50HZ

        ret = read(fd, g_amr_imu.raw_data, buf_len);
        if (ret != buf_len)
        {
            syslog(LOG_INFO,"amr_imu_task: get data failed : %u \n",ret);
        }

		//get the raw data
        if (ret == buf_len)
        {
			for(i = 0; i < IMU_DATA_LEN_7; i++){
				tempbuff[i] = ((int16_t) ((uint16_t) g_amr_imu.raw_data[ 2 * i ] << 8) | g_amr_imu.raw_data[ 2 * i + 1 ]);
			}
        }

		imu_data_transform(tempbuff, IMU_DATA_LEN_7);
	}
	amr_imu_deinit();
	return ret;
}
