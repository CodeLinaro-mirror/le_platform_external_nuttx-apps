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

#include <nuttx/timers/pwm.h>
#include <nuttx/ioexpander/gpio.h>
#include "imu.h"

#define IMU_BIT(n)  (1 << (n))

static struct amr_imu_s  g_amr_imu;

static uint8_t deviation_count = DEVIATION_COUNT;

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
		printf("amr_imu_init: open %s failed: %d\n", IMU_DEV, errno);
		return fd;
	}
	printf("amr_imu_init: open   %s\n",IMU_DEV);
	g_amr_imu.fd_imu = fd;

	return OK;
}

void amr_imu_deinit(void)
{
	close(g_amr_imu.fd_imu);
}


void get_imu_gyro_z(float *gyro_z)
{
	*gyro_z = g_amr_imu.data.zg;
}

void get_imu_data(struct imu_data_s *imu_data)
{
	memcpy(imu_data, &g_amr_imu.data, sizeof(struct imu_data_s));
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
	printf("imu data: raw speed x:%f, y:%f,z %f \n\n",data->xa, data->ya, data->za);
	/* zero bias*/
	if (deviation_count == 0){ 
		d_data = &g_amr_imu.deviation_data;
		data->xa = data->xa - d_data->xa;
		data->ya = data->ya - d_data->ya;
		data->za = data->za - d_data->za;
		data->xg = data->xg - d_data->xg;
		data->yg = data->yg - d_data->yg;
		data->zg = data->zg - d_data->zg;
		printf("imu data: actual speed x:%f, y:%f,z %f \n\n", data->xa, data->ya, data->za);
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
	uint8_t tempbuff[buf_len]; /* 3*2 acceleration;1*2 temp; 3*2 gyro; */
	ret = amr_imu_init();
	if (ret != OK){
		printf("amr_imu_task: init failed\n");
	}
	fd = g_amr_imu.fd_imu;
	data = &g_amr_imu.data;
	deviation_data = &g_amr_imu.deviation_data;

	while(true)
	{
		/* read imu data*/
		usleep(10000);  //frequency is 100HZ
		for(i = 0; i < 14; i++)
		{
			tempbuff[i] = 0;
		}
		ret = read(fd, tempbuff, buf_len);
		if (ret != buf_len)
		{
			printf("amr_imu_task: get data failed : %u \n",ret);
		}
		for(i = 0; i < 14; i++)
		{
			printf("read tempbuff %d : %x\n", i, tempbuff[i]);
		}
		//get the row data
		if (ret == buf_len)
		{
			g_amr_imu.raw_data[0]=((int16_t) ((uint16_t) tempbuff[0] << 8) + tempbuff[1]);
			for(i = 1; i < 4; i++)
			{
				/* Concatenate H and L bits */
				g_amr_imu.raw_data[i]=((int16_t) ((uint16_t) tempbuff[2 * i] << 8) + tempbuff[2 * i + 1]);
				/* Convert to m/(s*s) */
				//g_amr_imu.raw_data[i]=g_amr_imu.raw_data[i]/32767*MAX_IMU_ACCEL;
			}
			for(i = 4; i < 7; i++)
			{
				/* Concatenate H and L bits */
				g_amr_imu.raw_data[i]=((int16_t) ((uint16_t) tempbuff[2 * i] << 8) + tempbuff[2 * i + 1]);
				//g_amr_imu.raw_data[i]=g_amr_imu.raw_data[i]/32767*MAX_IMU_GYRO;
			}
		}
		imu_data_transform(g_amr_imu.raw_data, IMU_DATA_LEN_7);
	}
	amr_imu_deinit();
	return ret;
}
