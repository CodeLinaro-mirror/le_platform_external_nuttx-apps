/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/


#ifndef __APP_QCOMAMR_IMU_H
#define __APP_QCOMAMR_IMU_H
/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <nuttx/fs/fs.h>
#include <nuttx/sensors/mpu60x0.h>


#define CAR_IMU_PRIORITY    200
#define CAR_IMU_STACKSIZE  (2048)



#define IMU_DEV  "/dev/icm42688"

#define IMU_DATA_LEN_7          (7)


#define  MAX_IMU_DATA_ROW     (32767)

#define  MAX_IMU_GYRO     (500)   /*  ± 500 deg/sec */
#define  MAX_IMU_ACCEL      (2*9.8)		/* ± 2g */

#define  DEVIATION_COUNT  (1000)  /* the first 1000 data used to deviation */

struct imu_data_s
{
	float xa;
	float ya;
	float za;
	float xg;
	float yg;
	float zg;
};

struct car_imu_s{
	int   fd_imu;
	int16_t raw_data[IMU_DATA_LEN_7];  /* temp,xa,ya,za,zg,yg,zg */
	struct imu_data_s deviation_data;
	struct imu_data_s data;

	bool  data_ready_flag;
};



int imu_task(int argc, char *argv[]);
void get_imu_gyro_z(float *gyro_z);
void get_imu_data(struct imu_data_s *imu_data);


#endif
