/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/


#ifndef __APP_ACKERMAN_CAR_ENCODER_H
#define __APP_ACKERMAN_CAR_ENCODER_H
/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <stdio.h>
#include <stdint.h>

#ifdef CONFIG_APP_ACKERMAN

#define CAR_MOTOR_QE
#define CAR_MOTOR_QE_NUM 2  /* 1: qeA(right); 2:qeB(left) */
#define CAR_MOTOR_QE_A   0
#define CAR_MOTOR_QE_B   1


#define QE_MOTOR_A_DEV  "/dev/qeA"
#define QE_MOTOR_B_DEV  "/dev/qeB"


/** QE DEV **/
struct car_qe_s
{
  bool      initialized;
  FAR char  *devpath;
  bool      reset;
  int32_t   position;
  int 		qe_fd;
};

/** Read count and clear zero, count range 0~2^32 **/
int car_get_qe_count(int32_t *position,uint8_t index);
/** init QE  **/
int car_qe_init(struct car_qe_s **ackerman,uint8_t *encoder_num);
void car_qe_deinit(void);

#endif
#endif


