/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#ifndef __APP_ACKERMAN_GPIO_H
#define __APP_ACKERMAN_GPIO_H
/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <stdio.h>
#include <stdint.h>

#ifdef CONFIG_APP_ACKERMAN

#include "car_pwm.h"
#include "car_encoder.h"


#define MOTOR_A_GP1 "/dev/gpio2"
#define MOTOR_A_GP2 "/dev/gpio3"

#define MOTOR_B_GP1 "/dev/gpio4"
#define MOTOR_B_GP2 "/dev/gpio5"


/* ackerm car DC motor  speed control unit gpios */
struct  motion_gpio_s 
{
	int  gpio_a1_fd;
	int  gpio_a2_fd;
	int  gpio_b1_fd;
	int  gpio_b2_fd;
	bool  right_motor_direction;  /* true:move; false:back  */
	bool  left_motor_direction;
};

int gpio_motion_init(struct  motion_gpio_s **ackerman);
void gpio_motion_deinit(void);

int car_motor_direction(struct  motion_gpio_s *ackerman,
						int32_t V_right, int32_t V_left);


#endif
#endif

