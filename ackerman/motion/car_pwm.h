/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/


#ifndef __APP_ACKERMAN_CAR_PWM_H
#define __APP_ACKERMAN_CAR_PWM_H
/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <nuttx/timers/pwm.h>

#ifdef CONFIG_APP_ACKERMAN

/** motor PWM num **/
#define  CAR_PWM_MOTOR 8
#define CAR_PWM_SERVO  4


#define DC_MOTOR_PWM_DEV  "/dev/pwm8"
#define DC_MOTOR_PWM_FREQUENCY 10000   /* DC motor  10kHZ */

enum car_channel
{
  CAR_CHANNEL_1 = 1,
  CAR_CHANNEL_2,
  CAR_CHANNEL_3,
  CAR_CHANNEL_4
};

#define CAR_PWM_MOTOR_A_CHANNEL  CAR_CHANNEL_2   /* motor A used tim8 channel 1 */
#define CAR_PWM_MOTOR_B_CHANNEL  CAR_CHANNEL_1   /* motor B used tim8 channel 2 */


#define SERVO_PWM_DEV  "/dev/pwm4"
#define SERVO_PWM_FREQUENCY		(100)     /* Servo pwm frequency 100HZ */
#define CAR_PWM_SERVO_CHANNEL  CAR_CHANNEL_2
#define SERVO_ZERO_DUTY 1500

/** PWM structure**/

struct car_pwm_s 
{
	struct pwm_info_s motor_pwm;
	int    pwm_fd;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
int car_pwm_init(struct car_pwm_s **ackerman, int pwm_num);
void car_pwm_deinit(void);


int car_pwm_duty_config(uint8_t pwm_num,
					struct pwm_chan_s *channels,
					uint8_t len);
					
void car_get_pwm_status(struct car_pwm_s *ppwm,
					struct pwm_chan_s * channels,
					uint8_t len);
#endif
#endif