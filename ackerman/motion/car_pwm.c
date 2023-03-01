/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <string.h>
#include <inttypes.h>

#include "car_pwm.h"

#ifdef CAR_PWM_MOTOR
static struct car_pwm_s g_motor_pwm;
#endif

#ifdef CAR_PWM_SERVO
static struct car_pwm_s g_servo_pwm;
#endif

static int car_pwm_config(struct car_pwm_s *ppwm, uint8_t pwm_num){
	int ret;
	int fd;
	struct pwm_info_s info;
	
	ret = ERROR;
	if (NULL == ppwm){
		printf("car_pwm_config: error: invalid point\n");
		return ret;
	}

	if (pwm_num == CAR_PWM_MOTOR || pwm_num == CAR_PWM_SERVO ){
		/* DC motor pwm configure */
		memcpy(&info, &ppwm->motor_pwm, sizeof(struct pwm_info_s));
		
		fd = ppwm->pwm_fd;

		ret = ioctl(fd, PWMIOC_SETCHARACTERISTICS,
              (unsigned long)((uintptr_t)&info));
		if (ret < 0)
		{
			printf("car_pwm_config: ioctl(PWMIOC_SETCHARACTERISTICS) failed: %d\n",
					errno);
			goto errout_with_dev;
		}

		ret = ioctl(fd, PWMIOC_START, 0);
		if (ret < 0)
		{
			printf("car_pwm_config: ioctl(PWMIOC_START) failed: %d\n", errno);
			goto errout_with_dev;
		}

		fflush(stdout);
		ret = OK;
	}

	return ret;

errout_with_dev:
  close(fd);
errout:
  fflush(stdout);
  return ERROR;
  
}

void car_get_pwm_status(struct car_pwm_s *ppwm,
					struct pwm_chan_s * channels,
					uint8_t len)
{
	int ret;
	int fd;
	struct pwm_info_s motor_pwm;

	ret = ERROR;
	if (NULL == ppwm || NULL == channels){
		printf("car_get_pwm_status: ERROR: NULL POINT\n");
		return;
	}

	fd = ppwm->pwm_fd;
	ret = ioctl(fd, PWMIOC_GETCHARACTERISTICS,
              (unsigned long)((uintptr_t)&motor_pwm));
	if (ret < 0){
		printf("car_get_pwm_status: ioctl(PWMIOC_GETCHARACTERISTICS) failed: %d\n",
				errno);
		goto errout_with_dev;
	}

	fflush(stdout);

	printf("car_get_pwm_status:  output "
         "with frequency: %" PRIu32 " duty: %08" PRIx32 "\n",
         motor_pwm.frequency, (uint32_t)motor_pwm.channels[channels[0].channel-1].duty);

	return OK;

errout_with_dev:
  close(fd);
errout:
  fflush(stdout);
  return ERROR;
}
					
					

/*******
 * configure pwm duty.
 * @channel : value range [1,4]
 * @duty : value range [0~10000]
 ********/
/* configure pwm duty  */
int car_pwm_duty_config(uint8_t pwm_num,
					struct pwm_chan_s * channels,
					uint8_t len){
	int ret,num;
	struct car_pwm_s *ppwm;
	
	ret = ERROR;

	switch(pwm_num){
		case CAR_PWM_MOTOR:
			ppwm = &g_motor_pwm;
			break;
		case CAR_PWM_SERVO:
			ppwm = &g_servo_pwm;
			break;
		default:
			ppwm =NULL;
			break;
	}

	if (NULL != ppwm &&NULL != channels){
		
		for (num = 0;num < len;num++){
			ppwm->motor_pwm.channels[channels[num].channel -1].duty =
							((uint32_t)channels[num].duty << 16) / 10000;
		}
		ret = car_pwm_config(ppwm,pwm_num);
	}

	return ret;
}

/* Init pwm*/
int car_pwm_init(struct car_pwm_s **ackerman, int pwm_num){
	int ret;
	ret = ERROR;
	
	switch(pwm_num){
		case CAR_PWM_MOTOR:
			/** TIM8 used 2 pwm channels for two motor **/
			g_motor_pwm.motor_pwm.frequency = DC_MOTOR_PWM_FREQUENCY;
			g_motor_pwm.motor_pwm.channels[CAR_PWM_MOTOR_A_CHANNEL -1].duty = 0;
			g_motor_pwm.motor_pwm.channels[CAR_PWM_MOTOR_A_CHANNEL - 1].channel =
										CAR_PWM_MOTOR_A_CHANNEL;
			g_motor_pwm.motor_pwm.channels[CAR_PWM_MOTOR_B_CHANNEL - 1].duty = 0;
			g_motor_pwm.motor_pwm.channels[CAR_PWM_MOTOR_B_CHANNEL - 1].channel =
										CAR_PWM_MOTOR_B_CHANNEL;
#ifdef CONFIG_PWM_PULSECOUNT
			g_motor_pwm.count = 0;
#endif
			g_motor_pwm.pwm_fd = open(DC_MOTOR_PWM_DEV, O_RDONLY);
			printf("car_pwm_init: open   %s\n",DC_MOTOR_PWM_DEV);
			if (g_motor_pwm.pwm_fd < 0){
				printf("car_pwm_init: open %s failed: %d\n",
									DC_MOTOR_PWM_DEV, errno);
				return errno;
			}
			ret = car_pwm_config(&g_motor_pwm,CAR_PWM_MOTOR);
			*ackerman = &g_motor_pwm;
			break;
		case CAR_PWM_SERVO:
			/** TIM4 used 1 pwm channel for servo **/
			g_servo_pwm.motor_pwm.frequency = 100;
			g_servo_pwm.motor_pwm.channels[CAR_PWM_SERVO_CHANNEL -1].duty = SERVO_ZERO_DUTY;
			g_servo_pwm.motor_pwm.channels[CAR_PWM_SERVO_CHANNEL - 1].channel =
										CAR_PWM_SERVO_CHANNEL;
#ifdef CONFIG_PWM_PULSECOUNT
			g_servo_pwm.count = 0;
#endif
			g_servo_pwm.pwm_fd = open(SERVO_PWM_DEV, O_RDONLY);
			printf("car_pwm_init: open   %s\n",SERVO_PWM_DEV);
			if (g_servo_pwm.pwm_fd < 0){
				printf("car_pwm_init: open %s failed: %d\n",
									SERVO_PWM_DEV, errno);
				return errno;
			}
			ret = car_pwm_config(&g_servo_pwm,CAR_PWM_SERVO);
			*ackerman = &g_servo_pwm;
			break;
		default:
			printf("car_pwm_init: invalid pwm number\n");
			break;

	}

	return ret;
}

void car_pwm_deinit(void)
{
	close(g_motor_pwm.pwm_fd);
	close(g_servo_pwm.pwm_fd);
}