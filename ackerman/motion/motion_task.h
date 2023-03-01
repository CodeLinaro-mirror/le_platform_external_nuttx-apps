/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#ifndef __APP_ACKERMAN_MOTION_TASK_H
#define __APP_ACKERMAN_MOTION_TASK_H
/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <stdio.h>
#include <stdint.h>

#ifdef CONFIG_APP_ACKERMAN

#include "car_pwm.h"
#include "car_encoder.h"
#include "car_gpio.h"


#define CAR_MOTION_PRIORITY  (220)
#define CAR_MOTION_STACKSIZE  (16*2048)

/* motion task running frequency */
#define CONTROL_FREQUENCY    (100)

/*********************Control parameter *********************************/
#define MAX_SPEED            (2.0f)   /* actual is 1.82 m/s */
#define MAX_ANGLE           (0.52f)   /* angle */
#define SERVO_PWM_ZERO        (1500)
#define MAX_SERVO_PWM        (1900)
#define MIN_SERVO_PWM        (800)

#define PID_SAMPLE_SIZE      (2)        /* PID keep sample size */


#define KP_DEFAULT           (41.6f)
#define KI_DEFAULT           (41.6f)

#define PI_MATH              (3.1416f)

#define SENIOR_AKM_MIN_TURN_RADIUS  (0.75) /* 0.75m */

/*********************Car parameters **************************/

/* car's wheel diameter(/m) */
#define WHEEL_DIAMETER (0.215f)

/* encoder accuracy 500 pulses one cycle */
#define ENCODER_ACCURACY        (500)  

/* reducer ratio */
#define MD36N_51                (51)

/* Encoder crossover (1,2,4)*/
#define ENCODER_MULTI        (2)

#define ACKERMAN_AXLES_SPACE    (0.322f)
#define ACKERMAN_WHEEL_SPACE    (0.366f)

#define Slide2PWM_RATIO			(636.56)

#define ENCODER_PRECISION     (ENCODER_MULTI*ENCODER_ACCURACY*MD36N_51)

#define WHEEL_PERIMETER        (WHEEL_DIAMETER*PI_MATH)

#define MAX_PWM          (10000.0f)    //max amplitude, resitrict by PWM ARR

/****************************************************************/

/* car status check */
#define CAR_STATUS_OK   (true)


#define AMP_LIMIT(_val_, _min_, _max_)  \
	((_val_) < (_min_) ?  (_min_) : \
	((_val_) > (_max_) ? (_max_) : (_val_)))

/* PID control */
struct pid_cntl_s
{
    float KP;
    float KI;
    float KD;
    float err[PID_SAMPLE_SIZE];        /* the lagger index, the older err data */
};


struct motor_control_s
{
    struct pid_cntl_s pid_cntl;
    float v_goal;
    float v_actual;
    float pwm_ccr;  /* the lager CCR, the lager PWM duty, the higher motor speed */
};

struct servo_control_s
{
    float angle_goal_pwm;
	float angle_zero_pwm;
	float angle_bias_pwm;   /* servo biase */
};


/*define ackerm motion */
struct ackerman_motion_s
{
	struct motor_control_s motor_right;
	struct motor_control_s motor_left;
	struct servo_control_s servo_cntl;

	struct car_pwm_s *motor_pwm;
	struct car_pwm_s *servo_pwm;
	struct motion_gpio_s *gpios;
	struct car_adc_s *servo_adc; /* servo adc get slide position*/
	struct car_qe_s * encoders;
	uint8_t encoder_num;
	float smooth_speed;
};


void inverse_kinematics(float vx, float vz,
						float *vl, float *vr,
						struct servo_control_s *servo_cntl, bool rc_control);

int motion_task(int argc, char *argv[]);

int get_ros_goal_speed(float* vx, float* vz);

void get_servo_pwm(uint32_t *servo_pwm);
void get_motor_goal_speed(float *right, float *left);
void get_motor_actual_speed(float *right, float *left);

#endif
#endif

