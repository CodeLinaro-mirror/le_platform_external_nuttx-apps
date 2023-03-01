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
#include <syslog.h>

#include <nuttx/timers/pwm.h>
#include <nuttx/ioexpander/gpio.h>
#include <nuttx/arch.h>

#include "car_pwm.h"
#include "car_encoder.h"
#include "car_gpio.h"
#include "motion_task.h"
#include "ros_com.h"
#include "car_adc.h"


#include "motion_task.h"
#include "ros_com.h"
#include "rc.h"


#define PID_DEBUG

static struct ackerman_motion_s  g_ackerman_motion;


static float g_KP = 1000;
static float g_KI = 400;
#define SMOOTH_STEP (0.03) /* Max speed : Max accelerate = 1:1.5 */

static uint64_t g_lasttime;

/**********************************************************/

/* The PWM pulse width is proportional to the speed,
 * so the PID can be adjusted by the deviation value of the speed. */
static float incremental_PID(struct pid_cntl_s *pid, float err_k)
{
    float output;
    float *err = &pid->err[0];
    int32_t i;

	//err_k = fabsf(err_k);

    output = pid->KP * (err_k - err[0]) + pid->KI * err_k;
	//output = pid->KP * err_k;

    //shift err[] right to update err data
    for (i = PID_SAMPLE_SIZE - 1; i > 0 ; i--) {
        err[i] = err[i - 1];
    }
    err[0] = err_k;

    return output;
}

/**************************************************************************
* Function: Read the encoder value and calculate the wheel speed, unit m/s
* Speed = S/t = (encoder_count/ENCODER_PRECISION * WHEEL_PERIMETER) /(timer)
* Input   : 
* Output  : motor R&L actual speed.
**************************************************************************/
static void calculate_car_speed(float *speed_right, float *speed_left)
{
    //Retrieves the original data of the encoder
    int32_t position_a;
	int32_t position_b;
	struct timespec tp;
	uint64_t consume_timer;
	uint64_t timer;
	float time_sec;

	if (speed_right == NULL || speed_left == NULL) {
		printf("calculate_car_speed: error: Parameter is invalid\n");
		return;
	}

	car_get_qe_count(&position_a,CAR_MOTOR_QE_A);
	car_get_qe_count(&position_b,CAR_MOTOR_QE_B);
	clock_gettime(CLOCK_MONOTONIC, &tp);
	timer = ((uint64_t)(tp.tv_sec) * 1000000) + tp.tv_nsec / 1000;
	consume_timer = ( timer - g_lasttime);  /* (us) */
	time_sec = ((float)consume_timer)/ (float)1000000; /* second */
	g_lasttime = timer;
#ifdef ACKERMAN_DEBUG
	printf("position a: %d, b = %d time: %llu last =%llu \n",position_a,position_b,consume_timer,g_lasttime);
#endif
	*speed_right = ((float)position_a/ENCODER_PRECISION)* WHEEL_PERIMETER / time_sec;
	/* left encoder need change direction */
	*speed_left = -((float)position_b/ENCODER_PRECISION)* WHEEL_PERIMETER / time_sec; 
}

/**************************************************************************
Function: Smoothing the target velocity
Input   : goal velocity
Output  : none
**************************************************************************/
static void smooth_speed_control(float vx, float step)
{
    if (vx > g_ackerman_motion.smooth_speed)
        g_ackerman_motion.smooth_speed += step;
    else if (vx < g_ackerman_motion.smooth_speed)
        g_ackerman_motion.smooth_speed -= step;
    else
        g_ackerman_motion.smooth_speed = vx;

    if ((abs(vx) <0.1) && g_ackerman_motion.smooth_speed < (2.5 * step)
            && g_ackerman_motion.smooth_speed > (-2.5 * step))
        g_ackerman_motion.smooth_speed = vx;
}

/**************************************************************************
Function: Assign a value to the PWM register to control wheel speed and direction
Input   : PWM, range: 0~10000
Output  : none
**************************************************************************/
static void set_pwm(float pwm_right, float pwm_left, int32_t servo)
{
	struct pwm_chan_s motor_channels[2];
	struct pwm_chan_s servo_channels;

#ifdef CAR_PWM_MOTOR
	car_motor_direction(g_ackerman_motion.gpios,pwm_right,pwm_left);

	motor_channels[0].channel = CAR_PWM_MOTOR_A_CHANNEL;
	motor_channels[0].duty = abs(pwm_right);
	motor_channels[1].channel = CAR_PWM_MOTOR_B_CHANNEL;
	motor_channels[1].duty = abs(pwm_left);
	car_pwm_duty_config(CAR_PWM_MOTOR,&motor_channels,2);
#endif

#ifdef CAR_PWM_SERVO

	servo_channels.channel = CAR_PWM_SERVO_CHANNEL;
	servo_channels.duty = abs(servo);
	car_pwm_duty_config(CAR_PWM_SERVO,&servo_channels,1);

#endif

}

/* pid parameter init */
static void car_pid_init(void){
	struct pid_cntl_s *pid_cntl_r;
	struct pid_cntl_s *pid_cntl_l;
	uint8_t i;
	struct timespec tp;

	pid_cntl_r = &g_ackerman_motion.motor_right.pid_cntl;
	pid_cntl_l = &g_ackerman_motion.motor_left.pid_cntl;
	pid_cntl_r->KP = g_KP;
	pid_cntl_r->KI = g_KI;
	pid_cntl_r->KD = 0;
	pid_cntl_l->KP = g_KP;
	pid_cntl_l->KI = g_KI;
	pid_cntl_l->KD = 0;

	for (i = PID_SAMPLE_SIZE - 1; i > 0 ; i--) {
        pid_cntl_r->err[i] = 0;
		pid_cntl_r->err[i-1] = 0;
		pid_cntl_l->err[i] = 0;
		pid_cntl_l->err[i-1] = 0;
    }
	clock_gettime(CLOCK_MONOTONIC, &tp);
	g_lasttime = ((uint64_t)(tp.tv_sec) * 1000000) + tp.tv_nsec / 1000;   //us
}

/* servo init  */
static void car_servo_init(void){
	struct servo_control_s *servo_cntl;
	float bias;

	servo_cntl = &g_ackerman_motion.servo_cntl;
	servo_cntl->angle_zero_pwm = SERVO_PWM_ZERO;
	get_servo_zero_bias(&bias);
	servo_cntl->angle_bias_pwm = (int)(bias);
	servo_cntl->angle_goal_pwm = servo_cntl->angle_zero_pwm +
									servo_cntl->angle_bias_pwm;

}

static void car_motion_deinit(void){
	car_pwm_deinit();
	car_qe_deinit();
	gpio_motion_deinit();
}

/*********************** interface **********************************/
void get_motor_actual_speed(float *right, float *left){
	*right = g_ackerman_motion.motor_right.v_actual;
	*left = g_ackerman_motion.motor_left.v_actual;
}

void get_motor_goal_speed(float *right, float *left){
	*right = g_ackerman_motion.motor_right.v_goal;
	*left = g_ackerman_motion.motor_left.v_goal;
}

void get_servo_pwm(uint32_t *servo_pwm){
	*servo_pwm = g_ackerman_motion.servo_cntl.angle_goal_pwm;
}

/**********************************************************/

static int car_motion_init(void){
	int ret;

	/**Init **/
#ifdef CAR_PWM_MOTOR
	ret = car_pwm_init(&g_ackerman_motion.motor_pwm,CAR_PWM_MOTOR);
	if (ret != OK)
		printf("car_pwm_init: init failed %d \n",CAR_PWM_MOTOR);
#endif

#ifdef CAR_PWM_SERVO
	ret = car_pwm_init(&g_ackerman_motion.servo_pwm,CAR_PWM_SERVO);
	if (ret != OK)
		printf("car_pwm_init: init failed %d \n",CAR_PWM_SERVO);
	printf("car_pwm_init: servo init ok  \n");
#endif

#ifdef CAR_MOTOR_QE 
	ret = car_qe_init(&g_ackerman_motion.encoders,
					&g_ackerman_motion.encoder_num);
	if (ret != OK)
		printf("car_qe_init: init failed \n");

#endif
	ret = gpio_motion_init(&g_ackerman_motion.gpios);
	if (ret != OK)
		printf("gpio_motion_init: init failed \n");

	car_pid_init();
	car_servo_init();
	/* smooth init */
	g_ackerman_motion.smooth_speed = 0;
	return ret;
}

int motion_task(int argc, char *argv[]){
	int ret;
	struct motor_control_s *motor_right;
	struct motor_control_s *motor_left;
	struct servo_control_s *servo_cntl;
	struct pwm_chan_s channels;

	float vx_ros, vz_ros;
	float vx_rc, vz_rc;
	float kp_ros, ki_ros;
	float bias;
	float voltage;
	uint64_t starttime, endtime;
	struct timespec tp;
	uint64_t count = 0;

	car_motion_init();
	motor_right = &g_ackerman_motion.motor_right;
	motor_left = &g_ackerman_motion.motor_left;
	servo_cntl = &g_ackerman_motion.servo_cntl;

	printf("motion task start running \n");

	/* Self check function (need to do) */

	clock_gettime(CLOCK_MONOTONIC, &tp);
	starttime = ((uint64_t)(tp.tv_sec) * 1000000) + tp.tv_nsec / 1000;   /* (us) */

	/* ros control ackerman car */
	while(true){

		clock_gettime(CLOCK_MONOTONIC, &tp);
		endtime = ((uint64_t)(tp.tv_sec) * 1000000) + tp.tv_nsec / 1000;   //us
	    endtime = (float)(endtime - starttime) / (float)1000; /* (ms) */
		usleep(1000000/CONTROL_FREQUENCY);  /* frequency is 100HZ, actual is 33HZ */
#ifdef PID_DEBUG
		printf("%f	%f	%f	\n",endtime, motor_right->v_goal, motor_right->v_actual);
#endif
		/* STEP1: get goal (Move_x & Move_Z) */
		if (OK == get_ros_goal_speed(&vx_ros,&vz_ros)){
#ifdef AKM_DEBUG
			/* just for pid debug, ackerman car need deal with angle */
			motor_right->v_goal = vx_ros;
			motor_left->v_goal = vx_ros;
			servo_cntl->angle_goal_pwm = vz_ros* 1000;
#else
			inverse_kinematics(vx_ros, vz_ros,
								&motor_left->v_goal,
								&motor_right->v_goal,
								servo_cntl,false);					
#endif
		}
		/* get rc data */
		if (OK == get_rc_goal_speed(&vx_rc, &vz_rc)) {
			vx_rc = MAX_SPEED * vx_rc;
			vz_rc = -MAX_ANGLE * vz_rc;

			/* smoothing the input speed */
			smooth_speed_control(vx_rc, SMOOTH_STEP);
			vx_rc = g_ackerman_motion.smooth_speed;

			inverse_kinematics(vx_rc, vz_rc,
									&motor_left->v_goal,
									&motor_right->v_goal,
									servo_cntl,true);	
#ifdef ACKERMAN_DEBUG
			printf("motion_task get rc speed  =%f angle = %f\n",vx_rc, servo_cntl->angle_goal_pwm);
#endif
		}

		fflush(stdout);

		if (OK == get_ros_pid(&kp_ros, &ki_ros)){
			//just for pid debug, ackerman car need deal with angle
				motor_right->pid_cntl.KP = kp_ros;
				motor_right->pid_cntl.KI = kp_ros;
				motor_left->pid_cntl.KP = kp_ros;
				motor_left->pid_cntl.KI = kp_ros;
#ifdef ACKERMAN_DEBUG
				printf("kp  =%f \n\n\n\n",motor_right->pid_cntl.KP);
#endif
		}

		/* STEP2: get actual speed */
		calculate_car_speed(&motor_right->v_actual, &motor_left->v_actual);

		channels.channel = 2;
		/* check pwm status for debug */
		//car_get_pwm_status(g_ackerman_motion.servo_pwm,&channels, 1);
#ifdef AKM_DEBUG
		car_get_pwm_status(g_ackerman_motion.servo_pwm,&channels, 1);
		//car_get_pwm_status(g_ackerman_motion.motor_pwm,&channels, 1);	
#endif
		/* if CAR STATUS RIGHT  */
		if (true) {
			
			/* STEP3: do PID */
			motor_right->pwm_ccr += incremental_PID(&motor_right->pid_cntl,
							motor_right->v_goal -  motor_right->v_actual);
			motor_left->pwm_ccr += incremental_PID(&motor_left->pid_cntl,
							motor_left->v_goal -  motor_left->v_actual);
			motor_right->pwm_ccr = AMP_LIMIT(motor_right->pwm_ccr, -MAX_PWM, MAX_PWM);
			motor_left->pwm_ccr = AMP_LIMIT(motor_left->pwm_ccr, -MAX_PWM, MAX_PWM);

			//set servo angle 
			servo_cntl->angle_goal_pwm = AMP_LIMIT(servo_cntl->angle_goal_pwm, MIN_SERVO_PWM, MAX_SERVO_PWM);
			
			/* STEP4: really driver motor & servo */	
			set_pwm(motor_right->pwm_ccr, motor_left->pwm_ccr, servo_cntl->angle_goal_pwm);

		}
		else { /* stop car motor */
			set_pwm(0, 0, 1500);
		}
	}

	car_motion_deinit();
	printf("motion task exit\n");

	return ret;
}


