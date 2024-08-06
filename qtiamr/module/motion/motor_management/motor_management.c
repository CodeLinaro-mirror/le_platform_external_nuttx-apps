/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
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
#include <math.h>

#include "motion_management.h"
#include "motor_management.h"

#include "motion_management.h"
#include "motion_sm.h"
#include "kinematics.h"
#include "motion_msg.h"
#include "8015d.h"

/* motor management call kinematic & motor hal APIs to realize motor control */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MOTOR_HAL             HAL_8015D
#define MOTOR_THREAD_FRQUENCY 50

#define MOTOR_DRIVER_STATUS_CHECK_FRQUENCY 10

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* motor management structure */

struct motor_management_s
{
  enum control_mode_e   mode;
  void *                motor_hal;
  struct motor_hal_ops *motor_ops;
  motion_odom_cb        motion_odom_cb;
  motor_notify_cb       pose_done_cb;
  motor_notify_cb       switch_done_cb;
  uint32_t              frequency;
} __attribute__((aligned(4)));

/* hal object */
enum motor_hal_index_e
{
  HAL_8015D,
  HAL_MAX,
};

struct motor_hal_s
{
  enum motor_hal_index_e index;
  void *                 motor;
  struct motor_hal_ops * hal_ops;
};

struct pid_control_speed_s
{
  volatile int16_t  rpm_left;
  volatile int16_t  rpm_right;
  volatile bool     update;
} __attribute__((aligned(4)));

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int motor_get_speed_odom(float *vx, float *vz);
//static bool motor_check_position_reach(void);
static int motor_speed_odom_work(union motion_control_data_u data);
static int motor_driver_status_work(union motion_control_data_u data);

/****************************************************************************
 * Private Data
 ****************************************************************************/
static const struct motor_hal_s g_hal_list[] = {
  { HAL_8015D, &g_zlac_8015d, &zlac_8015d_ops },
};

static struct motor_management_s g_motor_manager = {
  .mode           = INACTIVE,
  .motor_hal      = NULL,
  .motor_ops      = NULL,
  .motion_odom_cb = NULL,
  .pose_done_cb   = NULL,
  .switch_done_cb = NULL,
  .frequency      = MOTOR_THREAD_FRQUENCY,
};

struct pid_control_speed_s g_pid_control_speed = {0,0,false};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int motor_get_speed_odom(float *vx, float *vz)
{
  int                   result;
  void *                motor   = g_motor_manager.motor_hal;
  struct motor_hal_ops *hal_ops = g_motor_manager.motor_ops;
  float                 left_rpm, right_rpm;

  if (g_motor_manager.mode != SPEED)
    {
      syslog(LOG_ERR, "motor_get_speed failed \n");
      return ERROR;
    }

  result = hal_ops->get_rpm(motor, &left_rpm, &right_rpm);
  if (result == OK)
    {
      if (false == speed_rpm_transfer_to_odom(left_rpm, right_rpm, vx, vz))
        {
          syslog(LOG_ERR, "speed_rpm_transfer_to_odom failed\n");
          result = ERROR;
        }
    }
  return result;
}

static int motor_speed_odom_work(union motion_control_data_u data)
{
  struct motion_odom_s odom;
  int                  result;
  struct timespec      timestamp;
  /* get speed odometry */

  odom.type = ODOM_SPEED;
  clock_gettime(CLOCK_REALTIME, &timestamp);
  odom.sec = timestamp.tv_sec;
  odom.ns  = timestamp.tv_nsec;
  /* call callback */

  if (g_motor_manager.motion_odom_cb != NULL)
    {
      result = motor_get_speed_odom(&odom.x, &odom.z);
      if (OK == result)
        {
          g_motor_manager.motion_odom_cb(odom);
        }
      else
        {
          syslog(LOG_ERR, "get odom failed result=%d\n", result);
          return ERROR;
        }
    }
  return OK;
}

static int motor_driver_status_work(union motion_control_data_u data)
{
  union motion_control_data_u driver_data;

  driver_data.motor_driver_status = motor_get_status_code();
  if (driver_data.motor_driver_status != NORMAL)
    {
      /* send drv_err event to motion stat machine */
      motion_sm_event(EV_DRI_ERR, driver_data);
      syslog(LOG_ERR, "ERROR: motor driver status error \n");
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: motor_set_pid
 ****************************************************************************/

int motor_set_pid(enum control_mode_e control_mode, struct motion_pid_s pid)
{
  void *motor = g_motor_manager.motor_hal;

  return g_motor_manager.motor_ops->set_pid(motor, control_mode, pid);
}

int motor_set_speed(float vx, float vz)
{
  int16_t               left_rpm, right_rpm;
  int                   result;

  if (speed_inverse_kinematics(vx, vz, &left_rpm, &right_rpm))
    {
      //speed control with pid thread
      g_pid_control_speed.rpm_left = left_rpm;
      g_pid_control_speed.rpm_right = right_rpm;
      g_pid_control_speed.update = true;
    }
  else
    {
      syslog(LOG_ERR, "motor_set_speed speed_inverse_kinematics failed\n");
      result = ERROR;
    }

  return result;
}

int motor_quick_stop(bool enable)
{
  void *motor = g_motor_manager.motor_hal;

  if (enable)
    {
      syslog(LOG_ERR, "motor_quick_stop enable = %d \n", enable);
      /* reset mode as inactive */
      g_motor_manager.mode = INACTIVE;
      return g_motor_manager.motor_ops->quick_stop(motor);
    }

  return OK;
}

/* get motor driver status */
enum motor_err_e motor_get_status_code(void)
{
  enum motor_err_e      err_code = NORMAL;
  void *                motor    = g_motor_manager.motor_hal;
  struct motor_hal_ops *hal_ops  = g_motor_manager.motor_ops;

  hal_ops->get_motor_status_code(motor, &err_code);

  return err_code;
}

int motor_switch_mode(enum control_mode_e mode)
{
  bool result = false;

  /* present, just support speed */
  if (SPEED == mode)
    {
      if (g_motor_manager.mode == mode)
        {
          result = true;
        }
      else
        {
          result = motor_management_init();
        }
      return (result == true) ? OK : ERROR;
    }
  else
    {
      syslog(LOG_ERR, "motor switch error present_mode=%d\n,"
                      "target_mode=%d",
             g_motor_manager.mode, mode);
      return ERROR;
    }
}

int motor_set_position(bool pose_type, float pose)
{
  return ERROR;
}

void motor_register_odometry_cb(motion_odom_cb odom_cb)
{
  g_motor_manager.motion_odom_cb = odom_cb;
}

/* register action done cb */
void motor_register_position_done_cb(motor_notify_cb pose_done_cb)
{
  g_motor_manager.pose_done_cb = pose_done_cb;
}

void motor_register_switching_done_cb(motor_notify_cb switch_done_cb)
{
  g_motor_manager.switch_done_cb = switch_done_cb;
}

void motor_set_odom_frquency(uint32_t frequency)
{
  g_motor_manager.frequency = frequency;
}

bool motor_management_init(void)
{
  bool                  result = false;
  void *                motor  = g_hal_list[MOTOR_HAL].motor;
  struct motor_hal_ops *ops    = g_hal_list[MOTOR_HAL].hal_ops;
  ;

  /* get hal */
  g_motor_manager.motor_hal = motor;
  g_motor_manager.motor_ops = ops;

  /* init hal */
  result = g_motor_manager.motor_ops->motor_hal_init(motor);
  if (result != true)
    {
      syslog(LOG_ERR, "ERROR:init_motor:  motor init failed\n");
      return false;
    }
  /* default mode is speed mode */
  g_motor_manager.mode = SPEED;
  syslog(LOG_INFO, "init_motor:  motor init done\n");
  return result;
}

/****************************************************************************
 * Name: motor_management_thread
 * function: get motor odom & status data
 ****************************************************************************/

int motor_management_thread(int argc, char *argv[])
{
  uint32_t                    frequency = g_motor_manager.frequency;
  enum motion_sm_state_e      control_state;
  union motion_control_data_u motion_data;
  uint32_t                    count      = 0;
  uint32_t                    driver_num = frequency / MOTOR_DRIVER_STATUS_CHECK_FRQUENCY;
  syslog(LOG_INFO, "motor_management_thread:  starting \n");

  while (true)
    {
      /* set frequency */
      usleep(1000000 / frequency);

      control_state = get_motion_sm_state();
      if (control_state == ST_SPEED)
        {
          motion_add_work(motor_speed_odom_work, motion_data);

          /* get motor driver status */
          count++;
          if (count % driver_num == 0)
            {
              motion_add_work(motor_driver_status_work, motion_data);
            }
        }
    }

  return ERROR;
}


/****************************************************************************
 * PID for low speed control (goal =< 10 rpm )
 *
 ****************************************************************************/
#define   PID_CONTROL_WAIT_TIME_S (1)

#define   SPPED_PID_P       (0.75)
#define   SPPED_PID_I       (0.003)

#define   MAX_SPEED         (10.0)  //rpm
#define   MIN_SPEED         (2.0)  //rpm
#define   MAX_SPEED_OUT     (13.0)  //rpm
#define   ERROR_LIMIT       (0.01)   //rpm
#define   PID_MAX_LOOP      (5)
#define   PID_TIME_DELAY_MS (15)     //ms,200HZ
#define   LEFT_MOTOR_INDEX  (0)
#define   RIGHT_MOTOR_INDEX (1)

/****************************************************************************
 * PID thread for low speed control (goal =< 10 rpm )
 ****************************************************************************/
struct pid_control_thread_s
{
  pthread_mutex_t    mutex;
  pthread_cond_t     cond;
  //sensor data
  volatile float     sensor_left_rpm;
  volatile float     sensor_right_rpm;
} __attribute__((aligned(4)));

struct pid_control_thread_s g_pid_control_thread;

static void pid_work_wait_notify(void);
void pid_notify_completed(void);
static int pid_speed_control_work(union motion_control_data_u data);
static int pid_get_rpm_work(union motion_control_data_u data);

static void pid_control_init(void)
{
  int status;
  /* init cond & mutex */
  status = pthread_mutex_init(&g_pid_control_thread.mutex, NULL);
  if (status != 0)
    {
      syslog(LOG_ERR, "pid_control_init: ERROR pthread_mutex_init failed, status=%d\n",
             status);
      ASSERT(false);
    }

  status = pthread_cond_init(&g_pid_control_thread.cond, NULL);
  if (status != 0)
    {
      syslog(LOG_ERR, "pid_control_init: ERROR pthread_cond_init failed, status=%d\n",
             status);
      ASSERT(false);
    }

  g_pid_control_thread.sensor_left_rpm = 0.0;
  g_pid_control_thread.sensor_right_rpm = 0.0;
}

static void pid_work_wait_notify(void)
{
  struct timespec ts;
  int             status;

  status = pthread_mutex_lock(&g_pid_control_thread.mutex);
  if (status != 0)
    {
      syslog(LOG_ERR, "pid_work_wait_notify: ERROR pthread_mutex_lock failed, status=%d\n",
             status);
      ASSERT(false);
    }

  status = clock_gettime(CLOCK_REALTIME, &ts);
  if (status != 0)
    {
      syslog(LOG_ERR, "pid_work_wait_notify: ERROR clock_gettime failed\n");
      ASSERT(false);
    }

  ts.tv_sec += PID_CONTROL_WAIT_TIME_S;

  status = pthread_cond_timedwait(&g_pid_control_thread.cond, &g_pid_control_thread.mutex, &ts);
  if (status != 0)
    {
      if (status == ETIMEDOUT)
        {
          syslog(LOG_INFO, "pid_work_wait_notify: pthread_cond_timedwait timed out\n");
        }
      else
        {
          syslog(LOG_ERR, "pid_work_wait_notify:"
                          "ERROR pthread_cond_timedwait failed, status=%d\n",
                 status);
          ASSERT(false);
        }
    }

  /* Release the mutex */

  status = pthread_mutex_unlock(&g_pid_control_thread.mutex);
  if (status != 0)
    {
      syslog(LOG_ERR, "config_wait_notify: ERROR pthread_mutex_unlock failed, status=%d\n",
             status);
      ASSERT(false);
    }
}

void pid_notify_completed(void)
{
  int status;

  status = pthread_mutex_lock(&g_pid_control_thread.mutex);
  if (status != 0)
    {
      syslog(LOG_ERR, "pid_notify_completed:"
                      "ERROR pthread_mutex_lock failed, status=%d\n",
             status);
      ASSERT(false);
    }

  status = pthread_cond_signal(&g_pid_control_thread.cond);
  if (status != 0)
    {
      syslog(LOG_ERR, "pid_notify_completed:"
                      "ERROR pthread_cond_signal failed, status=%d\n",
             status);
      ASSERT(false);
    }

  status = pthread_mutex_unlock(&g_pid_control_thread.mutex);
  if (status != 0)
    {
      syslog(LOG_ERR, "pid_notify_completed:"
                      "ERROR pthread_mutex_unlock failed, status=%d\n",
             status);
      ASSERT(false);
    }
}

int motor_pid_control_thread(int argc, char *argv[])
{
  int16_t               speed_goal[2] = {0};
  int16_t               speed_out_rpm[2];
  float                 speed_out[2] = {0.0};
  float                 speed_sensor[2] = {0.0};
  float                 pre_1_err[2] = {0.0}; /*last error */
  float                 error_pid[2] = {0.0}; /* error between sensor & goal*/
  bool                  pid_control_tag[2] = {false};
  bool                  do_pid = false;

  union motion_control_data_u motion_data;

  enum motion_sm_state_e      control_state;

  //init
  pid_control_init();

  while(1)
  {
    control_state = get_motion_sm_state();
    if (control_state == ST_SPEED)
    {
      break;
    }
    usleep(100000);
  }

  syslog(LOG_INFO, "motor_pid_thread:  starting \n");

  while (1)
    {
      usleep(PID_TIME_DELAY_MS*1000);
      //update goal
      if (true == g_pid_control_speed.update)
        {
          speed_goal[0] = g_pid_control_speed.rpm_left;
          speed_goal[1] = g_pid_control_speed.rpm_right;
          g_pid_control_speed.update = false;
          //syslog(LOG_INFO, "motor_pid_thread: update goal %d\t %d \n",speed_goal[0],speed_goal[1]);

          if (abs(speed_goal[0]) <= MAX_SPEED && abs(speed_goal[0]) >=MIN_SPEED)
            {
              pid_control_tag[LEFT_MOTOR_INDEX] = true;
            }
          else
            {
              pid_control_tag[LEFT_MOTOR_INDEX] = false;
            }

          if (abs(speed_goal[1]) <= MAX_SPEED && abs(speed_goal[1]) >=MIN_SPEED)
            {
              pid_control_tag[RIGHT_MOTOR_INDEX] = true;
            }
          else
            pid_control_tag[RIGHT_MOTOR_INDEX] = false;

          /* set speed first */
          motion_data.speed_rpm.left_rpm = speed_goal[LEFT_MOTOR_INDEX];
          motion_data.speed_rpm.right_rpm = speed_goal[RIGHT_MOTOR_INDEX];
          motion_add_work(pid_speed_control_work, motion_data);
          pid_work_wait_notify();

          usleep(PID_TIME_DELAY_MS*1000);

          //update speed out
          speed_out[0] = (float) speed_goal[LEFT_MOTOR_INDEX];
          speed_out[1] = (float) speed_goal[RIGHT_MOTOR_INDEX];
          pre_1_err[0] = 0.0;
          pre_1_err[1] = 0.0;

          if (false == pid_control_tag[LEFT_MOTOR_INDEX] && false ==
                                                pid_control_tag[RIGHT_MOTOR_INDEX])
            {
              do_pid = false;
            }
          else
            {
              do_pid = true;
            }
        }

      //do_pid
      if (do_pid)
        {
          //get sensor data
          motion_add_work(pid_get_rpm_work, motion_data);
          pid_work_wait_notify();
          speed_sensor[LEFT_MOTOR_INDEX] = g_pid_control_thread.sensor_left_rpm;
          speed_sensor[RIGHT_MOTOR_INDEX] = g_pid_control_thread.sensor_right_rpm;

          //syslog(LOG_INFO, "get speed \t\t%f\t\t%f\n",speed_sensor[LEFT_MOTOR_INDEX],
          //                                      speed_sensor[RIGHT_MOTOR_INDEX]);

          //calculate pid
          for (int i =0; i < 2; i++)
            {
              //calculate error
              error_pid[i] = (float)speed_goal[i] - speed_sensor[i];
              //syslog(LOG_ERR, "error_pid \t%f \n",error_pid[i] );

              //if (true == pid_control_tag[i] && (fabs(error_pid[i]) > ERROR_LIMIT))
              if (true == pid_control_tag[i])
                {
                  //pid calculate
                  speed_out[i] = speed_out[i] +
                                    (error_pid[i] - pre_1_err[i])*SPPED_PID_P +
                                        error_pid[i]*PID_TIME_DELAY_MS*SPPED_PID_I;
                  pre_1_err[i] = error_pid[i];

                  //limit max & get int16 rpm
                  speed_out_rpm[i] = (int16_t)round(AMP_LIMIT(speed_out[i],
                                                -MAX_SPEED_OUT, MAX_SPEED_OUT));
                }
              else
                {
                  pre_1_err[i] = error_pid[i];
                  pid_control_tag[i] = false;
                }
            }

          //set speed
          motion_data.speed_rpm.left_rpm = speed_out_rpm[LEFT_MOTOR_INDEX];
          motion_data.speed_rpm.right_rpm = speed_out_rpm[RIGHT_MOTOR_INDEX];
          motion_add_work(pid_speed_control_work, motion_data);
          pid_work_wait_notify();

          if (pid_control_tag[0] == pid_control_tag[1] && pid_control_tag[1] == false)
            {
              do_pid = false;
            }
        }
    }

  return ERROR;
}

/* speed cmd control */
static int pid_speed_control_work(union motion_control_data_u data)
{
  int16_t left_rpm = data.speed_rpm.left_rpm;
  int16_t right_rpm = data.speed_rpm.right_rpm;

  void *                motor   = g_motor_manager.motor_hal;
  struct motor_hal_ops *hal_ops = g_motor_manager.motor_ops;

  /* speed control */
  hal_ops->set_speed(motor, left_rpm, right_rpm);
  //syslog(LOG_INFO, "set speed :\t\t\t%d\t\t\t%d\n",left_rpm,right_rpm);
  pid_notify_completed();
  return OK;
}

static int pid_get_rpm_work(union motion_control_data_u data)
{
  void *                motor   = g_motor_manager.motor_hal;
  struct motor_hal_ops *hal_ops = g_motor_manager.motor_ops;
  float left_rpm = 0;
  float right_rpm = 0;

  if(OK != hal_ops->get_rpm(motor, &left_rpm, &right_rpm))
    {
      syslog(LOG_ERR, "get_rpm failed\n");
      return ERROR;
    }

  g_pid_control_thread.sensor_left_rpm = left_rpm;
  g_pid_control_thread.sensor_right_rpm = right_rpm;

  pid_notify_completed();
  return OK;
}