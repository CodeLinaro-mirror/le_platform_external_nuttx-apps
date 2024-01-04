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

#include "main.h"
#include "config_msg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DEFAULT_PRIORITY  (110)
#define DEFAULT_STACK_SIZE  (2048)


#define CONFIG_TIMEOUT  (2) /* second */

#define PI (3.14159f)

#define DEFAULT_CAR_MODEL       CYCLE_CAR
#define DEFAULT_KINEMATIC_MODEL DIFF_CAR
#define DEFAULT_WHEEL_SPACE     (0.250f)
#define DEFAULT_WHEEL_RADIUS    (0.05316f)
#define DEFAULT_MAX_SPEED       (1.0f)
#define DEFAULT_MAX_POSITION    (0.5)

#define DEFAULT_MAX_POS_ANGLE   (11)    /* RPM */
#define DEFAULT_MAX_POS_DIST    (60)    /* RPM */

#define DEFAULT_ODOM_FREQUENCY  (50)    /* HZ */

#define DEFAULT_SPEED_KP        (400)
#define DEFAULT_SPEED_KI        (200)

#define DEFAULT_POSITION_KP     (400)
#define DEFAULT_POSITION_KI     (200)

#define DEFAULT_SCALE           (1.0f)
#define DEFAULT_IMU_ENABLE      (false)
#define DEFAULT_ULTRA_ENABLE    (false)
#define DEFAULT_ULTRA_QUANTITY  (5)

#define DEFAULT_SAFE_DISTANCE   (0.050f)


/****************************************************************************
 * Private Types
 ****************************************************************************/

enum task_id_e
{
  IMU = 0,
  TIME_SYNC,
  MISC,
  MOTION_ODOM,
  ROBOT_CONTROLLER,
  CHARGER_CONTROLLER,
  REMOTE_CONTROLLER,
  EMERGENCY,
  QRC_MSG_MANAGEMENT,
  MOTION_MANAGEMENT,
  CHARGER_MANAGEMENT,
  RC_MANAGEMENT,
  AVOID_MANAGEMENT,
  MAX_START_ID,
};

struct config_parameters_s
{
  pthread_mutex_t config_mutex;
  pthread_cond_t config_cond;
  bool initialized;

} __attribute__((aligned(4)));

struct task_s {
  enum task_id_e id;
	uint32_t	priority;
	uint32_t	stack_size;
	int	(*task_func)(int argc, char *argv[]);
	char	*argv;
};

struct mcb_config_parameters_s {
	enum config_msg_type_e type;
  void * parameter;
  size_t length;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void config_parameter_qrc_msg_cb(struct qrc_pipe_s *pipe,void * data, size_t len, bool response);
static void config_parameter_msg_parse(struct qrc_pipe_s *pipe, struct motion_control_msg_s *control_msg);

static void config_wait_notify(void);
static void config_clear_initialization_status(void);
static void config_set_initialization_status(bool status);

/****************************************************************************
 * Private Data
 ****************************************************************************/
static struct mcb_task_s mcb_tasks[] = {
  {IMU,                 DEFAULT_PRIORITY, DEFAULT_STACK_SIZE, NULL, NULL},
  {TIME_SYNC,           DEFAULT_PRIORITY, DEFAULT_STACK_SIZE, NULL, NULL},
  {MISC,                DEFAULT_PRIORITY, DEFAULT_STACK_SIZE, NULL, NULL},
  {MOTION_ODOM,         DEFAULT_PRIORITY, DEFAULT_STACK_SIZE, NULL, NULL},
  {ROBOT_CONTROLLER,    DEFAULT_PRIORITY, DEFAULT_STACK_SIZE, NULL, NULL},
  {CHARGER_CONTROLLER,  DEFAULT_PRIORITY, DEFAULT_STACK_SIZE, NULL, NULL},
  {REMOTE_CONTROLLER,   DEFAULT_PRIORITY, DEFAULT_STACK_SIZE, NULL, NULL},
  {EMERGENCY,           DEFAULT_PRIORITY, DEFAULT_STACK_SIZE, NULL, NULL},
  {QRC_MSG_MANAGEMENT,  DEFAULT_PRIORITY, DEFAULT_STACK_SIZE, NULL, NULL},
  {MOTION_MANAGEMENT,   DEFAULT_PRIORITY, DEFAULT_STACK_SIZE, NULL, NULL},
  {CHARGER_MANAGEMENT,  DEFAULT_PRIORITY, DEFAULT_STACK_SIZE, NULL, NULL},
  {RC_MANAGEMENT,       DEFAULT_PRIORITY, DEFAULT_STACK_SIZE, NULL, NULL},
  {AVOID_MANAGEMENT,    DEFAULT_PRIORITY, DEFAULT_STACK_SIZE, NULL, NULL},
};


static bool g_initialized;
static struct config_parameters_s g_config_parameter;

struct config_car_s config_car =
{
  .car_model = DEFAULT_CAR_MODEL,
  .kinematic_model = DEFAULT_KINEMATIC_MODEL,
  .wheel_space = DEFAULT_WHEEL_SPACE;
  .wheel_radius = DEFAULT_WHEEL_RADIUS;
};

struct config_motion_s motion_parameters =
{
  .max_speed = DEFAULT_MAX_SPEED,
  .max_position = DEFAULT_MAX_POSITION,
  .max_position_line_speed = DEFAULT_MAX_POS_DIST,
  .max_position_angle_speed = DEFAULT_MAX_POS_ANGLE,
  .pid_speed = {DEFAULT_SPEED_KP,DEFAULT_SPEED_KI,0},
  .pid_position = {DEFAULT_POSITION_KP,DEFAULT_POSITION_KI,0},
  .odom_frequency = DEFAULT_ODOM_FREQUENCY,
};

struct config_scale_s config_scales =
{
  {DEFAULT_SCALE, DEFAULT_SCALE},
  {DEFAULT_SCALE, DEFAULT_SCALE},
  {DEFAULT_SCALE, DEFAULT_SCALE},
  {DEFAULT_SCALE, DEFAULT_SCALE},
};

struct config_sensor_s senor_parameters =
{
  .imu_enable = DEFAULT_IMU_ENABLE,
  .ultra_enable = DEFAULT_ULTRA_ENABLE,
  .ultra_quantity = DEFAULT_ULTRA_QUANTITY,
};

struct config_remote_controller_s rc_parameters =
{
  .max_speed = DEFAULT_MAX_SPEED;
};

struct config_obstacle_avoidance_s config_ob =
{
  .safe_distance = DEFAULT_SAFE_DISTANCE;
};

static struct mcb_config_parameters_s parameters_list[] = {
  {CAR,                 &config_car,        sizeof(struct config_car_s)},
  {MOTION,              &motion_parameters, sizeof(struct config_motion_s)},
  {SCALE,               &config_scales,     sizeof(struct config_scale_s)},
  {SENSOR,              &senor_parameters,  sizeof(struct config_sensor_s)},
  {REMOTE_CONTROLLER,   &rc_parameters,     sizeof(struct config_remote_controller_s)},
  {OBSTACLE_AVOIDANCE,  &config_ob,         sizeof(struct config_obstacle_avoidance_s)},
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int config_wait_notify(void)
{
  struct timespec ts;
  int status;

  status = pthread_mutex_lock(&g_config_parameter.config_mutex);
  if (status != 0)
    {
      syslog(LOG_ERR,"config_wait_notify: ERROR pthread_mutex_lock failed, status=%d\n",
              status);
      ASSERT(false);
    }

  status = clock_gettime(CLOCK_REALTIME, &ts);
  if (status != 0)
    {
      syslog(LOG_ERR,"config_wait_notify: ERROR clock_gettime failed\n");
      ASSERT(false);
    }

  ts.tv_sec += CONFIG_TIMEOUT;

  status = pthread_cond_timedwait(&g_config_parameter.config_cond, &g_config_parameter.config_mutex, &ts);
  if (status != 0)
    {
      if (status == ETIMEDOUT)
        {
          syslog(LOG_ERR,"config_wait_notify: pthread_cond_timedwait timed out\n");
        }
      else
        {
          syslog(LOG_ERR,"config_wait_notify: 
                  ERROR pthread_cond_timedwait failed, status=%d\n", status);
          ASSERT(false);
        }
    }
  else
    {
      syslog(LOG_ERR,"config_wait_notify: ERROR
             pthread_cond_timedwait returned without timeout, status=%d\n",
              status);
      ASSERT(false);
    }

  /* Release the mutex */

  status = pthread_mutex_unlock(&mutex);
  if (status != 0)
    {
      syslog(LOG_ERR,"config_wait_notify: ERROR pthread_mutex_unlock failed, status=%d\n",
              status);
      ASSERT(false);
    }
  return status;
}

static void config_clear_initialization_status(void)
{
  g_initialized = false;
}

static void config_set_initialization_status(bool status)
{
  g_initialized = status;
}

static void config_parameter_qrc_msg_cb(struct qrc_pipe_s *pipe,void * data, size_t len, bool response)
{


}

static void config_parameter_msg_parse(struct qrc_pipe_s *pipe, struct config_msg_s *config_msg)
{
  enum config_msg_type_e type;
  void * parameters;


  struct motion_control_msg_s *control_msg;

  if (pipe == NULL || data ==NULL)
    {
      return;
    }
  if (len == sizeof(struct motion_control_msg_s))
    {
      control_msg = (struct motion_control_msg_s *)data;
      robot_control_msg_parse(pipe, control_msg);
    }

}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int get_configuration_parameters(enum config_msg_type_e type, void *parameters)
{
  if (parameters == NULL)
    {
      return ERROR;
    }
  if ( type >= 0 && type >= CONFIG_MSG_TYPE_MAX)
    {
      if (parameters_list[type].type == type)
        {
          memcpy(parameters, parameters_list[type].parameter, parameters_list[type].length);
          return OK;
        }
    }
  return ERROR;
}

void config_notify_completed(bool initialized)
{
  int status;

  status = pthread_mutex_lock(&g_config_parameter.config_mutex);
  if (status != 0)
    {
      syslog(LOG_ERR,"config_notify_completed:
            ERROR pthread_mutex_lock failed, status=%d\n", status);
      ASSERT(false);
    }

  config_set_initialization_status(initialized);

  status = pthread_cond_signal(&g_config_parameter.config_cond);
  if (status != 0)
    {
      syslog(LOG_ERR,"config_notify_completed:
              ERROR pthread_cond_signal failed, status=%d\n", status);
      ASSERT(false);
    }

  status = pthread_mutex_unlock(&mutex);
  if (status != 0)
    {
      syslog(LOG_ERR,"config_notify_completed:
                ERROR pthread_mutex_unlock failed, status=%d\n", status);
      ASSERT(false);
    }
}

int config_parameter_init(int argc, char *argv[]){
{
  char pipe_name[] = CONFIG_PIPE;

  struct qrc_pipe_s *pipe;

  /* init cond & mutex */

  pthread_mutex_t config_mutex;
  pthread_cond_t config_cond;

  printf("thread_waiter: Initializing mutex\n");
  status = pthread_mutex_init(&g_config_parameter.config_mutex, NULL);
  if (status != 0)
    {
      syslog(LOG_ERR,"confg_parameter: ERROR pthread_mutex_init failed, status=%d\n",
              status);
      ASSERT(false);
    }

  status = pthread_cond_init(&g_config_parameter.config_cond, NULL);
  if (status != 0)
    {
      syslog(LOG_ERR,"confg_parameter: ERROR pthread_cond_init failed, status=%d\n",
              status);
      ASSERT(false);
    }

  /* get pipe */
  pipe =  qrc_get_pipe(pipe_name);
  if (pipe == NULL)
    {
      syslog(LOG_ERR,"config_parameter: get qrc pipe error\n");
      return -1;
    }

  if (!qrc_register_message_cb(pipe, config_parameter_qrc_msg_cb))
    {
      syslog(LOG_ERR,"qrc register config parameter cb error\n");
      return -1;
    }

  g_config_parameter.initialized = false;

  return 0;
}