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
#include "qrc_msg_management.h"

#include "motion_odom.h"
#include "robot_controller.h"
#include "motion_odom.h"
#include "motion_management.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/


#define CONFIG_TIMEOUT  (2) /* second */

#define PI (3.14159f)

#define DEFAULT_CAR_MODEL       CYCLE_CAR
#define DEFAULT_KINEMATIC_MODEL DIFF_CAR
#define DEFAULT_WHEEL_SPACE     (0.250f)
#define DEFAULT_WHEEL_PERIMETER    (0.334)
#define DEFAULT_MAX_SPEED       (1.0f)
#define DEFAULT_MAX_ANGLE_SPEED (1.5f)
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

struct config_parameters_s
{
  pthread_mutex_t config_mutex;
  pthread_cond_t config_cond;
  struct qrc_pipe_s *pipe;
  bool initialized;
} __attribute__((aligned(4)));

struct mcb_task_s
{
  const char  *name;
  uint32_t	priority;
	uint32_t	stack_size;
	int	(*task_func)(int argc, char *argv[]);
  char * const *argv;
  int task_id;
};

struct mcb_config_parameters_s
{
	enum config_msg_type_e type;
  void * parameter;
  size_t length;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void config_parameter_qrc_msg_cb(struct qrc_pipe_s *pipe, void *data, size_t len, bool response);
static void config_parameter_msg_parse(struct qrc_pipe_s *pipe, struct config_msg_s *config_msg);

static int config_wait_notify(void);
static void config_clear_initialization_status(void);
static void config_set_initialization_status(bool status);
static enum mcb_task_id_e start_mcb_task(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* Index match with enum mcb_task_id_e */
static struct mcb_task_s mcb_tasks[] =
{
  {"MOTION_MANAG",        DEFAULT_PRIORITY, DEFAULT_STACK_SIZE, motion_management_init, NULL ,0},
  {"CHARGER_MANAG",       DEFAULT_PRIORITY, DEFAULT_STACK_SIZE, NULL,                   NULL ,0},
  {"RC_MANAG",            DEFAULT_PRIORITY, DEFAULT_STACK_SIZE, NULL,                   NULL ,0},
  {"AVOID_MANAGE",        DEFAULT_PRIORITY, DEFAULT_STACK_SIZE, NULL,                   NULL ,0},
  {"TIME_SYNC",           DEFAULT_PRIORITY, DEFAULT_STACK_SIZE, NULL,                   NULL ,0},
  {"IMU",                 DEFAULT_PRIORITY, DEFAULT_STACK_SIZE, NULL,                   NULL ,0},
  {"MISC",                DEFAULT_PRIORITY, DEFAULT_STACK_SIZE, NULL,                   NULL ,0},
  {"MOTION_ODOM",         DEFAULT_PRIORITY, DEFAULT_STACK_SIZE, motion_odom,            NULL ,0},
  {"ROBOT_CONTROLLER",    DEFAULT_PRIORITY, DEFAULT_STACK_SIZE, robot_controller,       NULL ,0},
  {"CLIENT_CONTROLLER",   DEFAULT_PRIORITY, DEFAULT_STACK_SIZE, client_controller,      NULL ,0},
  {"CHARGER_CONTROLLER",  DEFAULT_PRIORITY, DEFAULT_STACK_SIZE, NULL,                   NULL ,0},
  {"REMOTE_CONTROLLER",   DEFAULT_PRIORITY, DEFAULT_STACK_SIZE, NULL,                   NULL ,0},
  {"EMERGENCY",           DEFAULT_PRIORITY, DEFAULT_STACK_SIZE, NULL,                   NULL ,0},
};

static bool g_initialized;
static struct config_parameters_s g_config_parameter;

struct config_car_s config_car =
{
  .car_model = DEFAULT_CAR_MODEL,
  .kinematic_model = DEFAULT_KINEMATIC_MODEL,
  .wheel_space = DEFAULT_WHEEL_SPACE,
  .wheel_perimeter = DEFAULT_WHEEL_PERIMETER,
};

struct config_motion_s motion_parameters =
{
  .max_speed = DEFAULT_MAX_SPEED,
  .max_angle_speed = DEFAULT_MAX_ANGLE_SPEED,
  .max_position_dist = DEFAULT_MAX_POSITION,
  .max_position_angle = DEFAULT_MAX_POSITION,
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
  .max_speed = DEFAULT_MAX_SPEED,
};

struct config_obstacle_avoidance_s config_ob =
{
  .safe_distance = DEFAULT_SAFE_DISTANCE,
};

static struct mcb_config_parameters_s  g_parameters_list[] =
{
  {CAR,                 &config_car,        sizeof(struct config_car_s)},
  {MOTION,              &motion_parameters, sizeof(struct config_motion_s)},
  {SCALE,               &config_scales,     sizeof(struct config_scale_s)},
  {SENSOR,              &senor_parameters,  sizeof(struct config_sensor_s)},
  {RC,                  &rc_parameters,     sizeof(struct config_remote_controller_s)},
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
          syslog(LOG_ERR,"config_wait_notify:"
                          "ERROR pthread_cond_timedwait failed, status=%d\n", status);
          ASSERT(false);
        }
    }
  else
    {
      syslog(LOG_ERR,"config_wait_notify: ERROR"
              "pthread_cond_timedwait returned without timeout, status=%d\n",
              status);
      ASSERT(false);
    }

  /* Release the mutex */

  status = pthread_mutex_unlock(&g_config_parameter.config_mutex);
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

static void config_parameter_qrc_msg_cb(struct qrc_pipe_s *pipe, void *data, size_t len, bool response)
{
  struct config_msg_s *config_msg;

  if (pipe == NULL || data == NULL)
    {
      return;
    }
  if (len == sizeof(struct config_msg_s))
    {
      config_msg = (struct config_msg_s *)data;
      config_parameter_msg_parse(pipe, config_msg);
    }
  else
    {
      syslog(LOG_ERR,"config_parameter_qrc_msg_cb: message size mismatch\n");
    }
}

static void config_parameter_msg_parse(struct qrc_pipe_s *pipe, struct config_msg_s *config_msg)
{
  enum config_msg_type_e type;
  void * parameters;
  size_t length;
  struct config_msg_s config_msg_reply;
  struct config_apply_s *apply;
  enum mcb_task_id_e task_error_id;
  uint8_t apply_status;
  enum qrc_write_status_e result;

  if (pipe == NULL || config_msg ==NULL)
    {
      return;
    }

  type = config_msg->type;
  if ( type >= 0 && type <= CONFIG_MSG_TYPE_MAX)
    {
      /* save parameters */
      parameters =  g_parameters_list[type].parameter;
      length =  g_parameters_list[type].length;
      memcpy(parameters, &config_msg->data, length);
    }
  else if (APPLY == type)
    {
      apply = (struct config_apply_s *)&config_msg->data;
      if (apply ->status == 0)  /* 0 is success, not 0 is failed */
        {
          /* do boot function */
          task_error_id = start_mcb_task();
          /* check, if completed and reply it */
          if (task_error_id == ID_MAX_TASK)
            {
              /* all tasks startup completed */
              apply_status = 0;
            }
          else
            {
              apply_status = -1;
            }
          /* send status msg to RB5 */
          config_msg_reply.type = APPLY;
          config_msg_reply.data.apply.status = apply_status;
          config_msg_reply.data.apply.error_type = task_error_id;
          result = qrc_write(pipe, (void *)&config_msg_reply, sizeof(struct config_msg_s), false);
          if (result != SUCCESS)
            {
              syslog(LOG_ERR, "config_parameter_msg_parse msg send failed %d\n", result);
            }
        }
    }
  else
    {
      syslog(LOG_ERR, "config_parameter_msg_parse type is invalid %d\\n", type);
    }
}

static enum mcb_task_id_e start_mcb_task(void)
{
  int index;
  int ret;
  int errcode;

  for (index = 0; index < ID_MAX_TASK; index ++)
    {
      /* start tasks */

      if (NULL != mcb_tasks[index].task_func)
        {
          ret = task_create(mcb_tasks[index].name,
                        mcb_tasks[index].priority,
                        mcb_tasks[index].stack_size,
                        mcb_tasks[index].task_func,
                        mcb_tasks[index].argv);
          if (ret < 0)
            {
              errcode = errno;
              syslog(LOG_INFO,"car_main: ERROR: Failed to start %s: %d\n",
                            mcb_tasks[index].name,errcode);
              return index;
            }
            syslog(LOG_INFO, "start_mcb_task: Starting the pid %d\n", ret);
            mcb_tasks[index].task_id = ret;
        }

      /* clear initialization tatus */
      config_clear_initialization_status();

      /* wait notify */
      config_wait_notify();

      /* check status */
      if (false == g_initialized)
        {
        break;
        }
      syslog(LOG_INFO, "start_mcb_task: Started the task %s\n", mcb_tasks[index].name);
    }

  return index;
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
  if ( type >= 0 && type <= CONFIG_MSG_TYPE_MAX)
    {
      if ( g_parameters_list[type].type == type)
        {
          memcpy(parameters,  g_parameters_list[type].parameter,  g_parameters_list[type].length);
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
      syslog(LOG_ERR, "config_notify_completed:"
            "ERROR pthread_mutex_lock failed, status=%d\n", status);
      ASSERT(false);
    }

  config_set_initialization_status(initialized);

  status = pthread_cond_signal(&g_config_parameter.config_cond);
  if (status != 0)
    {
      syslog(LOG_ERR, "config_notify_completed:"
              "ERROR pthread_cond_signal failed, status=%d\n", status);
      ASSERT(false);
    }

  status = pthread_mutex_unlock(&g_config_parameter.config_mutex);
  if (status != 0)
    {
      syslog(LOG_ERR, "config_notify_completed:"
              "ERROR pthread_mutex_unlock failed, status=%d\n", status);
      ASSERT(false);
    }
}

int config_parameter_init(void)
{
  char pipe_name[] = CONFIG_PIPE;
  int status;

  /* init cond & mutex */

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
  g_config_parameter.pipe =  qrc_get_pipe(pipe_name);
  if (g_config_parameter.pipe == NULL)
    {
      syslog(LOG_ERR,"config_parameter: get qrc pipe error\n");
      return -1;
    }

  if (!qrc_register_message_cb(g_config_parameter.pipe, config_parameter_qrc_msg_cb))
    {
      syslog(LOG_ERR,"qrc register config parameter cb error\n");
      return -1;
    }

  g_config_parameter.initialized = false;

  return 0;
}
