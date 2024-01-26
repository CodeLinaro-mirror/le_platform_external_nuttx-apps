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
#include "motion_management.h"
#include "time_sync.h"
#include "remote_controller.h"
#include "rc_management.h"

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
#define DEFAULT_IMU_ENABLE      (0)
#define DEFAULT_ULTRA_ENABLE    (0)
#define DEFAULT_ULTRA_QUANTITY  (5)

#define DEFAULT_SAFE_DISTANCE   (0.050f)

#define DEFAULT_RC_ENABLE       (1)
#define DEFAULT_RC_MAX_SPEED    (0.8f)
#define DEFAULT_RC_MAX_ANGLE_SPEED  (2.0f)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct config_parameters_s
{
  pthread_mutex_t config_mutex;
  pthread_mutex_t set_params_mutex;
  pthread_cond_t config_cond;
  struct qrc_pipe_s *pipe;
  bool notify_initialized;
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
static bool config_get_initialization_status(void);
static enum mcb_task_id_e start_mcb_task(void);
static void print_parameters(void);
/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Index match with enum mcb_task_id_e */
static struct mcb_task_s mcb_tasks[] = {
  {"MOTION_MANAGEMENT",   DEFAULT_PRIORITY, DEFAULT_STACK_SIZE, motion_management_init, NULL ,0},
  {"CHARGER_MANAGEMENT",  DEFAULT_PRIORITY, DEFAULT_STACK_SIZE, NULL,                   NULL ,0},
  {"RC_MANAGEMENT",       DEFAULT_PRIORITY, DEFAULT_STACK_SIZE, rc_management_task,     NULL ,0},
  {"AVOID_MANAGEMENT",    DEFAULT_PRIORITY, DEFAULT_STACK_SIZE, NULL,                   NULL ,0},
  {"TIME_SYNC",           DEFAULT_PRIORITY, DEFAULT_STACK_SIZE, time_sync_thread,       NULL ,0},
  {"IMU",                 DEFAULT_PRIORITY, DEFAULT_STACK_SIZE, NULL,                   NULL ,0},
  {"MISC",                DEFAULT_PRIORITY, DEFAULT_STACK_SIZE, NULL,                   NULL ,0},
  {"MOTION_ODOM",         DEFAULT_PRIORITY, DEFAULT_STACK_SIZE, motion_odom,            NULL ,0},
  {"ROBOT_CONTROLLER",    DEFAULT_PRIORITY, DEFAULT_STACK_SIZE, robot_controller,       NULL ,0},
  {"CLIENT_CONTROLLER",   DEFAULT_PRIORITY, DEFAULT_STACK_SIZE, client_controller,      NULL ,0},
  {"CHARGER_CONTROLLER",  DEFAULT_PRIORITY, DEFAULT_STACK_SIZE, NULL,                   NULL ,0},
  {"REMOTE_CONTROLLER",   DEFAULT_PRIORITY, DEFAULT_STACK_SIZE, rc_controller_task,     NULL ,0},
  {"EMERGENCY",           DEFAULT_PRIORITY, DEFAULT_STACK_SIZE, NULL,                   NULL ,0},
};

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
  .rc_enable = DEFAULT_RC_ENABLE,
  .max_speed = DEFAULT_RC_MAX_SPEED,
  .max_angle_speed = DEFAULT_RC_MAX_ANGLE_SPEED,
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

static void print_parameters(void)
{
  syslog(LOG_INFO,"check parameters: config_car:car_model=%d\n"
                                      "kinematic_model=%d\n"
                                      "wheel_space=%f\n"
                                      "wheel_perimeter=%f\n"
                                      ,config_car.car_model,
                                      config_car.kinematic_model,
                                      config_car.wheel_space,
                                      config_car.wheel_perimeter);

  syslog(LOG_INFO,"check parameters: config_motion: max_speed=%f\n"
                                      "max_angle_speed=%f\n"
                                      "max_position_dist=%f\n"
                                      "max_position_angle=%f\n"
                                      "max_position_line_speed=%f\n"
                                      "max_position_angle_speed=%f\n"
                                      "pid_speed_0=%f\n"
                                      "pid_speed_1=%f\n"
                                      "pid_speed_2=%f\n"
                                      "pid_position_0=%f\n"
                                      "pid_position_1=%f\n"
                                      "pid_position_2=%f\n"
                                      "odom_frequency=%ld\n"
                                      ,motion_parameters.max_speed,
                                      motion_parameters.max_angle_speed,
                                      motion_parameters.max_position_dist,
                                      motion_parameters.max_position_angle,
                                      motion_parameters.max_position_line_speed,
                                      motion_parameters.max_position_angle_speed,
                                      motion_parameters.pid_speed[0],
                                      motion_parameters.pid_speed[1],
                                      motion_parameters.pid_speed[2],
                                      motion_parameters.pid_position[0],
                                      motion_parameters.pid_position[1],
                                      motion_parameters.pid_position[2],
                                      motion_parameters.odom_frequency);
  
  syslog(LOG_INFO,"check parameters: config_scale_s: speed_scale_0=%f\n"
                                      "speed_scale_1=%f\n"
                                      "position_scale_0=%f\n"
                                      "position_scale_1=%f\n"
                                      "speed_odom_scale_0=%f\n"
                                      "speed_odom_scale_1=%f\n"
                                      "position_odom_scale_0=%f\n"
                                      "position_odom_scale_1=%f\n"
                                      ,config_scales.speed_scale[0],
                                      config_scales.speed_scale[1],
                                      config_scales.position_scale[0],
                                      config_scales.position_scale[1],
                                      config_scales.speed_odom_scale[0],
                                      config_scales.speed_odom_scale[1],
                                      config_scales.position_odom_scale[0],
                                      config_scales.position_odom_scale[1]);

  syslog(LOG_INFO,"check parameters: config_sensor:imu_enable=%d\n"
                                      "ultra_enable=%d\n"
                                      "ultra_quantity=%d\n"
                                      ,senor_parameters.imu_enable,
                                      senor_parameters.ultra_enable,
                                      senor_parameters.ultra_quantity);

  syslog(LOG_INFO,"check parameters: config_remote_controller:rc_enable=%d\n"
                                      "max_speed=%f\n"
                                      "max_angle_speed=%f\n"
                                      ,rc_parameters.rc_enable,
                                      rc_parameters.max_speed,
                                      rc_parameters.max_angle_speed);

  syslog(LOG_INFO,"check parameters: config_obstacle_avoidance_s:"
                                      "safe_distance=%f\n"
                                      ,config_ob.safe_distance);
}

static int config_wait_notify(void)
{
  struct timespec ts;
  int status;
  int result;

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
          syslog(LOG_INFO,"config_wait_notify: pthread_cond_timedwait timed out\n");
        }
      else
        {
          syslog(LOG_ERR,"config_wait_notify:"
                          "ERROR pthread_cond_timedwait failed, status=%d\n", status);
          ASSERT(false);
        }
    }

  result = status;
  /* Release the mutex */

  status = pthread_mutex_unlock(&g_config_parameter.config_mutex);
  if (status != 0)
    {
      syslog(LOG_ERR,"config_wait_notify: ERROR pthread_mutex_unlock failed, status=%d\n",
              status);
      ASSERT(false);
    }

  return result;
}

static void config_clear_initialization_status(void)
{
  g_config_parameter.notify_initialized = false;
}

static void config_set_initialization_status(bool status)
{
  g_config_parameter.notify_initialized = status;
}

static bool config_get_initialization_status(void)
{
  return g_config_parameter.notify_initialized;
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
      syslog(LOG_ERR,"config_parameter_qrc_msg_cb: message size mismatch len = %d, actual = %d\n",len,sizeof(struct config_msg_s));
    }
}

static void config_parameter_msg_parse(struct qrc_pipe_s *pipe, struct config_msg_s *config_msg)
{
  enum config_msg_type_e type;
  void * parameters;
  size_t length;
  struct config_msg_s config_msg_reply = {0};
  enum mcb_task_id_e task_error_id;
  int apply_status;
  enum qrc_write_status_e result;

  int status;

  if (pipe == NULL || config_msg ==NULL)
    {
      return;
    }

  status = pthread_mutex_lock(&g_config_parameter.set_params_mutex);
  if (status != 0)
    {
      syslog(LOG_ERR, "config_notify_completed:"
            "ERROR pthread_mutex_lock failed, status=%d\n", status);
      ASSERT(false);
    }

  type = config_msg->type;
  if ( type >= 0 && type < CONFIG_MSG_TYPE_MAX)
    {
      /* save parameters */
      parameters =  g_parameters_list[type].parameter;
      length =  g_parameters_list[type].length;
      memcpy(parameters, &config_msg->data, length);
      syslog(LOG_ERR, "config_parameter_msg_parse get type=%d \n",type);
      config_msg_reply.type = type;

      result = qrc_write(pipe, (uint8_t *)&config_msg_reply, sizeof(struct config_msg_s), false);
      if (result != SUCCESS)
      {
        syslog(LOG_ERR, "config_parameter_msg_parse msg send failed %d\n", result);
      }
    }
  else if (APPLY == type)
    {
      print_parameters();
      /* do boot function */
      syslog(LOG_INFO, "start  type = %d\n",type);

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
      result = qrc_write(pipe,(void *)&config_msg_reply, sizeof(struct config_msg_s), false);
      if (result != SUCCESS)
        {
          syslog(LOG_ERR, "config_parameter_msg_parse msg send failed %d\n", result);
        }
    }
  else
    {
      syslog(LOG_ERR, "config_parameter_msg_parse type is invalid %d\\n", type);
    }

  status = pthread_mutex_unlock(&g_config_parameter.set_params_mutex);
  if (status != 0)
    {
      syslog(LOG_ERR, "config_notify_completed:"
              "ERROR pthread_mutex_unlock failed, status=%d\n", status);
      ASSERT(false);
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

          /* clear initialization tatus */
          config_clear_initialization_status();

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
            syslog(LOG_INFO, "start_mcb_task: Starting the pid %d index=%d\n", ret,index);
            mcb_tasks[index].task_id = ret;

          /* wait notify */
          config_wait_notify();

          /* check status */
          if (false == config_get_initialization_status())
            {
              syslog(LOG_INFO, "start_mcb_task: task init %s failed \n", mcb_tasks[index].name);
              break;
            }

          syslog(LOG_INFO, "start_mcb_task: Started the task %s done\n", mcb_tasks[index].name);
        }
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

  status = pthread_mutex_init(&g_config_parameter.set_params_mutex, NULL);
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

  syslog(LOG_DEBUG, "configure register qrc pipe done \n");

  if (!qrc_register_message_cb(g_config_parameter.pipe, config_parameter_qrc_msg_cb))
    {
      syslog(LOG_ERR,"qrc register config parameter cb error\n");
      return -1;
    }

  syslog(LOG_INFO,"confg_parameter init done \n");

  return 0;
}
