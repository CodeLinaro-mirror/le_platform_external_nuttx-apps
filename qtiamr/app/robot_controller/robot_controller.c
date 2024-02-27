/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
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

#include "robot_controller.h"
#include "motion_sm.h"
#include "main.h"

/* switch done cb register need to do  */


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void robot_control_qrc_msg_cb(struct qrc_pipe_s *pipe,void * data, size_t len, bool response);
static void robot_control_msg_parse(struct qrc_pipe_s *pipe, struct motion_control_msg_s *control_msg);
static void motion_emergency_done_cb(void *user, int data);
static void motion_drv_err_cb(void *user, int data);
static void motion_switch_done_cb(void *user, int data);

/****************************************************************************
 * Private Data
 ****************************************************************************/
struct qrc_pipe_s *g_robot_control_pipe = NULL;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void robot_control_msg_parse(struct qrc_pipe_s *pipe, struct motion_control_msg_s *control_msg)
{
  enum control_msg_type_e msg_type;
  enum control_client_e client = ROBOT_CONTROLLER;

  if (pipe == NULL || control_msg ==NULL)
    {
      return;
    }

  msg_type = control_msg->msg_type;
  switch(msg_type)
    {
      case SET_SPEED:
        {
          motion_speed_control(client,control_msg->data.speed_cmd.vx, control_msg->data.speed_cmd.vz);
          //syslog(LOG_INFO,"Robot control msg set speed vx=%f,vz=%f\n",control_msg->data.speed_cmd.vx, control_msg->data.speed_cmd.vz);
          break;
        }
      case SWITCH_MODE:
        {
          motion_switch_mode(control_msg->data.mode);
          /* switch done callback need to be registered */
          break;
        }
      case SET_EMERGENCY:
        {
          motion_set_emergency(control_msg->data.emergency);
          break;
        }
      case SET_POSITION:
      default:
        {
          syslog(LOG_ERR,"Robot control msg type is invalid %d\n",msg_type);
        }
    }
}

static enum control_mode_e get_control_mode(enum motion_sm_state_e state)
{
  enum control_mode_e control_mode;

  switch (state)
  {
    case ST_SPEED:
      {
        control_mode = SPEED;
        break;
      }
    case ST_DRIVER_ERR:
      {
        control_mode = SET_DRV_ERR;
        break;
      }
    default:/* control mode is inactive */
      {
        control_mode = INACTIVE;
      }
  }

  return control_mode;
}


/* robot controller motion work done callback */

static void motion_switch_done_cb(void *user, int data)
{
  struct qrc_pipe_s *pipe = (struct qrc_pipe_s *)user;
  struct motion_control_msg_s control_msg;
  int result;

  control_msg.msg_type = SWITCH_MODE;
  control_msg.data.mode = get_control_mode((enum motion_sm_state_e) data);

  /* write status to rb5 */
  result = qrc_write(pipe, (uint8_t *)&control_msg, sizeof(struct motion_control_msg_s), false);
  if (result != SUCCESS)
    {
      syslog(LOG_ERR, "Motion cb qrc send failed %d\n", result);
    }
}

static void motion_emergency_done_cb(void *user, int data)
{
  struct qrc_pipe_s *pipe = (struct qrc_pipe_s *)user;
  struct motion_control_msg_s control_msg;
  int result;

  control_msg.msg_type = SET_EMERGENCY;
  control_msg.data.emergency = data;

  /* write status to rb5 */
  result = qrc_write(pipe, (uint8_t *)&control_msg, sizeof(struct motion_control_msg_s), false);
  if (result != SUCCESS)
    {
      syslog(LOG_ERR, "Motion cb qrc send failed %d\n", result);
    }
}

static void motion_drv_err_cb(void *user, int data)
{
  struct qrc_pipe_s *pipe = (struct qrc_pipe_s *)user;
  struct motion_control_msg_s control_msg;
  int result;

  control_msg.msg_type = SWITCH_MODE;
  control_msg.data.mode = get_control_mode((enum motion_sm_state_e) data);

  /* write status to rb5 */
  result = qrc_write(pipe, (uint8_t *)&control_msg, sizeof(struct motion_control_msg_s), false);
  if (result != SUCCESS)
    {
      syslog(LOG_ERR, "Motion drv status send failed %d\n", result);
    }
}


/* robot controller qrc msg callback */

static void robot_control_qrc_msg_cb(struct qrc_pipe_s *pipe,void * data, size_t len, bool response)
{

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

/****************************************************************************
 * Name: robot controller Thread function
 ****************************************************************************/

int robot_controller(int argc, char *argv[])
{
  char pipe_name[] = MOTION_PIPE;
  struct qrc_pipe_s *pipe;

  pipe =  qrc_get_pipe(pipe_name);
  if (pipe == NULL)
    {
      config_notify_completed(false);
      return -1;
    }

  if (!qrc_register_message_cb(pipe, robot_control_qrc_msg_cb))
    {
      syslog(LOG_ERR,"qrc register robot control cb error\n");
      config_notify_completed(false);
      return -1;
    }

  /* register motion cb */
  register_motion_switch_done_cb(motion_switch_done_cb, (void *)pipe);
  register_motion_emergency_done_cb(motion_emergency_done_cb, (void *)pipe);
  register_motion_drv_err_cb(motion_drv_err_cb, (void *)pipe);

  /* notify ok */
  config_notify_completed(true);

  /* set motion as speed mode */
  /*
  sleep(18);
  enum motion_result_e motion_res;
  motion_res = motion_switch_mode(SPEED);
  if (M_OK != motion_res)
    {
      syslog(LOG_ERR,"robot_controller switch motion failed res=%d\n",
                                                          motion_res);
    }
  syslog(LOG_INFO,"robot_controller switch motion as speed mode\n");
  */
  return 0;
}