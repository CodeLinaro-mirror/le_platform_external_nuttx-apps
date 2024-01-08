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
        }
      case SET_POSITION:
      default:
        {
          syslog(LOG_ERR,"Robot control msg type is invalid %d\n",msg_type);
        }
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

  /* get pipe */
  pipe =  qrc_get_pipe(pipe_name);
  if (pipe == NULL)
    {
      /* notify error */
      return -1;
    }

  if (!qrc_register_message_cb(pipe, robot_control_qrc_msg_cb))
    {
      syslog(LOG_ERR,"qrc register robot control cb error\n");
      /* notify error */
      return -1;
    }

  /* notify ok */

  return 0;
}