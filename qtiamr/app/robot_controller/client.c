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

#include "robot_controller.h"
#include "main.h"
#include "motion_management.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void client_qrc_msg_cb(struct qrc_pipe_s *pipe,void * data, size_t len, bool response);
static void client_msg_parse(struct qrc_pipe_s *pipe, struct client_msg_s *client_msg);

/****************************************************************************
 * Private Data
 ****************************************************************************/
struct qrc_pipe_s *g_client_pipe = NULL;
/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void client_msg_parse(struct qrc_pipe_s *pipe, struct client_msg_s *client_msg)
{
  enum client_msg_type_e msg_type;
  struct client_msg_s  msg;
  int result;

  if (pipe == NULL || client_msg ==NULL)
    {
      return;
    }

  msg_type = client_msg->msg_type;
  switch(msg_type)
    {
      case SET_CLIENT:
        {
          syslog(LOG_INFO,"client msg SET_CLIENT =  %d\n",client_msg->client);
          set_control_client(client_msg->client);
          break;
        }
      case GET_CLIENT:
        {
          msg.client = get_current_client();
          msg.msg_type = GET_CLIENT;
          result = qrc_write(pipe, (void*)&msg, sizeof(struct client_msg_s), false);
          if (result != SUCCESS)
            {
              syslog(LOG_ERR,"client msg send error  %d\n",result);
            }
          break;
        }
      default :
        {
          syslog(LOG_ERR,"client msg type is invalid %d\n",msg_type);
        }
    }
}

/* client qrc msg callback */

static void client_qrc_msg_cb(struct qrc_pipe_s *pipe,void * data, size_t len, bool response)
{
  struct client_msg_s *client_msg;

  if (pipe == NULL || data ==NULL)
    {
      return;
    }
  if (len == sizeof(struct client_msg_s))
    {
      client_msg = (struct client_msg_s *)data;
      client_msg_parse(pipe, client_msg);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: client_controller Thread function
 ****************************************************************************/

int client_controller(int argc, char *argv[])
{
  char pipe_name[] = CLIENT_PIPE;

  struct qrc_pipe_s *pipe;

  /* get pipe */

  pipe =  qrc_get_pipe(pipe_name);
  if (pipe == NULL)
    {
      config_notify_completed(false);
      return -1;
    }

  if (!qrc_register_message_cb(pipe, client_qrc_msg_cb))
    {
      syslog(LOG_ERR,"qrc register client cb error\n");
      config_notify_completed(false);
      return -1;
    }

  /* notify ok */
  config_notify_completed(true);
  syslog(LOG_INFO,"client_controller client exit \n");
  return 0;
}

