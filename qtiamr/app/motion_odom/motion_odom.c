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

#include "motion_management.h"
#include "motion_odom.h"
#include "motion_msg.h"
#include "qrc_msg_management.h"
#include "main.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void motion_public_odom(struct motion_odom_s motion_odom);

/****************************************************************************
 * Private Data
 ****************************************************************************/
struct qrc_pipe_s *g_odom_pipe = NULL;
/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* send odom to RB5 */
static void motion_public_odom(struct motion_odom_s motion_odom)
{
  enum qrc_write_status_e result;

  if (g_odom_pipe == NULL)
    {
      return;
    }

  /* write odom command with ack */
  result = qrc_write(g_odom_pipe, (uint8_t *)&motion_odom, sizeof(struct motion_odom_s), false);
  if (result != SUCCESS)
    {
      syslog(LOG_ERR, "Motion odom send failed %d\n", result);
    }

  return;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: motion_odom Thread function
 ****************************************************************************/

int motion_odom(int argc, char *argv[])
{
  char pipe_name[] = ODOM_PIPE;

  /* get qrc pipe */

  g_odom_pipe =  qrc_get_pipe(pipe_name);
  if (g_odom_pipe == NULL)
    {
      config_notify_completed(false);
      return -1;
    }

  /* register odom cb*/
  register_motion_odom_cb(motion_public_odom);

  /* notify ok */
  config_notify_completed(true);

  return 0;
}