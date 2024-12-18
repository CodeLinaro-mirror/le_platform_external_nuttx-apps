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
#include <errno.h>
#include <debug.h>

#include "misc.h"
#include "misc_msg.h"
#include "qrc_msg_management.h"
#include "main.h"
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define WATCHDOG_TIME  (5) /* watchdog timer (second)*/
/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: misc Thread function
 ****************************************************************************/

int misc_task(int argc, char *argv[])
{
  char pipe_name[] = MISC_PIPE;
  struct qrc_pipe_s *misc_pipe = NULL;
  uint8_t count = 0;
  struct watchdog_msg watchdog_message;
  enum qrc_write_status_e result;

  /* get qrc pipe */

  misc_pipe = qrc_get_pipe(pipe_name);
  if (misc_pipe == NULL)
    {
      config_notify_completed(false);
      return -1;
    }

  /* notify ok */
  config_notify_completed(true);

  /* send watchdog msg */
  while(true)
    {
      /*sleep*/
      sleep(WATCHDOG_TIME);
      watchdog_message.count = count;

      /* write watchdog message with nack */
      result = qrc_write(misc_pipe, (uint8_t *)&watchdog_message, sizeof(struct watchdog_msg), false);
      if (result != SUCCESS)
        {
          syslog(LOG_ERR, "watchdog message send failed %d\n", result);
        }

      count ++;
    }

  return 0;
}