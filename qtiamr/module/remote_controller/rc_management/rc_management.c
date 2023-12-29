/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <sched.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <syslog.h>


int rc_management_task(int argc, char *argv[])
{
  /*chose hal dirver*/
  /*wait-condition*/
  while(1)
  {
    wait-condition()
	if (callback_list != null)
	{
	  get_speed_from_hal()
	  callback()
	}
	sleep_ms(30);
  }
}
