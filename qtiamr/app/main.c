/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>
#include <stdio.h>
#include <errno.h>
#include <sched.h>
#include <syslog.h>

#include "main.h"

#include "qrc_msg_management.h"
#include "qrc.h"

#define VERSION (2.000)

int main(int argc, FAR char *argv[])
{

  /* main function */
  printf("\n\n\n#############################\n");
  printf("#MCB firmware version:%.3f #\n", VERSION);
  printf("#MCB firmware tag AMR_release.1.0.0 #\n");
  printf("#############################\n\n");

  syslog(LOG_INFO, "main: qtiamr  main start\n");

  /* init qrc */
  if (false == init_qrc_management())
    {
      syslog(LOG_ERR, "main: qrc init failed\n");
      return -1;
    }

  /* config parameter */
  config_parameter_init();

  syslog(LOG_INFO, "main: qtiamr main startup completed\n");
  qrc_pipe_threads_join();
  return EXIT_SUCCESS;
}
