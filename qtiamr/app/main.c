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

int main(int argc, FAR char *argv[])
{

  /* main function */

  syslog(LOG_INFO, "main: qtiamr  main start\n");

  /* init qrc */
  init_qrc_management();


  /* config parameter */
  config_parameter_init();

  syslog(LOG_INFO, "main: qtiamr  main exit\n");
  return EXIT_SUCCESS;
}
