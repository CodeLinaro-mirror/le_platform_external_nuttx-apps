/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#ifndef __MAIN_H
#define __MAIN_H

#include "config_msg.h"

#define DEFAULT_PRIORITY  (110)
#define DEFAULT_STACK_SIZE  (2048)

/* configuration API */

int get_configuration_parameters(enum config_msg_type_e type, void *parameters);

void config_notify_completed(bool initialized);
int config_parameter_init(void);  /* called by main */


#endif