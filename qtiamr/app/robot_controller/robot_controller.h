/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#ifndef __ROBOT_CONTROLLER_H
#define __ROBOT_CONTROLLER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "client_control_msg.h"
#include "motion_msg.h"
#include "motion_management.h"
#include "qrc_msg_management.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/


/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
/* robot controller */
int robot_controller(int argc, char *argv[]);

/* client controller */
int client_controller(int argc, char *argv[]);

#endif /* __ROBOT_CONTROLLER_H */