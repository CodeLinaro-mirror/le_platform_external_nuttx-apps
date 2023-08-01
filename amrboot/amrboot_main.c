/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <sched.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdint.h>

#include "flash.h"
#include "xmodem.h"

#define VERSION (1.001)

int main(int argc, FAR char *argv[])
{
    printf("\nbootloader version:%.3f\n\n\n",VERSION);

    if (OK != xmodem_init())
    {
        return -1;
    }

    bootloader_workflow();

    xmodem_deinit();
    return OK;
}

