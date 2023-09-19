/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#ifndef __APPS_EXAMPLES_ULTRASOUND_H
#define __APPS_EXAMPLES_ULTRASOUND_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/
#define RS485_MSG_LEN (16) /* 8 bytes message len & 8 for extern */
#define SINGLE_FRAME_LEN (6)
#define RS485_RECEIVE_TIME_OUT (2000000)

#define BROADCAST_ADDR  (0xFF)
#define CONTROLLER_ADDR (0x01)
#define R_SINGLE_REG    (0x03)
#define W_SINGLE_REG    (0x06)

#define ADDR_REG        (0x0200)
#define DIST_REG        (0x0100)
#define RAWDIST_REG     (0x0101)
#define TEMP_REG        (0x0102)

struct ulteasound_example_s
{
    FAR char *devpath;  /* Path to the capture device */
    uint8_t addr;
    uint8_t dir;        /*Dir of r/w*/
    uint16_t reg;
    union
    {
        uint16_t cmd_data;
        uint16_t reg_len;
    };
    float dist;        /* Collect this number of samples */
    float tmp;         /* Delay this number of seconds between samples */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern struct ulteasound_example_s g_ulteasound;

#endif /* __APPS_EXAMPLES_ULTRASOUND_H */
