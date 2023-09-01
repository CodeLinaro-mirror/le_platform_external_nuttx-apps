/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#ifndef __APP_QCOMAMR_ULTRASOUND_H
#define __APP_QCOMAMR_ULTRASOUND_H
/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <sched.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <syslog.h>

#ifdef CONFIG_APP_QTIAMR

#define RS485_MSG_LEN (16) /* 8 bytes message len & 8 for extern */
#define RS485_RECEIVE_TIME_OUT (200000)

struct rs485_data_s
{
    int rs485_fd;
    int (*send_msg)(char *buff, int len, int fd);
};

struct us_cmd_buffer_s
{
    char us_addr;
    char us_cmd_id;
    int16_t us_register;
    int16_t us_reg_len;
};

typedef struct
{
    uint8_t addr;
    float dist;
    float temp;
    float alert_limit;
    bool status;
    void (*avoidance_handle)(void* this);
} us_data_s;

#define ULRTUSOUND_NUM_MAX (7)

#define SINGLE_FRAME_LEN (6)  /* No CRC frame*/
#define RETURN_RAME_WRITE_SIZE  (8)
#define RETURN_RAME_READ_SINGLE_SIZE  (7)

#define BROADCAST_ADDR  (0xFF)
#define CONTROLLER_ADDR (0x01)
#define R_SINGLE_REG    (0x03)
#define W_SINGLE_REG    (0x06)

#define DIST_REG        (0x0100)
#define RAWDIST_REG     (0x0101)
#define TEMP_REG        (0x0102)
#define ADDR_REG        (0x0200)

int ultrasound_task(int argc, char *argv[]);
bool motor_need_stop();

#endif
#endif
