/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#ifndef XMODEM_H_
#define XMODEM_H_

#include <nuttx/config.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <sched.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <stdbool.h>

#include "flash.h"

/* Maximum allowed errors (user defined). */
#define X_MAX_ERRORS ((uint8_t)128u)

/* Sizes of the packets. */
#define X_PACKET_NUMBER_SIZE  ((uint16_t)2u)
#define X_PACKET_128_SIZE     ((uint16_t)128u)
#define X_PACKET_1024_SIZE    ((uint16_t)1024u)
#define X_PACKET_CRC_SIZE     ((uint16_t)2u)

/* Indexes inside packets. */
#define X_PACKET_NUMBER_INDEX             ((uint16_t)0u)
#define X_PACKET_NUMBER_COMPLEMENT_INDEX  ((uint16_t)1u)
#define X_PACKET_CRC_HIGH_INDEX           ((uint16_t)0u)
#define X_PACKET_CRC_LOW_INDEX            ((uint16_t)1u)

/* Bytes defined by the protocol. */
#define X_SOH ((uint8_t)0x01u)
#define X_STX ((uint8_t)0x02u)
#define X_EOT ((uint8_t)0x04u)
#define X_ACK ((uint8_t)0x06u)
#define X_NAK ((uint8_t)0x15u)
#define X_CAN ((uint8_t)0x18u) 
#define X_C   ((uint8_t)0x43u)
#define X_a   ((uint8_t)0x61u)

#define XMODEM_DEV "/dev/ttyS1" /* device USART2 as xmodem trans dev */

/* Status report for the functions. */
enum {
  XMODEM_OK            = 0x00u,
  XMODEM_ERROR_CRC     = 0x01u,
  XMODEM_ERROR_NUMBER  = 0x02u,
  XMODEM_ERROR_NUM_MIN = 0x10u,
  XMODEM_ERROR_UART    = 0x04u,
  XMODEM_ERROR_FLASH   = 0x08u,
  XMODEM_ERROR         = 0xFFu,
};

enum command_type
{
    FLASH = '1',
    APP = '2',
    EXIT = '3'

};

struct boot_xmodem_s
{
    int fd;
    bool first_packet_received;
    uint8_t xmodem_packet_number;
    uint8_t received_packet_number[X_PACKET_NUMBER_SIZE];
    uint8_t received_packet_data[X_PACKET_1024_SIZE];
    uint8_t received_packet_crc[X_PACKET_CRC_SIZE];
};

static uint16_t xmodem_calc_crc(uint8_t *data, uint16_t length);
static uint8_t xmodem_handle_packet(uint8_t header);
static uint8_t xmodem_error_handler(uint8_t *error_number, uint8_t max_error_number);
static void xmodem_receive(void);

void bootloader_workflow(void);
int xmodem_init(void);
void xmodem_deinit(void);

#endif /* XMODEM_H_ */
