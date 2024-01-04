/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/
 #ifnded _MODULE_DYP_02_H
 #define _MODULE_DYP_02_H

 
 #include <stdio.h>
 #include <stdint.h>

#define RS485_MSG_LEN (16) /* 8 bytes message len & 8 for extern */
#define RS485_RECEIVE_TIME_OUT (200000) /* read time out */

#define SINGLE_FRAME_LEN (6)	/* No CRC frame*/
#define RETURN_RAME_WRITE_SIZE	(8)	/* write frame length */
#define RETURN_RAME_READ_SINGLE_SIZE	(7)	/* readed data length */

#define BROADCAST_ADDR  (0xFF)
#define CONTROLLER_ADDR (0x01)
#define R_SINGLE_REG	(0x03)
#define W_SINGLE_REG	(0x06)

#define DIST_REG        (0x0100)
#define RAWDIST_REG     (0x0101)
#define TEMP_REG        (0x0102)
#define ADDR_REG        (0x0200)

int rs485_ultra_read(void *buff, uint8_t len, int fd);
int rs485_ultra_write(void *data, uint8_t len, int fd);


 #endif

