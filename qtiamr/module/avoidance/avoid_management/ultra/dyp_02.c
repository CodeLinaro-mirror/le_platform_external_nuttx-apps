/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <syslog.h>
#include <sys/types.h>
#include <sys/ioctl.h>

#include "dyp_02.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define RS485_RECEIVE_RETRY_TIMES (20)    /* read time out N*100ms */
#define RS485_RECEIVE_WAIT        (15000) /*wait time before read data from sensor*/

#define FRAME_DATA_LEN (6) /* RW command frame length w/o crc16 */
#define FRAME_READ_LEN (7) /*  Return data frame length of read */
#define FRAME_FULL_LEN (8) /*  RW command frame length with crc16 */

#define BROADCAST_ADDR  (0xFF)
#define CONTROLLER_ADDR (0x01) /*default address*/
#define R_SINGLE_REG    (0x03) /*modbus protocol read*/
#define W_SINGLE_REG    (0x06) /*modbus protocol write*/

#define DIST_REG    (0x0100) /*handled value read only, ~500ms*/
#define RAWDIST_REG (0x0101) /*real-time value read only, ~100ms*/
#define TEMP_REG    (0x0102) /*temperature read only, ~100ms*/
#define ADDR_REG    (0x0200) /*slave address read write*/

#define DEBOUNCE_ACC (1.003f) /*accuracy (1+S*0.3%)*/
#define DYP_DEBUG    (0)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* DYP Ultra data frame structure */
struct rs485_data_msg
{
  uint8_t  addr; /*ultrasound sensor address*/
  uint8_t  cmd;  /*read dist 0x03 or write address 0x06*/
  uint16_t reg;  /*host register or slave read data len*/
  uint16_t data;
  uint16_t crc16;
} __attribute__((aligned(4)));

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Name: dist_debounce
 * Description:
 *  RS485 blind zone 3cm, range 3~450cm, accurancy +-(1+S*0.3%)cm
 *
 ****************************************************************************/
static int dist_debounce(int dist)
{
  int d_dist;

  d_dist = (int)(dist * DEBOUNCE_ACC);

  return d_dist;
}

/****************************************************************************
 * Name: crc16_modbus
 *
 * Description:
 *  The implementation of crc16_modbus: x^16 + x^15 + x^2 + 1
 *	XOR the data with pre-defined UCRC (0xFFFF) low 8-bit and assign to UCRC
 *  LOOP 8 times, >>1 and XOR 0XA001 if lowest bit !null, otherwise >> 1
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   crc16-modbus check code
 *
 ****************************************************************************/

static uint16_t crc16_modbus(const uint8_t *data, uint8_t data_len)
{
  uint16_t ucrc = 0xffff;
  uint8_t  i;

  while (data_len--)
    {
      ucrc ^= (*data++);

      for (i = 0; i < 8; i++)
        {
          if (ucrc & 0x0001)
            {
              ucrc = ucrc >> 1;
              ucrc ^= 0xA001;
            }
          else
            {
              ucrc = ucrc >> 1;
            }
        }
    }
  //syslog(LOG_DEBUG,"crc16: %x \n", ucrc);
  return ucrc;
}

/****************************************************************************
 * Name: rs485_frame_coding
 * Description:
 *	Register and register data put high byte first(low memory) in rs485 message, 
 *	but nuttx use little debian, need convert the message data first
 ****************************************************************************/

static void rs485_frame_coding(struct rs485_data_msg *msg, uint8_t buff_len)
{

  int16_t temp;

  if (!msg)
    return;

  /*Register and register data use big debian*/
  temp     = ((msg->reg & 0XFF) << 0x8) + ((msg->reg & 0XFF00) >> 0x8);
  msg->reg = temp;

  temp      = ((msg->data & 0XFF) << 0x8) + ((msg->data & 0XFF00) >> 0x8);
  msg->data = temp;

  /*send crc16 do not need convert*/
  temp       = crc16_modbus((uint8_t *)msg, FRAME_DATA_LEN);
  msg->crc16 = temp;
}

/****************************************************************************
 * Name: rs485_transmit
 * Description:
 *	Send rs485 command and receive the respone
 ****************************************************************************/

static int rs485_send_msg(uint8_t *buff, int fd)
{
  int ret;

  ret = write(fd, buff, FRAME_FULL_LEN);

  if (ret < 0)
    {
      syslog(LOG_INFO, "rs485 write failed\n");
      return ERROR;
    }

#if DYP_DEBUG
  printf("DEBUG send cmd: ");
  for (int i = 0; i < FRAME_FULL_LEN; i++)
    {
      printf("%X ", buff[i]);
    }
  printf("\n");
  syslog(LOG_DEBUG, "Waiting receive rs485 data ...\n");
#endif

  return ret;
}

static int rs485_receive_msg(int fd, bool is_read)
{
  int     retry = 0, count = 0, crc16 = 0, rec_crc16 = 0, ret = 0, data_len = 0;
  uint8_t rec_buff[10] = { 0 };

  if (is_read)
    data_len = FRAME_READ_LEN;
  else
    data_len = FRAME_FULL_LEN;

  usleep(RS485_RECEIVE_WAIT);

  while (retry < RS485_RECEIVE_RETRY_TIMES)
    {
      ret = read(fd, &rec_buff[count], data_len);
      //syslog(LOG_DEBUG,"read %d byte data\n", ret);

      if (ret > 0)
        {
          retry = 0;
          count += ret;

          if (count == data_len)
            {
              rec_crc16 = (rec_buff[data_len - 1] << 8) + (rec_buff[data_len - 2]);
              crc16     = crc16_modbus(rec_buff, data_len - 2);

              if (rec_crc16 == crc16)
                {
#if DYP_DEBUG
                  printf("Rec data %d byte:", ret);
                  for (int i = 0; i < ret; i++)
                    {
                      printf("%X ", rec_buff[i]);
                    }
                  printf("\n");
#endif
                  return ((rec_buff[data_len - 4] << 8) + rec_buff[data_len - 3]);
                }

              /*return error if crc check fail*/
              syslog(LOG_DEBUG, "RS485 CRC check failed\n");
              return ERROR;
            }
        }

      /*increase retry counter if read fail*/
      usleep(RS485_RECEIVE_WAIT);
      retry++;
    }

  syslog(LOG_INFO, "Recive time out!\n");
  return ERROR;
}

static int rs485_ultra_read(uint8_t addr, uint16_t reg, int fd)
{
  int                   ret;
  struct rs485_data_msg send_data;

  send_data.addr = addr;
  send_data.cmd  = R_SINGLE_REG;
  send_data.reg  = reg;
  send_data.data = 0x1;

  rs485_frame_coding(&send_data, FRAME_DATA_LEN);

  ret = rs485_send_msg((uint8_t *)&send_data, fd);

  if (ret > 0)
    ret = rs485_receive_msg(fd, 1);

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
int rs485_ultra_check(uint8_t addr, int fd)
{
  return rs485_ultra_read(addr, ADDR_REG, fd);
}

int rs485_ultra_raw_dist(uint8_t addr, int fd)
{
  int dist     = 0;
  int deb_dist = 0;

  dist = rs485_ultra_read(addr, RAWDIST_REG, fd);

  if ((dist > 0) && (dist < 5000))
    {
      deb_dist = dist_debounce(dist);

      syslog(LOG_DEBUG, "ultra sensor %u: dist %d, after debounce %d\n", addr, dist, deb_dist);
      return deb_dist;
    }
  else
    {
      //syslog(LOG_DEBUG, "ultra sensor %u get unexpected dist %d\n",addr, dist);
      return dist;
    }
}
