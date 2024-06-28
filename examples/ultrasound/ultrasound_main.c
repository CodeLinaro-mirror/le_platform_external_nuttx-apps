/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 * Copyright 2020 The Apache Software Foundation
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at

 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <limits.h>
#include <inttypes.h>
#include <errno.h>
#include <debug.h>
#include <syslog.h>

#include "ultrasound.h"

struct ulteasound_example_s g_ulteasound;
static uint16_t             ulteasound_reg_arr[4] = { ADDR_REG, DIST_REG, RAWDIST_REG, TEMP_REG };
static int                  read_delay            = 200000; /*default 200ms*/
static int                  addr_mask             = 0x1;
static int                  check_times           = 1;
/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void ultrasound_help(void)
{
  printf("\nUsage: cap [OPTIONS]\n\n");
  printf("OPTIONS include:\n");
  printf("  [-p path] Ultrasound device path\n");
  printf("  [-a addr] Sensor addr mask, use broadcast address if not set\n");
  printf("  [-r read] Read ultrasound reg data\n");
  printf("  [-w wirte] Wirte device addr\n");
  printf("  [-d delay] delay time before read\n");
  printf("  [-R Register] Register to control\n");
  printf("  [-h] Shows this message and exits\n\n");
  printf("  [-t] read times\n\n");

  printf("Read options:\n");
  printf("  0: Device address\n");
  printf("  1: Distance data\n");
  printf("  2: Raw distance data\n");
  printf("  3: Temperature data\n");
}

static void ultrasound_devpath(FAR const char *devpath)
{
  /* Get rid of any old device path */

  if (g_ulteasound.devpath)
    {
      free(g_ulteasound.devpath);
    }

  /* The set-up the new device path by copying the string */
  g_ulteasound.devpath = strdup(devpath);
}

static int arg_string(FAR char **arg, FAR char **value)
{
  FAR char *ptr = *arg;

  if (ptr[2] == '\0')
    {
      *value = arg[1];
      return 2;
    }
  else
    {
      *value = &ptr[2];
      return 1;
    }
}

/****************************************************************************
 * Name: arg_decimal
 ****************************************************************************/
static int arg_decimal(FAR char **arg, FAR long *value)
{
  FAR char *string;
  int       ret;

  ret    = arg_string(arg, &string);
  *value = strtol(string, NULL, 10);
  return ret;
}

static int arg_hex(FAR char **arg, FAR unsigned int *value)
{
  FAR char *string;
  int       ret;

  ret = arg_string(arg, &string);
  sscanf(string, "%x", value);
  return ret;
}
/****************************************************************************
 * Name: parse_args
 ****************************************************************************/
static void parse_args(int argc, FAR char **argv)
{
  FAR char *ptr;
  FAR char *str;
  long      value = 0;
  int       index;
  int       nargs;
  for (index = 1; index < argc;)
    {
      ptr = argv[index];
      if (ptr[0] != '-')
        {
          printf("Invalid options format: %s\n", ptr);
          exit(0);
        }

      switch (ptr[1])
        {
          case 'p':
            nargs = arg_string(&argv[index], &str);
            ultrasound_devpath(str);
            index += nargs;
            break;

          case 'a':
            nargs = arg_hex(&argv[index], (unsigned int *)&value);
            if (value < 0 || value > 0xFF)
              {
                exit(1);
              }

            printf("get -a %#x\n", (unsigned int)value);
            addr_mask = (unsigned int)value;
            index += nargs;
            break;

          case 'r':
            nargs = arg_decimal(&argv[index], &value);
            if (value < 0 || value > 4)
              {
                exit(1);
              }
            printf("get -r %ld\n", value);
            g_ulteasound.dir = R_SINGLE_REG;
            g_ulteasound.reg = ulteasound_reg_arr[value];
            index += nargs;
            break;

          case 't':
            nargs = arg_decimal(&argv[index], &value);
            if (value < 0)
              {
                exit(1);
              }
            printf("get -t %ld\n", value);
            check_times = (int)value;
            index += nargs;
            break;

          case 'w':
            nargs = arg_decimal(&argv[index], &value);
            if (value < 0 || value > 0xFF)
              {
                printf("out of range 0 ~ 0xFF\n");
                exit(1);
              }
            printf("get -w %ld\n", value);
            g_ulteasound.dir      = W_SINGLE_REG;
            g_ulteasound.reg      = ADDR_REG;
            g_ulteasound.cmd_data = value;
            index += nargs;
            break;

          case 'd':
            nargs = arg_decimal(&argv[index], &value);
            if (value < 0 || value > 500000)
              {
                printf("out of range 0 ~ 0xFF\n");
                exit(1);
              }
            printf("get -d %ld\n", value);
            read_delay = (int)value;
            index += nargs;
            break;

          case 'R':
            nargs = arg_hex(&argv[index], (unsigned int *)&value);
            if (value < 0 || value > 0X021F)
              {
                printf("Register out of range 0 ~ 200ms\n");
                exit(1);
              }
            printf("get -R %#x\n", (unsigned int)value);
            g_ulteasound.reg = (unsigned int)value;
            index += nargs;
            break;

          case 'h':
            ultrasound_help();
            exit(EXIT_SUCCESS);

          default:
            printf("Unsupported option: %s\n", ptr);
            ultrasound_help();
            exit(EXIT_FAILURE);
        }
    }
}

static uint16_t crc16_modbus(const uint8_t *data, uint8_t data_len)
{
  uint16_t ucrc = 0xffff;
  uint8_t  num;

  for (num = 0; num < data_len; num++)
    {
      ucrc = (*data++) ^ ucrc;
      for (uint8_t x = 0; x < 8; x++)
        {
          if (ucrc & 0x0001)
            {
              ucrc = ucrc >> 1;
              ucrc = ucrc ^ 0xA001;
            }
          else
            {
              ucrc = ucrc >> 1;
            }
        }
    }
  return ucrc;
}

static int rs485_send_msg(uint8_t *buff, int len, int fd)
{
  uint16_t crc16;
  uint32_t current = len;

  /* calculate crc */
  crc16 = crc16_modbus((uint8_t *)buff, len);
  memcpy(&buff[current], &crc16, sizeof(uint16_t));

  /* send fame to MC */
  write(fd, buff, FRAME_FULL_LEN);
  printf("DEBUG send cmd: ");
  for (int i = 0; i < FRAME_FULL_LEN; i++)
    {
      printf("%x ", buff[i]);
    }
  printf("\n");
  return 0;
}

static int rs485_revice(uint8_t *rec_buff, int fd)
{
  int current = 0;
  int count = 0, crc16 = 0, rec_crc16 = 0;
  int read_size, data_len;

  if (g_ulteasound.dir == W_SINGLE_REG)
    data_len = FRAME_FULL_LEN;
  else
    data_len = FRAME_FULL_LEN - 1;

  syslog(LOG_INFO, "Waiting recive rs485 data ...\n");
  usleep(read_delay);

  while (current < RS485_RECEIVE_TIME_OUT)
    {
      read_size = read(fd, &rec_buff[count], data_len);
      syslog(LOG_INFO, "read %d byte data\n", read_size);

      if (read_size > 0)
        {
          current = 0;
          count += read_size;

          if (count == data_len)
            {
              rec_crc16 = (rec_buff[data_len - 1] << 8) + (rec_buff[data_len - 2]);
              crc16     = crc16_modbus(rec_buff, data_len - 2);
              if (rec_crc16 == crc16)
                {
                  syslog(LOG_INFO, "Get data %d \n",
                         ((rec_buff[data_len - 4] << 8) + (rec_buff[data_len - 3])));
                  return count;
                }
            }
        }

      usleep(read_delay);
      current += read_delay;
    }
  syslog(LOG_INFO, "Recive time out!\n");
  return ERROR;
}

static void us_cmd_frame_coding(uint8_t *buff, uint8_t buff_len, struct ulteasound_example_s *us_example)
{
  int16_t data;
  uint8_t current = 0;
  if (buff == NULL || us_example == NULL)
    {
      return;
    }

  memcpy(&buff[current], &us_example->addr, 1); //addr &id
  current++;

  memcpy(&buff[current], &us_example->dir, 1); //addr &id
  current++;

  data = ((us_example->reg & 0xff) << 0x8) + ((us_example->reg & 0xff00) >> 0x8);
  memcpy(&buff[current], &data, 2);
  current += 2;

  data = ((us_example->cmd_data & 0xff) << 0x8) + ((us_example->cmd_data & 0xff00) >> 0x8);
  memcpy(&buff[current], &data, 2);
}

int main(int argc, FAR char *argv[])
{
  int fd;
  int ret;

  uint8_t rec_buff[RS485_MSG_LEN];
  uint8_t send_buff[FRAME_FULL_LEN];

  printf("ultrasound_main: Init..\n");

  ultrasound_devpath(CONFIG_EXAMPLES_ULTRASOUND_DEVPATH);
  g_ulteasound.addr    = BROADCAST_ADDR;
  g_ulteasound.reg     = ADDR_REG;
  g_ulteasound.reg_len = 0x01;
  g_ulteasound.dir     = R_SINGLE_REG;

  /* Parse command line arguments */
  parse_args(argc, argv);

  printf("ultrasound_main: Opening device: %s\n", g_ulteasound.devpath);

  fd = open(g_ulteasound.devpath, O_RDWR | O_NONBLOCK);

  if (fd < 0)
    {
      printf("ulteasound_main: open %s failed: %d\n", g_ulteasound.devpath, errno);
      return 0;
    }

  if (g_ulteasound.dir == W_SINGLE_REG)
    {
      us_cmd_frame_coding(send_buff, SINGLE_FRAME_LEN, &g_ulteasound);
      rs485_send_msg(send_buff, SINGLE_FRAME_LEN, fd);
      ret = rs485_revice(rec_buff, fd);

      printf("Rec data %d byte:", ret);

      if (ret > 0)
        {
          for (int i = 0; i < ret; i++)
            {
              printf("%x ", rec_buff[i]);
            }
          printf("\n\n");
        }

      close(fd);
      return 0;
    }

  for (int t = 0; t < check_times; t++)
    {
      for (int a = 0; a < 8; a++)
        {
          if (!(addr_mask & (0x1 << a)))
            continue;

          g_ulteasound.addr = a + 1;
          us_cmd_frame_coding(send_buff, SINGLE_FRAME_LEN, &g_ulteasound);
          rs485_send_msg(send_buff, SINGLE_FRAME_LEN, fd);
          ret = rs485_revice(rec_buff, fd);

          printf("Rec data %d byte:", ret);

          if (ret > 0)
            {
              for (int i = 0; i < ret; i++)
                {
                  printf("%x ", rec_buff[i]);
                }
              printf("\n\n");
            }

          usleep(10000);
        }
    }

  printf("Read finished \n\n");
  close(fd);
  return 0;
}
