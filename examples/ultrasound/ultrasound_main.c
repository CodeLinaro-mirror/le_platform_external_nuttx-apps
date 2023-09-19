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

#include "ultrasound.h"

struct ulteasound_example_s g_ulteasound;
static uint16_t ulteasound_reg_arr[4] = {ADDR_REG, DIST_REG, RAWDIST_REG, TEMP_REG};
/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void ultrasound_help()
{
    printf("\nUsage: cap [OPTIONS]\n\n");
    printf("OPTIONS include:\n");
    printf("  [-p ultraso] Ultrasound device path\n");
    printf("  [-a addr] Device addr, default is broadcast address\n");
    printf("  [-r read] Read ultrasound reg data\n");
    printf("  [-w wirte] Wirte device addr\n");
    printf("  [-h] Shows this message and exits\n\n");

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
    int ret;

    ret = arg_string(arg, &string);
    *value = strtol(string, NULL, 10);
    return ret;
}

/****************************************************************************
 * Name: parse_args
 ****************************************************************************/
static void parse_args(int argc, FAR char **argv)
{
    FAR char *ptr;
    FAR char *str;
    long value=0;
    int index;
    int nargs;
    for (index = 1; index < argc; )
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
                nargs = arg_decimal(&argv[index], &value);
                if (value < 0 || value > 0xFF)
                {
                    exit(1);
                }

                printf("get -a %d\n",value);
                g_ulteasound.addr = value;
                index += nargs;
                break;

            case 'r':
                nargs = arg_decimal(&argv[index], &value);
                if (value < 0 || value > 4)
                {
                    exit(1);
                }
                printf("get -r %d\n",value);
                g_ulteasound.dir = R_SINGLE_REG;
                g_ulteasound.reg = ulteasound_reg_arr[value];
                index += nargs;
                break;

            case 'w':
                nargs = arg_decimal(&argv[index], &value);
                if (value < 0 || value > 0xFF)
                {
                    printf("out of range 0 ~ 0xFF\n");
                    exit(1);
                }
                printf("get -w %d\n",value);
                g_ulteasound.dir = W_SINGLE_REG;
                g_ulteasound.reg = ADDR_REG;
                g_ulteasound.cmd_data = value;
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
    uint8_t num =0;

    for(uint8_t num=0; num<data_len; num++)
    {
        ucrc = (*data++)^ucrc;
        for(uint8_t x=0;x<8;x++)
        {
            if(ucrc&0x0001)
            {
                ucrc = ucrc>>1;
                ucrc = ucrc^0xA001;
            }
            else
            {
                ucrc = ucrc>>1;
            }
        }
    }
    return ucrc;
}


static int rs485_send_msg(char *buff, int len, int fd)
{
    uint16_t crc16;
    uint32_t current = len;
    /* calculate crc */
    crc16 = crc16_modbus(buff, len);

    memcpy(&buff[current], &crc16, 2);
    crc16 = 0;
    current = current + 2;
    /* send fame to MC */
    write(fd, buff, current);
    printf("DEBUG send cmd: ");
    for(int i=0;i<current;i++)
    {
        printf("%x ",buff[i]);
    }
    printf("\n");
}

static int rs485_revice(uint8_t *rec_buff, int fd)
{
    int current= 0;
    int count =0, crc16 =0, rec_crc16=0;
    int read_size;
    printf("Waiting recive rs485 data ...\n");
    while ( current < RS485_RECEIVE_TIME_OUT )
    {
        read_size = read(fd, &rec_buff[count], 1);
        if (read_size > 0)
        {
            current =0;
            count = read_size + count;
            if(count > 2)
            {
                rec_crc16 = (rec_buff[count - 1] << 8 ) + (rec_buff[ count - 2 ]);
                crc16 = crc16_modbus(rec_buff, count - 2);
                if(rec_crc16 == crc16)
                {
                    return count;
                }
            }
        }
        usleep(RS485_RECEIVE_TIME_OUT/10);
        current = current + RS485_RECEIVE_TIME_OUT/10;
    }
    printf("Recive time out!\n");
    return ERROR;
}

static void us_cmd_frame_coding(uint8_t* buff, uint8_t buff_len, struct ulteasound_example_s *us_example)
{
    int16_t data;
    uint8_t current = 0;
    uint8_t i;
    if (buff ==NULL || us_example == NULL)
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
    int8_t dist;
    int32_t addr;
    int fd;
    int ret;

    uint8_t rec_buff[RS485_MSG_LEN];
    uint8_t send_buff[SINGLE_FRAME_LEN];

    printf("Start ultrasound test app\n");

    ultrasound_devpath(CONFIG_EXAMPLES_ULTRASOUND_DEVPATH);
    g_ulteasound.addr = BROADCAST_ADDR;
    g_ulteasound.reg = ADDR_REG;
    g_ulteasound.reg_len = 0x01;
    g_ulteasound.dir = R_SINGLE_REG;

    /* Parse command line arguments */
    parse_args(argc, argv);

    printf("ultrasound_main: Hardware initialized. Opening the ultrasound device: %s\n", g_ulteasound.devpath);

    fd = open(g_ulteasound.devpath, O_RDWR|O_NONBLOCK);

    if (fd < 0)
    {
        printf("ulteasound_main: open %s failed: %d\n", g_ulteasound.devpath, errno);
        return 0;
    }

    us_cmd_frame_coding(send_buff,SINGLE_FRAME_LEN,&g_ulteasound);
    rs485_send_msg(send_buff,SINGLE_FRAME_LEN,fd);
    ret = rs485_revice(rec_buff, fd);

    printf("Rec data %d byte:",ret );
    for(int i=0;i<ret;i++)
    {
        printf("%x ",rec_buff[i]);
    }
    printf("\nExit ultrasound test app \n");

  return 0;
}
