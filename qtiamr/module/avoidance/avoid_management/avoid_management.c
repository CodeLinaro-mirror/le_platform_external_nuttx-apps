/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <syslog.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/ioctl.h>

#include "dyp_02.h"
#include "avoidance.h"
#include "avoid_management.h"
#include "modlog_filter.h"
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define RETRY_NUM 3

/****************************************************************************
 * Public data
 ****************************************************************************/
static struct avoid_sensor g_ultra_sensor_list[SENSOR_MAX] = {
  { BOTTOM, 0X1, 0 },
  { BOTTOM, 0X2, 0 },
  { SIDE, 0X3, 0 },
  { SIDE, 0X4, 0 },
  { FRONT, 0X5, 0 },
  { FRONT, 0X6, 0 },
  { FRONT, 0X7, 0 },
};

static uint8_t ultra_sensor_masks = 0XF8;
static rmutex_t avoid_lock    = NXRMUTEX_INITIALIZER;

/****************************************************************************
 * Pravite Function
 ****************************************************************************/
void check_client_trigger(struct list_node *avoid_client_list, struct avoid_sensor *sensor, int dist)
{
  bool                 thres_meet = FALSE;
  struct avoid_client *client;

  nxrmutex_lock(&avoid_lock);
  list_for_every_entry(avoid_client_list, client, struct avoid_client, node)
  {
    switch (sensor->type)
      {
        case BOTTOM:
          thres_meet = dist > client->thres_bottom;
          break;
        case SIDE:
          thres_meet = dist < client->thres_side;
          break;
        case FRONT:
          thres_meet = dist < client->thres_front;
          break;
        default:
          syslog(LOG_ERR, "sensor type unrecognized\n");
          nxrmutex_unlock(&avoid_lock);
          return;
      }

    if (thres_meet && ((0x1 << sensor->addr) & ultra_sensor_masks))
      {
        client->trigger |= (0x1 << sensor->addr);
        syslog(LOG_INFO, "!!!!!! [%#X] Sensor %d triggerd emergency stop: %d\n",
               client->trigger, sensor->addr, dist);

        client->cb(sensor->addr, dist, TRUE);
      }
    else
      {
        if (((client->trigger >> sensor->addr) & 0x1) && (++(sensor->count) >= RETRY_NUM))
          {
            client->trigger ^= (0x1 << sensor->addr);
            sensor->count = 0;
            syslog(LOG_INFO, "!!!!!! [%#X] Sensor %d exit emergency stop: %d\n",
                   client->trigger, sensor->addr, dist);
          }
      }

    if (!(client->trigger ^ 0x1))
      {
        client->cb(sensor->addr, dist, FALSE);
      }
  }
  nxrmutex_unlock(&avoid_lock);
}

/****************************************************************************
 * Public Function
 ****************************************************************************/
struct avoid_sensor *get_ultra_sensor_list(void)
{
  return g_ultra_sensor_list;
}

void update_sensor_check_list(uint8_t mask)
{
  ultra_sensor_masks = mask;
}

int avoid_init(void)
{
  int fd;
  int ret;
  int ultra_sensor_num = get_ultra_num();
  int retry = 0;

  fd = open(ULTRASOUND_DEV, O_RDWR | O_NONBLOCK);
  if (fd < 0)
    {
      syslog(LOG_ERR, "ultrasound device open failed \n");
      return fd;
    }

  syslog(LOG_DEBUG, "ultrasound device open  %s done \n", ULTRASOUND_DEV);

  usleep(100000 * 2);

  for (int i = 0; i < ultra_sensor_num; i++)
    {
      retry = 0;
      ret = rs485_ultra_init(g_ultra_sensor_list[i].addr, fd);

      if (ret <= 0 && retry < RETRY_NUM)
        {
          syslog(LOG_INFO, "ultrasound sensor %u unreachable, retry %d \n", g_ultra_sensor_list[i].addr, retry);

          ret = rs485_ultra_init(g_ultra_sensor_list[i].addr, fd);
          retry++;
        }
    }

  syslog(LOG_DEBUG, "avoid init successfully \n");
  close(fd);
  return OK;
}

int avoid_management_thread(int argc, char *argv[])
{
  int      dist = 0;
  int      fd   = -1;
  sigset_t set;

  struct avoid_sensor *sensor;

  int               ultra_sensor_num  = get_ultra_num();
  struct list_node *avoid_client_list = get_avoid_client_list();

  sigemptyset(&set);
  sigaddset(&set, AVOID_WAKEUP);
  sigprocmask(SIG_UNBLOCK, &set, NULL);

  while (1)
    {
      if (list_is_empty(avoid_client_list))
        {
          if (fd > 0)
            {
              syslog(LOG_DEBUG, "[avoidance mangement] close ultra dev %d\n", fd);
              close(fd);
              fd = -1;
            }
          syslog(LOG_INFO, "[avoidance mangement] client empty, pending...\n");
          sigwaitinfo(&set, NULL);
          syslog(LOG_INFO, "[avoidance mangement] receive client register...\n");
        }

      if (fd == -1)
        {
          fd = open(ULTRASOUND_DEV, O_RDWR | O_NONBLOCK);
          syslog(LOG_DEBUG, "[avoidance mangement] open ultra dev %d\n", fd);

          if (fd < 0)
            {
              syslog(LOG_ERR, "ultrasound device open failed \n");
              return fd;
            }
        }

      modlog_dbg(LOG_ULEMERG, "[avoidance mangement]Ultra sensor dist, unit(mm)\n");
      for (int i = 1; i <= ultra_sensor_num; i++)
        {
          if (!(ultra_sensor_masks & (0x1 << i)))
            continue;

          sensor = &g_ultra_sensor_list[i-1];
          dist   = rs485_ultra_raw_dist(sensor->addr, fd);
          if (dist <= 0)
            {
              modlog_dbg(LOG_ULEMERG, "sensor %d read fail or untrusted distance: %d \n",
                     sensor->addr, dist);
              continue;
            }

          if (dist == 0XFFFD)
            {
              modlog_dbg(LOG_ULEMERG, "ultra sensor %u no object detected\n", sensor->addr);
            }

          check_client_trigger(avoid_client_list, sensor, dist);
          usleep(1000);
        }
    }
}
