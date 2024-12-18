/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include "qrc_msg_management.h"
#include "time_sync.h"
#include "main.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static void config_timesync_qrc_msg_cb(struct qrc_pipe_s *pipe, void *data,
                                       size_t len, bool response);
static void wakeup_ts_thread(void);
static void timesync_wait_cmd(void);
static void timesync_handle_cmd(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/
static struct qrc_pipe_s *    g_timesync_pipe = NULL;
static struct time_sync_msg_s g_timesync_msg;
static pthread_cond_t         ts_wait_cond;
static pthread_mutex_t        ts_mutex = PTHREAD_MUTEX_INITIALIZER;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static void config_timesync_qrc_msg_cb(struct qrc_pipe_s *pipe, void *data,
                                       size_t len, bool response)
{
  struct time_sync_msg_s *msg;
  size_t                  exp_len = sizeof(struct time_sync_msg_s);

  if (pipe == NULL || data == NULL)
    {
      return;
    }
  if (len == sizeof(struct time_sync_msg_s))
    {
      msg = (struct time_sync_msg_s *)data;
      memcpy(&g_timesync_msg, msg, sizeof(struct time_sync_msg_s));
      wakeup_ts_thread();
    }
  else
    {
      syslog(LOG_ERR, "timesync msg: msg size mismatch, len %u, exp: %u\n",
             len, exp_len);
    }
}

static void wakeup_ts_thread(void)
{
  pthread_mutex_lock(&ts_mutex);
  pthread_cond_signal(&ts_wait_cond);
  pthread_mutex_unlock(&ts_mutex);
}

static void timesync_wait_cmd(void)
{
  pthread_mutex_lock(&ts_mutex);

  if (0 != pthread_cond_wait(&ts_wait_cond, &ts_mutex))
    {
      syslog(LOG_ERR, "timesync cond wait failed\n");
    }

  pthread_mutex_unlock(&ts_mutex);
}

static void timesync_handle_cmd(void)
{
  struct time_sync_msg_s msg;
  struct timespec        ts;

  if (g_timesync_msg.type == TIME_LOOP)
    {
      msg.type = TIME_LOOP;
      if (SUCCESS != qrc_write_fast(g_timesync_pipe, (void *)&msg,
                                    sizeof(struct time_sync_msg_s)))
        {
          syslog(LOG_ERR, "timesync, timeloop send response failed\n");
        }
      syslog(LOG_DEBUG, "timesync, timeloop send response done\n");
    }
  else if (g_timesync_msg.type == GET_TIME)
    {
      clock_gettime(CLOCK_REALTIME, &ts);
      msg.type = GET_TIME;
      msg.sec  = ts.tv_sec;
      msg.ns   = ts.tv_nsec;
      if (SUCCESS != qrc_write_fast(g_timesync_pipe, (void *)&msg,
                                    sizeof(struct time_sync_msg_s)))
        {
          syslog(LOG_ERR, "timesync, getime send response failed\n");
        }
      syslog(LOG_INFO, "timesync, gettime response done\n");
    }
  else if (g_timesync_msg.type == SET_TIME)
    {
      ts.tv_sec  = g_timesync_msg.sec;
      ts.tv_nsec = g_timesync_msg.ns;
      clock_settime(CLOCK_REALTIME, &ts);
      syslog(LOG_INFO, "timesync, set time done\n");

      msg.type = SET_TIME;
      if (SUCCESS != qrc_write_fast(g_timesync_pipe, (void *)&msg,
                                    sizeof(struct time_sync_msg_s)))
        {
          syslog(LOG_ERR, "timesync, settime response failed\n");
        }
    }
  else
    {
      syslog(LOG_ERR, "timesync, not support msg\n");
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: time_sync Thread function
 ****************************************************************************/
int time_sync_thread(int argc, char *argv[])
{
  char pipe_name[] = TIME_SYNC_PIPE;

  /* get qrc pipe */
  g_timesync_pipe = qrc_get_pipe(pipe_name);
  if (g_timesync_pipe == NULL)
    {
      /* notify error */
      config_notify_completed(false);
      return -1;
    }

  /* register timesync cb*/
  if (!qrc_register_message_cb(g_timesync_pipe, config_timesync_qrc_msg_cb))
    {
      config_notify_completed(false);
      return -1;
    }

  /* init wait condition */
  if (0 != pthread_cond_init(&ts_wait_cond, NULL))
    {
      syslog(LOG_ERR, "timesync cond init failed\n");
    }

  /* notify init done */
  config_notify_completed(true);

  /* wait cmds and handle them */
  while (1)
    {
      timesync_wait_cmd();
      timesync_handle_cmd();
    }

  return 0;
}