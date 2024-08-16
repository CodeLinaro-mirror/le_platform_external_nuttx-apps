/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/
#include <stdio.h>
#include <unistd.h>
#include <syslog.h>
#include <string.h>

#include "main.h"
#include "qrc.h"
#include "config_msg.h"
#include "emergency_msg.h"
#include "qrc_msg_management.h"

#include "avoidance.h"
#include "avoid_management.h"
#include "emergency_avoidance.h"
#include "motion_management.h"
#include "motion_sm.h"

/****************************************************************************
 * Public data
 ****************************************************************************/
#define ULTRA_DIRECTION    (0)

static struct avoid_client emerg_client;
static struct qrc_pipe_s * emerg_pipe    = NULL;
static rmutex_t sensor_mask_lock = NXRMUTEX_INITIALIZER;

/****************************************************************************
 * Private Functions
 ****************************************************************************/
#if ULTRA_DIRECTION
static int                 pre_direction = FORWARD;
static bool emerg_direction_cb(float vx, float vz)
{
  int     direction  = 0;
  float   div        = 0;
  uint8_t sensormask = 0;

  nxrmutex_lock(&sensor_mask_lock);

  if (vx < 0.0)
    {
      direction  = BACKWARD;
      sensormask = 0x70;
    }
  else
    {
      if (!vz)
        {
          direction  = FORWARD;
          sensormask = 0X70;
          goto update;
        }

      div = vx / vz;
      syslog(LOG_DEBUG, "Emerg direction vx=%.2f, vz=%.2f, div=%.2f\n", vx, vz, div);

      if ((vz > 0.0) && (abs(div) < 1.0))
        {
          direction  = TURN_LEFT;
          sensormask = 0x38;
        }
      else if ((vz < 0.0) && (abs(div) < 1.0))
        {
          direction  = TURN_RIGHT;
          sensormask = 0X54;
        }
      else
        {
          direction  = FORWARD;
          sensormask = 0X70;
        }
    }

update:
  if (direction != pre_direction)
    {
      update_sensor_check_list(sensormask);
      emerg_client.trigger &= ((sensormask << 1) | 0x1);
      pre_direction = direction;
      syslog(LOG_INFO, "Ultrasound update sensormask: %#X ,client trigger %#X \n", sensormask, emerg_client.trigger);
    }
  nxrmutex_unlock(&sensor_mask_lock);

  return 0;
}
#endif

/*AMR could go backward if emergency stop triggered, so return TRUE if vx < 0  */
static bool emerg_speed_cb(float vx, float vz)
{
  return ((vx > 0) || (!vx && vz))  ? FALSE : TRUE;
}

/*callback of qrc_message*/
static void emerg_qrc_msg_parse(struct qrc_pipe_s *pipe, struct emerg_msg_s *emerg_msg)
{
  int                ret;
  struct emerg_msg_s emerg_msg_reply = { 0 };

  if (!pipe || !emerg_msg)
    return;

  switch (emerg_msg->msg_type)
    {
      case ENABLEMENT:
        syslog(LOG_INFO, "Received emergency enablement %d \n", emerg_msg->data.value);
        emerg_msg_reply.msg_type   = ENABLEMENT;
        emerg_msg_reply.data.value = 1;
        if (emerg_msg->data.value)
          {
            register_ultra_client(&emerg_client);
          }
        else
          {
            motion_motor_stop(FALSE);
            unregister_ultra_client(&emerg_client);
          }
        break;

      case EVENT:
        syslog(LOG_DEBUG, "emerg event received\n");
        return;

      default:
        syslog(LOG_DEBUG, "emerg receive qrc msg unknown\n");
        return;
    }

  ret = qrc_write(pipe, (uint8_t *)&emerg_msg_reply, sizeof(struct emerg_msg_s), false);
  if (ret != SUCCESS)
    {
      syslog(LOG_ERR, "emerg_qrc_msg responde failed\n");
    }
}
static void emerg_msg_cb(struct qrc_pipe_s *pipe, void *data, size_t len, bool response)
{
  struct emerg_msg_s *emerg_msg;

  if (!pipe || !data)
    return;

  if (len == sizeof(struct emerg_msg_s))
    {
      emerg_msg = (struct emerg_msg_s *)data;
      emerg_qrc_msg_parse(pipe, emerg_msg);
    }
  else
    {
      syslog(LOG_ERR, "emerg qrc message received fail, need:%d, actual:%d \n",
             sizeof(struct emerg_msg_s), len);
    }
}

/*callback of avoidance client*/
static void emerg_client_cb(uint8_t addr, uint16_t dist, bool enter)
{
  int                ret = 0;
  struct emerg_msg_s msg = { 0 };

  nxrmutex_lock(&sensor_mask_lock);
  if (enter)
    {
      msg.msg_type                  = EVENT;
      msg.data.event.type           = ENTER;
      msg.data.event.trigger_sensor = (int)addr;

      if (!(emerg_client.trigger & 0x1))
        {
          motion_motor_stop(TRUE);
          emerg_client.trigger |= 0x1;
        }
    }
  else
    {
      msg.msg_type                  = EVENT;
      msg.data.event.type           = EXIT;
      msg.data.event.trigger_sensor = (int)addr;
      motion_motor_stop(FALSE);
      emerg_client.trigger = 0;
    }
  nxrmutex_unlock(&sensor_mask_lock);

  if (!emerg_pipe)
    {
      syslog(LOG_ERR, "emerg pipe NULL\n");
      return;
    }

  ret = qrc_write(emerg_pipe, (uint8_t *)&msg, sizeof(struct emerg_msg_s), false);
  if (ret != SUCCESS)
    {
      syslog(LOG_ERR, "emerg send event qrc_msg failed\n");
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int emergency_main(int argc, char *argv[])
{
  int                                ret = 0;
  struct config_obstacle_avoidance_s obs_avoid_param;

  if (!is_ultra_enabled())
    {
      syslog(LOG_INFO, "emergency_main exit cause ultra_enable disabled\n");
      config_notify_completed(true);
      return ret;
    }

  if (!is_avoidance_inited())
    {
      syslog(LOG_INFO, "emergency_main exit cause avoidance init failed\n");
      config_notify_completed(false);
      return ret;
    }
  emerg_client.name    = EMERG_PIPE;
  emerg_client.trigger = 0;
  emerg_client.cb      = emerg_client_cb;

  ret = get_configuration_parameters(OBSTACLE_AVOIDANCE, (void *)&obs_avoid_param);
  if (ret < 0)
    {
      syslog(LOG_ERR, "emergency avoidance parameters get failed\n");
      config_notify_completed(false);
      return ERROR;
    }

  emerg_client.thres_bottom = (uint16_t)(obs_avoid_param.bottom_dist * 1000);
  emerg_client.thres_side   = (uint16_t)(obs_avoid_param.side_dist * 1000);
  emerg_client.thres_front  = (uint16_t)(obs_avoid_param.front_dist * 1000);
  syslog(LOG_INFO, "emergency threshold (b,s,f)-(%d,%d,%d) \n",
         emerg_client.thres_bottom, emerg_client.thres_side, emerg_client.thres_front);

  emerg_pipe = qrc_get_pipe(EMERG_PIPE);
  if (!emerg_pipe)
    {
      syslog(LOG_ERR, "emergency avoidance get pipe failed\n");
      config_notify_completed(false);
      return ERROR;
    }

  if (!qrc_register_message_cb(emerg_pipe, emerg_msg_cb))
    {
      syslog(LOG_ERR, "emergency stop register qrc_msg callback failed\n");
      config_notify_completed(false);
      return ERROR;
    }

  register_emergency_check_speed_cb(emerg_speed_cb);

  #if ULTRA_DIRECTION
    register_speed_subscribe_cb(emerg_direction_cb);
  #endif

  /*Enable emergency avoidance by default*/
  register_ultra_client(&emerg_client);

  syslog(LOG_DEBUG, "emergency avoidance init done\n");
  config_notify_completed(true);

  return ret;
}
