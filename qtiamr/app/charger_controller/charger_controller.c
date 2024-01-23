/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#include <stdio.h>
#include <pthread.h>
#include <syslog.h>
#include <signal.h>
#include <assert.h>

#include "qrc_msg_management.h"
#include "charger_management.h"
#include "motion_management.h"
#include "charger_control_msg.h"
#include "main.h"
#include "charger_device.h"


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/
struct qrc_pipe_s *g_charger_pipe = NULL;

/* only for debug */
static const char * sm_ctl_labels[] = {
  [CHR_SM_IDLE] =           "idle",
  [CHR_SM_SEARCHING] =      "searching",
  [CHR_SM_CONTROLLING] =    "controlling",
  [CHR_SM_FORCE_CHARGING] = "force charging",
  [CHR_SM_CHARGING] =       "charging",
  [CHR_SM_CHARGER_DONE] =   "charging done",
  [CHR_SM_EXCEPTION] =      "sm exception",
};

static const char * exception_ctl_labels[] = {
  [CHARGER_VOLTAGE_ERROR] =           "exception: voltage error!",
  [CHARGER_CHARGING_CURRENT_ERROR] =  "exception: current error!",
  [CHARGER_PILE_STAT_ERROR] =         "exception: pile state error!",
  [CHARGER_IS_CHARGING_ERROR] =       "exception: is charging error!",
  [CHARGER_GET_SPEED_ERROR] =         "exception: get speed error!",
  [CHARGER_SM_STATE_ERROR] =          "exception: sm state error!",
  [CHARGER_DEV_NOT_READY_ERROR] =     "exception: device not ready error!",
  [CHARGER_ENTER_EXCEPTION] =         "exception: common error!",
};

static const char * msg_type[] = {
  [KEY_VOLTAGE] =         "notify voltage",
  [KEY_CURRENT] =         "notify current",
  [KEY_SM_STATE] =        "notify sm state",
  [KEY_EXCEPTION] =       "notify exception",
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void charger_qrc_msg_send(struct charger_ctl_msg_s *ctl_msg_data){

  enum qrc_write_status_e result;
  if (g_charger_pipe == NULL)
    {
      return;
    }
  result = qrc_write(g_charger_pipe , (void *) ctl_msg_data, sizeof(struct charger_ctl_msg_s), true);
  if (result != SUCCESS)
   {
      syslog(LOG_ERR,"charger_qrc_msg_send: qrc write pipe failed(%d)!\n",result);
   }
  return;
}

static void charger_management_msg_print_debug(struct charger_msg_s *charger_msg)
{

   switch(charger_msg->msg_type)
    {
      case KEY_VOLTAGE:
         syslog(LOG_DEBUG, "CHARGER CONTROLLER: %s:(%d), msg_data:(%fV)", msg_type[charger_msg->msg_type],charger_msg->msg_type, charger_msg->data.voltage);
         break;
      case KEY_CURRENT:
         syslog(LOG_DEBUG, "CHARGER CONTROLLER: %s:(%d), msg_data:(%fA)", msg_type[charger_msg->msg_type],charger_msg->msg_type, charger_msg->data.current);
         break;
      case KEY_SM_STATE:
         syslog(LOG_DEBUG, "CHARGER CONTROLLER: %s:(%d), msg_data:%s(%d)", msg_type[charger_msg->msg_type],charger_msg->msg_type,sm_ctl_labels[charger_msg->data.sm_state] ,charger_msg->data.sm_state);
         break;
      case KEY_EXCEPTION:
         syslog(LOG_DEBUG, "CHARGER CONTROLLER: %s:(%d), msg_data:%s(%d)", msg_type[charger_msg->msg_type],charger_msg->msg_type,exception_ctl_labels[charger_msg->data.exception_err], charger_msg->data.exception_err);
         break;
      default:
         break;
   }
   return;
}

static void charger_send_qrc_async_msg_cb(struct charger_msg_s *charger_msg){

    enum qrc_write_status_e result;

    charger_management_msg_print_debug(charger_msg);
  
    if (g_charger_pipe == NULL)
      {
        return;
      }

    /* write charger command with ack */
    result = qrc_write(g_charger_pipe, (void *)&charger_msg, sizeof(struct charger_msg_s), true);
    if (result != SUCCESS)
      {
        syslog(LOG_ERR, "Motion odom send failed %d\n", result);
      }
    return;
}

static void charger_motion_speed_send_cb(float vx, float vz)
{
  int ret;
  ret = motion_speed_control(CHARGER_CONTROLLER, vx,vz);
  if (ret != OK)
    {
      syslog(LOG_DEBUG, "CHARGER CONTROLLER: motion_speed_control error(%d): vx = (%f), vz = (%f)\n",ret, vx, vz);
    }
  return;
}

static void charger_controller_msg_parse(struct qrc_pipe_s *pipe, struct charger_ctl_msg_s *control_msg)
{
  struct charger_ctl_msg_s qrc_ctl_msg_send;
  float voltage;
  float current;
  bool charger_pile;
  bool is_charging;
  
  int ret;
  
  if (pipe == NULL || control_msg ==NULL)
  {
    return;
  }
  
   switch(control_msg->cmd_type)
    {

        case GET_CTL_VOLTAGE:
          memset(&qrc_ctl_msg_send, 0, sizeof(struct charger_ctl_msg_s));
          ret = charger_dev_get_voltage(&voltage);
          if(ret == ERROR)
            {
              qrc_ctl_msg_send.cmd_type = GET_CTL_EXCEPTION;
              qrc_ctl_msg_send.cmd_data.exception_value = CHARGER_VOLTAGE_ERROR;
              charger_qrc_msg_send(&qrc_ctl_msg_send);
              break;
            }
          qrc_ctl_msg_send.cmd_type = GET_CTL_VOLTAGE;
          qrc_ctl_msg_send.cmd_data.voltage = voltage;
          syslog(LOG_DEBUG, "CHARGER CONTROLLER: charger_controller_msg_parse: get_voltage = (%fV)\n", qrc_ctl_msg_send.cmd_data.voltage);
          charger_qrc_msg_send(&qrc_ctl_msg_send);
          break;

        case GET_CTL_CURRENT:
          memset(&qrc_ctl_msg_send, 0, sizeof(struct charger_ctl_msg_s));
          ret = charger_dev_get_charging_current(&current);
          if(ret == ERROR)
            {
              qrc_ctl_msg_send.cmd_type = GET_CTL_EXCEPTION;
              qrc_ctl_msg_send.cmd_data.exception_value = CHARGER_CHARGING_CURRENT_ERROR;
              charger_qrc_msg_send(&qrc_ctl_msg_send);
              break;
            }
          qrc_ctl_msg_send.cmd_type = GET_CTL_CURRENT;
          qrc_ctl_msg_send.cmd_data.current = current;
          syslog(LOG_DEBUG, "CHARGER CONTROLLER: charger_controller_msg_parse: get_current = (%fA)\n", qrc_ctl_msg_send.cmd_data.current);
          charger_qrc_msg_send(&qrc_ctl_msg_send);
          break;

        case GET_CTL_PILE_STATE:
          memset(&qrc_ctl_msg_send, 0, sizeof(struct charger_ctl_msg_s));
          ret = charger_dev_get_pile_stats(&charger_pile);
          if(ret == ERROR)
            {
              qrc_ctl_msg_send.cmd_type = GET_CTL_EXCEPTION;
              qrc_ctl_msg_send.cmd_data.exception_value = CHARGER_PILE_STAT_ERROR;
              charger_qrc_msg_send(&qrc_ctl_msg_send);
              break;
            }
          qrc_ctl_msg_send.cmd_type = GET_CTL_PILE_STATE;
          qrc_ctl_msg_send.cmd_data.pile_stats = (uint32_t)charger_pile;
          syslog(LOG_DEBUG, "CHARGER CONTROLLER: charger_controller_msg_parse: get_pile = (%ld)\n", qrc_ctl_msg_send.cmd_data.pile_stats);
          charger_qrc_msg_send(&qrc_ctl_msg_send);

          break;

          case GET_CTL_IS_CHARGING:
            memset(&qrc_ctl_msg_send, 0, sizeof(struct charger_ctl_msg_s));
            ret = charger_dev_get_is_charging_stats(&is_charging);
            if(ret == ERROR)
              {
                qrc_ctl_msg_send.cmd_type = GET_CTL_EXCEPTION;
                qrc_ctl_msg_send.cmd_data.exception_value = CHARGER_IS_CHARGING_ERROR;
                charger_qrc_msg_send(&qrc_ctl_msg_send);
                break;
              }
            qrc_ctl_msg_send.cmd_type = GET_CTL_IS_CHARGING;
            qrc_ctl_msg_send.cmd_data.pile_stats = (uint32_t)is_charging;
            syslog(LOG_DEBUG, "CHARGER CONTROLLER: charger_controller_msg_parse: get_pile = (%ld)\n", qrc_ctl_msg_send.cmd_data.is_charging);
            charger_qrc_msg_send(&qrc_ctl_msg_send);
          
            break;

        case GET_CTL_SM_STATE:
          memset(&qrc_ctl_msg_send, 0, sizeof(struct charger_ctl_msg_s));
          qrc_ctl_msg_send.cmd_data.sm_state = charger_dev_get_sm_stats();
          qrc_ctl_msg_send.cmd_type = GET_CTL_SM_STATE;
          syslog(LOG_DEBUG, "CHARGER CONTROLLER: charger_controller_msg_parse: get_sm_state = (%ld)\n", qrc_ctl_msg_send.cmd_data.sm_state);
          charger_qrc_msg_send(&qrc_ctl_msg_send);
          break;

        case GET_CTL_ALL_STATE:
          ret = charger_dev_get_all_stats(&voltage, &current, &charger_pile, &is_charging);
          if(ret == ERROR)
            {
              qrc_ctl_msg_send.cmd_type = GET_CTL_EXCEPTION;
              qrc_ctl_msg_send.cmd_data.exception_value = CHARGER_CTL_GET_ALL_ERROR;
              charger_qrc_msg_send(&qrc_ctl_msg_send);
              break;
            }
          memset(&qrc_ctl_msg_send, 0, sizeof(struct charger_ctl_msg_s));
          qrc_ctl_msg_send.cmd_type = GET_CTL_VOLTAGE;
          qrc_ctl_msg_send.cmd_data.voltage = voltage;
          charger_qrc_msg_send(&qrc_ctl_msg_send);

          memset(&qrc_ctl_msg_send, 0, sizeof(struct charger_ctl_msg_s));
          qrc_ctl_msg_send.cmd_type = GET_CTL_CURRENT;
          qrc_ctl_msg_send.cmd_data.current = current;
          charger_qrc_msg_send(&qrc_ctl_msg_send);

          memset(&qrc_ctl_msg_send, 0, sizeof(struct charger_ctl_msg_s));
          qrc_ctl_msg_send.cmd_type = GET_CTL_PILE_STATE;
          qrc_ctl_msg_send.cmd_data.pile_stats = (uint32_t)charger_pile;
          charger_qrc_msg_send(&qrc_ctl_msg_send);

          memset(&qrc_ctl_msg_send, 0, sizeof(struct charger_ctl_msg_s));
          qrc_ctl_msg_send.cmd_type = GET_CTL_IS_CHARGING;
          qrc_ctl_msg_send.cmd_data.pile_stats = (uint32_t)is_charging;
          charger_qrc_msg_send(&qrc_ctl_msg_send);

          memset(&qrc_ctl_msg_send, 0, sizeof(struct charger_ctl_msg_s));
          qrc_ctl_msg_send.cmd_type = GET_CTL_SM_STATE;
          qrc_ctl_msg_send.cmd_data.sm_state = charger_dev_get_sm_stats();
          charger_qrc_msg_send(&qrc_ctl_msg_send);
          break;

        case START_CTL_CHARGING:

          sm_start_charging();
          break;

        case STOP_CTL_CHARGING:

          sm_stop_charging();
          break;

        default:
          break;
    }
}

/* charger controller qrc msg callback */

static void charger_controller_qrc_msg_cb(struct qrc_pipe_s * pipe, void * data, size_t len, bool response)
{

  struct charger_ctl_msg_s *charger_msg;
  
  if (pipe == NULL || data ==NULL)
  {
    return;
  }

  if (len == sizeof(struct charger_ctl_msg_s))
    {

      charger_msg = (struct charger_ctl_msg_s *)data;
      charger_controller_msg_parse(pipe, charger_msg);
    }
  return;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int32_t charger_controller(int argc, char *argv[])
{
  char pipe_name[] = CHARGER_PIPE;

    g_charger_pipe =  qrc_get_pipe(pipe_name);
    if (g_charger_pipe == NULL)
      {
        config_notify_completed(false);
        return ERROR;
      }
  
    if (!qrc_register_message_cb(g_charger_pipe, charger_controller_qrc_msg_cb))
      {
        syslog(LOG_ERR,"qrc register robot control cb error\n");
        config_notify_completed(false);
        return ERROR;
      }
   if(notification_callback_register(charger_send_qrc_async_msg_cb) != OK)
    {
      syslog(LOG_ERR,"CHARGER CONTROLLER: notification_callback_register cb error\n");
      config_notify_completed(false);
      return ERROR;
    }
   syslog(LOG_DEBUG, "CHARGER CONTROLLER: charger_motion_speed_send_cb(%p)!\n",charger_motion_speed_send_cb);
   if (motion_callback_register(charger_motion_speed_send_cb) != OK)
    {
      syslog(LOG_ERR,"CHARGER CONTROLLER: motion_callback_register cb error\n");
      config_notify_completed(false);
      return ERROR;
    }
    /* notify ok */
    config_notify_completed(true);
    syslog(LOG_DEBUG, "CHARGER CONTROLLER: charger_controller Initialize successfully!\n");

    return OK;
}

