/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <sched.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <syslog.h>
#include <errno.h>
#include <nuttx/mutex.h>
#include <pthread.h>

#include "hotrc.h"
#include "rc_management.h"


#define ABS_LIMIT(_val_, _abs_)  \
        ((_val_) < (-_abs_) ?  (-_abs_) : \
        ((_val_) > (_abs_) ? (_abs_) : (_val_)))
//#define RC_MGR_DEBUG /*for debug*/

struct remote_contrl_platform_s
{
  char hw_name[8];
  char enable_state;
  struct rc_parameter_s init_setting;
  mutex_t lock;
  struct rc_hal_ops_s *ops;
  bool init_done;
  bool rc_need_update;
  pthread_mutex_t mutex;
  pthread_cond_t  cond;
};

static struct remote_contrl_platform_s g_rc_mgr =
{
  .init_setting = {2,2,true},
  .init_done = false,
  .rc_need_update = false,
};

static struct rc_hal_ops_s * const hal_ops_g[1] =
{
  [0] = &zl_ops,
};

static struct rc_management_cb_s g_root_cb;
static mutex_t g_cb_list_lock;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int wait_condtion(void)
{
  int status;

  /*wait enable condtion*/

  status = pthread_mutex_lock(&g_rc_mgr.mutex);
  if (status != 0)
  {
    return ERROR;
  }
#ifdef RC_MGR_DEBUG
  syslog(LOG_INFO,"rc_debug %s : rc_mgr_thread start waitting.\n", __func__);
#endif
  status = pthread_cond_wait(&g_rc_mgr.cond, &g_rc_mgr.mutex);
  if (status != 0)
  {
    return ERROR;
  }
#ifdef RC_MGR_DEBUG
  syslog(LOG_INFO,"rc_debug %s : rc_mgr_thread go ahead.\n", __func__);
#endif
  status = pthread_mutex_unlock(&g_rc_mgr.mutex);
  if (status != 0)
  {
    return ERROR;
  }

  return OK;
}

static int rc_manag_hal_init(void)
{
  int status = OK;
  if (hal_ops_g[0] == NULL)
  {
    syslog(LOG_INFO,"%s : set hal_ops is null", __func__);
    return ERROR;
  }
  g_rc_mgr.ops = hal_ops_g[0];

  if (g_rc_mgr.ops->init !=NULL)
  {
    status = g_rc_mgr.ops->init();
  }

  return status;
}

static int notify_client_cb(enum rc_action_e action, void *data)
{
  int status = OK;
  struct rc_management_cb_s *next;

  for (next = g_root_cb.list_nlink; next != &g_root_cb ; next = next->list_nlink)
  {
    /*todo*/
    if (next->rc_client_callback == NULL)
    {
      continue;
    }
    status = next->rc_client_callback(action,data);
    if (status != OK)
    {
      syslog(LOG_INFO,"rc_mgr callback :%pf fail \n",next->rc_client_callback);
    }
#ifdef RC_MGR_DEBUG
    syslog(LOG_INFO,"rc_mgr callback :%pf success \n",next->rc_client_callback);
#endif
  }

  return OK;
}

static int rc_manag_basic_init(void)
{
  int status;

  nxmutex_init(&g_rc_mgr.lock);

  g_root_cb.list_nlink = &g_root_cb;
  g_root_cb.list_prelink = &g_root_cb;
  g_root_cb.rc_client_callback = NULL;
  nxmutex_init(&g_cb_list_lock);

  status = pthread_mutex_init(&g_rc_mgr.mutex, NULL);
  if (status != 0)
  {
    return ERROR;
  }
  status = pthread_cond_init(&g_rc_mgr.cond, NULL);
  if (status != 0)
  {
    return ERROR;
  }
  g_rc_mgr.init_done = true;

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int register_rc_mgr_callback(struct rc_management_cb_s *cb)
{
  if ((cb->rc_client_callback == NULL)||(g_rc_mgr.init_done == false))
  {
    return ERROR;
  }
  nxmutex_lock(&g_cb_list_lock);
  cb->list_nlink = g_root_cb.list_nlink;
  g_root_cb.list_nlink = cb;
  cb->list_prelink = &g_root_cb;
  nxmutex_unlock(&g_cb_list_lock);
  return OK;
}

int get_vx_vz_speed_from_hal(struct speed_req_s *speed)
{
  int status = 0;

  if (g_rc_mgr.ops == NULL)
  {
    return ERROR;
  }
  status = g_rc_mgr.ops->get_vx_vz_speed(speed);
  if (status != OK)
  {
    syslog(LOG_INFO,"%s : get speed fail", __func__);
    return ERROR;
  }

  return status;
}

int rc_manag_enable(void)
{
  int status = OK;

  status = pthread_mutex_lock(&g_rc_mgr.mutex);
  if (status != 0)
  {
    return ERROR;
  }
  g_rc_mgr.rc_need_update = true;
  status = pthread_cond_signal(&g_rc_mgr.cond);
#ifdef RC_MGR_DEBUG
  syslog(LOG_INFO,"rc_debug : set rc_need_update: true.\n");
#endif
  if (status != 0)
  {
    return ERROR;
  }
  status = pthread_mutex_unlock(&g_rc_mgr.mutex);
  if (status != 0)
  {
    return ERROR;
  }

  return status;
}

int rc_manag_disable(void)
{
  int status = OK;

  status = pthread_mutex_lock(&g_rc_mgr.mutex);
  if (status != 0)
  {
    return ERROR;
  }
  g_rc_mgr.rc_need_update = false;
#ifdef RC_MGR_DEBUG
  syslog(LOG_INFO,"rc_debug : set rc_need_update: false.\n");
#endif
  status = pthread_mutex_unlock(&g_rc_mgr.mutex);
  if (status != 0)
  {
    return ERROR;
  }
  return OK;
}

int set_rc_manage_init_setting(struct rc_parameter_s data)
{
  int status = OK;
  g_rc_mgr.init_setting = data;

  if (g_rc_mgr.ops->set_max_speed !=NULL)
  {
    status = g_rc_mgr.ops->set_max_speed(data.x_speed,
		    data.z_speed);
  }
  return status;
}

int rc_management_task(int argc, char *argv[])
{
  int status = 0;
  struct speed_req_s speed_req;
  /*chose hal dirver*/

  status = rc_manag_hal_init();
  if (status != 0)
  {
    syslog(LOG_INFO,"%s: rc_mgr_hal_init error.\n",__func__);
    return ERROR;
  }
  status = rc_manag_basic_init();
  if (status != 0)
  {
    syslog(LOG_INFO,"%s: rc_mgr_init error.\n",__func__);
    return ERROR;
  }

  while(1)
  {
    /*wait enable condtion*/

    status = wait_condtion();
    if (status != 0)
    {
      syslog(LOG_INFO,"%s: wait_condition error.\n",__func__);
      return ERROR;
    }
    if (!g_rc_mgr.init_setting.enable_rc_management)
    {
      /*disable RC module*/
      syslog(LOG_INFO,"%s: disable rc mode, thread exit.\n",__func__);
      if (g_rc_mgr.ops->release !=NULL)
      {
        status = g_rc_mgr.ops->release();
	if (status !=OK)
	{
	  syslog(LOG_INFO,"%s: release hal fail, thread exit.\n",__func__);
	}
      }
      syslog(LOG_INFO,"%s: disable rc mode, thread exit.\n",__func__);

      break;
    }
    while(1)
    {
      if (!g_rc_mgr.rc_need_update)
      {
        break;
      }
      /*todo get speed*/
      speed_req.x_speed = 0;
      speed_req.z_speed = 0;
      status = get_vx_vz_speed_from_hal(&speed_req);
      if (status != OK)
      {
        speed_req.x_speed = 0;
        speed_req.z_speed = 0;
        syslog(LOG_INFO,"%s:get speed fail, clear vx/vz \n",__func__);
      }else
      {
        speed_req.x_speed = ABS_LIMIT(speed_req.x_speed,g_rc_mgr.init_setting.x_speed);
        speed_req.z_speed = ABS_LIMIT(speed_req.z_speed,g_rc_mgr.init_setting.z_speed);
      }
#ifdef RC_MGR_DEBUG
      syslog(LOG_INFO,"rc_debug %s :vx: %f , vz: %f \n",__func__, speed_req.x_speed, speed_req.z_speed);
#endif
      /*notify speed*/
      notify_client_cb(RC_UPDATE_SPEED, &speed_req);
      usleep(30*1000);
    }
  }
  return OK;
}
