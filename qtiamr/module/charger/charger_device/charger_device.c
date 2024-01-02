/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#include <stdio.h>
#include <pthread.h>

#include "charger_device.h"
#include "charger_hal.h"


charger_dev_t g_charger_dev;
static charger_dev_t *g_charger_dev_p = &g_charger_dev;



void chr_core_rwlock_init(pthread_rwlock_t *cc_rw_lock){
  
   int status;
   status = pthread_rwlock_init(cc_rw_lock, NULL);
    if (status != 0)
    {
      syslog(LOG_ERR,"CHARGER: sm_init: ERROR pthread_rwlock_init failed, status=%d\n", status);
      ASSERT(false);
    }
}


void chr_core_rwlock_rdlock_acquire(FAR pthread_rwlock_t *cc_rw_lock){
  int status;
  status = pthread_rwlock_rdlock(cc_rw_lock);
  if (status != 0)
  {
      syslog(LOG_ERR,"CHARGER: sm_rwlock_rdlock_acquire: ERROR Failed to open rwlock for reading. Status: %d\n", status);
      ASSERT(false);
    }
}

void chr_core_rwlock_wrlock_acquire(FAR pthread_rwlock_t *cc_rw_lock){
  int status;
  status = pthread_rwlock_wrlock(cc_rw_lock);
  if (status != 0)
  {
      syslog(LOG_ERR,"CHARGER: sm_rwlock_wrlock_acquire: ERROR Failed to lock for writing\n");
      ASSERT(false);
    }
}



void chr_core_rwlock_release(FAR pthread_rwlock_t *rw_lock){
  int status;
  status = pthread_rwlock_unlock(rw_lock);
  if (status != 0)
  {
      syslog(LOG_ERR,"CHARGER: sm_rwlock_release: ERROR Failed to unlock lock held for writing\n");
      ASSERT(false);
    }
}


void chr_core_mutex_init(pthread_mutex_t *cc_mutex){
  int status;
    status = pthread_mutex_init(cc_mutex, NULL);
   if (status != 0)
     {
       syslog(LOG_ERR,"CHARGER: chr_core_mutex_init: ERROR pthread_mutex_init failed, status=%d\n", status);
       ASSERT(false);
     }
}


void chr_core_mutex_lock(pthread_mutex_t *cc_mutex){
   int status;
    status = pthread_mutex_lock(cc_mutex);
       if (status != 0)
         {
           syslog(LOG_ERR,"CHARGER: chr_core_mutex_lock: ERROR pthread_mutex_lock failed, status=%d\n", status);
           ASSERT(false);
         }
}


void chr_core_mutex_unlock(pthread_mutex_t *cc_mutex){
   int status;
    status = pthread_mutex_unlock(cc_mutex);
       if (status != 0)
         {
           syslog(LOG_ERR,"CHARGER: chr_core_mutex_unlock: ERROR pthread_mutex_unlock failed, status=%d\n", status);
           ASSERT(false);
         }
}



float chr_core_get_voltage(){


}


float chr_core_get_charging_curr(){

}


bool chr_core_get_pile_stats(){

}


bool chr_core_get_if_charging_stats(){

}


void chr_core_get_wheel_speed(float * vx, float * vz){



}


uint32_t chr_core_get_sm_stats(){
	

}


void chr_core_drv_ops_register(const char *name,const chr_drv_ops_t *ops){


}


void chr_core_notify_ops_register(const chr_drv_ops_t *ops){


}


void battery_voltage_notify(float voltage){


}




