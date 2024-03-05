/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#ifndef __APP_CHARGER_SM_H
#define __APP_CHARGER_SM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <pthread.h>

/********************************************************************************
 * Pre-processor Definitions
 ********************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum chr_sm_st_e
{
  CHR_SM_IDLE = 1,
  CHR_SM_SEARCHING,
  CHR_SM_CONTROLLING,
  CHR_SM_FORCE_CHARGING,
  CHR_SM_CHARGING,
  CHR_SM_CHARGER_DONE,
  CHR_SM_EXCEPTION
};

enum chr_sm_event_e
{
  SM_EVENT_START_CHARGING = 1,
  SM_EVENT_FIND_PILE,
  SM_EVENT_ATTACH_PILE,
  SM_EVENT_TO_NORMAL_CHARGING,
  SM_EVENT_STOP_CHARGING,
  SM_EVENT_BACK_TO_IDLE,
  SM_EVENT_EXCEPTION
};

struct charger_sm_state_s {
  bool sm_task_started;
  pthread_rwlock_t  sm_rw_lock;
  enum chr_sm_st_e  cur_state;
  
}__attribute__((aligned(4)));

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

uint32_t get_curr_state(void);
int32_t start_sm_task(void);

#endif /* __APP_CHARGER_SM_H */
