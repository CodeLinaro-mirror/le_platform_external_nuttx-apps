/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
//#include <nuttx/config.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <sched.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <syslog.h>

#include "rc_controller.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define  RC_ANGLE_DEV "/dev/capture1"
#define  RC_SPEED_DEV "/dev/capture0"

#define RC_MIDDLE_VALUE     (75)   /* rc default value is 75  */
#define RC_HAVE_VALUE       (80)   /* check rc if have data */
#define RC_MAX_VALUE        (92)
#define RC_MIN_VALUE        (57)


#define RC_SAMPLE_TIME				(10)  /* ms */
#define MAX_SPEED					(2.0f)   /* actual is 1.82 m/s */
#define MAX_ANGULAR_VELOCITY		2

#define AMP_LIMIT(_val_, _min_, _max_)  \
        ((_val_) < (_min_) ?  (_min_) : \
        ((_val_) > (_max_) ? (_max_) : (_val_)))

#endif


/****************************************************************************
 * Private Types
 ****************************************************************************/

static struct rc_data_s g_rc_data;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct remote_contrl_ops_s zl_ops
{
  .init_fucnt = init_rc_hal_data;
  .get_speed = get_speed;
  .set_speed = set_max_speed;
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/
 
 

int rc_management_callback()
{
  return 0;
}

//void (*sm_notify_cb)( enum mcb_sm_e state);

void rc_sm_callback( enum mcb_sm_e state)
{
  switch (cb_sm_e)
  case REMOTE_CONTROL
    if (urr_state != REMOTE_CONTROL)
	{
	  /*enable rc data update*/
	}
    break;
  default:
    if (urr_state != REMOTE_CONTROL)
	{
	  /*disable rc data update*/
	}
    break;   

}

//extern bool mcb_sm_register_state_notify_cb(sm_notify_cb cb_fun);

static int register_rc_callback()
{
  /*regitster MCB state callback */
  
  if (!mcb_sm_register_state_notify_cb(rc_sm_callback))
  {
    /*register fail*/
    return -1;
  }
  
  /*register callback for RC management*/
  //TODO
  
  return 0;
}




/****************************************************************************
 * Public Functions
 ****************************************************************************/

// struct remote_contrl_platform remote_controller_manage_platform on HAL define

int rc_controller_task(int argc, char *argv[]);
{
  
  if (register_rc_callback())
  {
    /*register fail*/
  }
  
  
  return 0;
}

