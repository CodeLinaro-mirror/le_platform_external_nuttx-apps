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

//#include <nuttx/timers/capture.h>


#include "main.h"

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

struct rc_data_s
{
	int speed_fd;
	int angle_fd;
	uint8_t angle_duty;
	uint8_t speed_duty;
	float max_speed_x;
	float max_speed_z;
	uint32_t sample_time;  /* sampling interval/ms */
	bool rc_attached;      /* RC mode  */
	bool rc_data_ready;     /* false: data not ready; true: rc_data ready */
	uint32_t timestamp;
};

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
static void rc_filter(uint8_t *speed_duty, uint8_t *angle_duty)
{
        /* filter */
        *speed_duty = AMP_LIMIT(*speed_duty, RC_MIN_VALUE, RC_MAX_VALUE);
        *angle_duty = AMP_LIMIT(*angle_duty, RC_MIN_VALUE, RC_MAX_VALUE);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

// struct remote_contrl_platform remote_controller_manage_platform on HAL define

extern struct remote_contrl_platform rc_controller_manage

int regitster_hotrc_hal(struct remote_contrl_platform_s *hal_control);
{
  hal_control.ops = zl_ops;
  return 0;
}

int init_rc_hal_data();
{
  g_rc_data.angle_fd = open(RC_ANGLE_DEV, O_RDONLY);

  if (g_rc_data.angle_fd < 0)
  {
    syslog(LOG_INFO,"rc_init: open %s failed: %d\n",
      RC_ANGLE_DEV, errno);
	return -1;
  }
  syslog(LOG_INFO,"rc_init: opened  %s fd = %d \n",RC_ANGLE_DEV,g_rc_data.angle_fd);

  g_rc_data.speed_fd = open(RC_SPEED_DEV, O_RDONLY);
  if (g_rc_data.speed_fd < 0)
  {
    syslog(LOG_INFO,"rc_init: open %s failed: %d\n",
      RC_SPEED_DEV, errno);
	return -1;
  }
  syslog(LOG_INFO,"rc_init: opened %s fd = %d\n",RC_SPEED_DEV,g_rc_data.speed_fd);

  g_rc_data.rc_attached = false;
  g_rc_data.rc_data_ready = false;
  g_rc_data.sample_time = RC_SAMPLE_TIME;
  g_rc_data.max_speed_x = MAX_SPEED;
  g_rc_data.max_speed_z = MAX_ANGULAR_VELOCITY;
  return 0;
}

static int get_rc_duty(int fd, uint8_t* duty){
  int ret;
  uint8_t count = 0;
  fflush(stdout);
  /* Get the dutycycle data using the ioctl */
  //uint32_t start = clock_systime_ticks();
  do {
    ret = ioctl(fd, CAPIOC_DUTYCYCLE, (unsigned long)((uintptr_t)duty));
    if (ret < 0){
      syslog(LOG_INFO,"get_rc_duty: ioctl(CAPIOC_DUTYCYCLE) failed: %d, ret = %d \n", fd,ret);
      return ret;
    }
    count ++;
    usleep(5*1000);
    if (count > 30){
      *duty = RC_MIDDLE_VALUE;
      break;
    }
  }while(*duty < RC_MIN_VALUE ||*duty > RC_MAX_VALUE );
  //syslog(LOG_INFO, "rc read delay %dms\n", (clock_systime_ticks() - start)*10);

  return OK;
}

int get_speed(struct speed_req_s *speed)
{
  if (OK == get_rc_duty(g_rc_data.speed_fd, &speed_duty) &&
    OK == get_rc_duty(g_rc_data.angle_fd, &angle_duty))
  {
    rc_filter(&speed_duty, &angle_duty);
    g_rc_data.speed_duty = speed_duty;
    g_rc_data.angle_duty = angle_duty;
    g_rc_data.timestamp = clock_systime_ticks();
    speed->speed_x = 2 * (float)(g_rc_data.speed_duty -RC_MIDDLE_VALUE) /(float)(RC_MAX_VALUE - RC_MIN_VALUE) * g_rc_data.max_speed_x;
    speed->speed_z = -2 * (float)(g_rc_data.angle_duty -RC_MIDDLE_VALUE) /(float)(RC_MAX_VALUE - RC_MIN_VALUE) * g_rc_data.max_speed_z;
	return 0;
  }
  return -ETIME;
}

int set_max_speed(struct speed_req_s *speed)
{
  return 0;
}


