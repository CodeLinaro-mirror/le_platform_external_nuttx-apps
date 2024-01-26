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
#include <nuttx/timers/capture.h>

#include "hotrc.h"
#include "main.h"
#include "rc_management.h"


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
//#define  RC_HAL_DEBUG		/*for debug*/

#define  RC_ANGLE_DEV "/dev/capture1"
#define  RC_SPEED_DEV "/dev/capture0"

#define RC_MIDDLE_VALUE     (75)   /* rc default value is 75  */
#define RC_HAVE_VALUE       (80)   /* check rc if have data */
#define RC_MAX_VALUE        (92)
#define RC_MIN_VALUE        (57)


#define RC_SAMPLE_TIME				(10)  /* ms */
#define MAX_SPEED					(1.0f)   /* actual is 1.82 m/s */
#define MAX_ANGULAR_VELOCITY		2

#define AMP_LIMIT(_val_, _min_, _max_)  \
        ((_val_) < (_min_) ?  (_min_) : \
        ((_val_) > (_max_) ? (_max_) : (_val_)))



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

static int init_rc_hal_data(void);
static int get_speed(struct speed_req_s *speed);
static int set_max_speed(float x_speed, float z_speed);
static int release_rc_hal_data(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct rc_hal_ops_s zl_ops =
{
  .init = init_rc_hal_data,
  .release = release_rc_hal_data,
  .get_vx_vz_speed = get_speed,
  .set_max_speed = set_max_speed,
};

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

int init_rc_hal_data(void)
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

int release_rc_hal_data(void)
{
  close(g_rc_data.angle_fd);
  g_rc_data.angle_fd = -1;
  close(g_rc_data.speed_fd);
  g_rc_data.speed_fd = -1;
  g_rc_data.max_speed_x = MAX_SPEED;
  g_rc_data.max_speed_z = MAX_ANGULAR_VELOCITY;

  return OK;
}
static int get_rc_duty(int fd, uint8_t* duty){
  int ret;
  uint8_t count = 0;
  fflush(stdout);
  /* Get the dutycycle data using the ioctl */
  //uint32_t start = clock_systime_ticks();
  if (fd < 0)
  {
    return ERROR;
  }
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
#ifdef RC_HAL_DEBUG
    syslog(LOG_INFO, "read from duty fd:%d duty:%d,count: %d \n", fd, *duty, count);
#endif

  return OK;
}

int get_speed(struct speed_req_s *speed)
{
  uint8_t speed_duty = 0;
  uint8_t angle_duty = 0;

  if (OK == get_rc_duty(g_rc_data.speed_fd, &speed_duty) &&
    OK == get_rc_duty(g_rc_data.angle_fd, &angle_duty))
  {
#ifdef RC_HAL_DEBUG
    syslog(LOG_INFO, "rc_hal duty vx:%d vz:%d \n", speed_duty, angle_duty);
#endif
    rc_filter(&speed_duty, &angle_duty);
    g_rc_data.speed_duty = speed_duty;
    g_rc_data.angle_duty = angle_duty;
#ifdef RC_HAL_DEBUG
    syslog(LOG_INFO, "rc_hal rc_filter duty vx:%d vz:%d \n", speed_duty, angle_duty);
#endif
    g_rc_data.timestamp = clock_systime_ticks();
    speed->x_speed = 2 * (float)(g_rc_data.speed_duty -RC_MIDDLE_VALUE) /(float)(RC_MAX_VALUE - RC_MIN_VALUE) * g_rc_data.max_speed_x;
    speed->z_speed = -2 * (float)(g_rc_data.angle_duty -RC_MIDDLE_VALUE) /(float)(RC_MAX_VALUE - RC_MIN_VALUE) * g_rc_data.max_speed_z;
#ifdef RC_HAL_DEBUG
    syslog(LOG_INFO, "rc_hal speed  vx:%f vz:%f \n", speed->x_speed, speed->z_speed);
#endif
	return 0;
  }
  return -ETIME;
}

int set_max_speed(float x_speed, float z_speed)
{
  g_rc_data.max_speed_x = x_speed;
  g_rc_data.max_speed_z = z_speed;
  return 0;
}
