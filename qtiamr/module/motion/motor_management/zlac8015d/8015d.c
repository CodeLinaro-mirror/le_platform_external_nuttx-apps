/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include "motor_management.h"
#include "canopen.h"
#include "motion_management.h"
#include "8015d.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MOTOR_DRIVER_DEV "/dev/can0"

/* motor config parameter */

#define POS_ANGLE_SPEED_MAX 11
#define POS_DIST_SPEED_MAX  60
#define MOTOR_BUS_MODE      BUS_CANOPEN

#define SPEED_KP (400)
#define SPEED_KI (200)

#define CURRENT_KP (800)
#define CURRENT_KI (300)

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum zlac_8015d_control_code_e
{
  CODE_DEBUG_CC = 0,
  CODE_CLEAN_ERROR,
  CODE_MOTOR_STATUS,
  CODE_CAN_ASYNC,
  CODE_CAN_SYNC,
  CODE_CAN_ENABLE_1,
  CODE_CAN_ENABLE_2,
  CODE_CAN_ENABLE_3,
  CODE_QUICK_STOP,
  CODE_SPEED_MODE,
  CODE_POSITION_MODE,
  CODE_SET_POS_LEFT,
  CODE_SET_POS_RIGHT,
  CODE_SET_POS_REL_START_PREPARE,
  CODE_SET_POS_REL_START_RUN,
  CODE_SET_SPEED_LEFT,
  CODE_SET_SPEED_RIGHT,
  CODE_SET_SPEED_LEFT_ACC,
  CODE_SET_SPEED_RIGHT_ACC,
  CODE_SET_SPEED_LEFT_DEC,
  CODE_SET_SPEED_RIGHT_DEC,
  CODE_SYNC_SPEED,
  CODE_SPEED_READ,
  CODE_LEFT_COUNT,
  CODE_RIGHT_COUNT,
  CODE_SPEED_LEFT_KP,
  CODE_SPEED_RIGHT_KP,
  CODE_SPEED_LEFT_KI,
  CODE_SPEED_RIGHT_KI,
  CODE_LEFT_POLE,
  CODE_RIGHT_POLE,
  CODE_LEFT_ENCODER_RANGE,
  CODE_RIGHT_ENCODER_RANGE,
  CODE_DRIVER_VERSION,
  CODE_CAN_RESET,
  CODE_POS_MAX_SPEED_LEFT,
  CODE_POS_MAX_SPEED_RIGHT,
  CODE_POS_ANGLE_MAX_SPEED_LEFT,
  CODE_POS_ANGLE_MAX_SPEED_RIGHT,
  CODE_SET_CURRENT_KP_LEFT,
  CODE_SET_CURRENT_KP_RIGHT,
  CODE_SET_CURRENT_KI_LEFT,
  CODE_SET_CURRENT_KI_RIGHT,
};

struct driver_sdo_data_s
{
  uint8_t sdo_cmd;
  uint8_t index_l;
  uint8_t index_h;
  uint8_t sub_index;
  uint8_t data1_l;
  uint8_t data1_h;
  uint8_t data2_l;
  uint8_t data2_h;
};

struct mc_cmd_s
{
  enum zlac_8015d_control_code_e cmd;
  struct driver_sdo_data_s       command;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int zlac_8015d_write_single_opcode(struct motor_zlac_8015d_s *    motor,
                                          enum zlac_8015d_control_code_e opcode,
                                          int                            value);

static bool zlac_8015d_init(void *motor);
static void zlac_8015d_deinit(void *motor);

static int zlac_8015d_switch_mode(void *motor, enum control_mode_e mode);
static int zlac_8015d_set_pid(void *              motor, enum control_mode_e,
                              struct motion_pid_s pid);
static int zlac_8015d_sync_speed(void *motor, int16_t left_rpm,
                                 int16_t right_rpm);
static int zlac_8015d_set_position(void *motor, int left_count,
                                   int right_count);
static int zlac_8015d_quick_stop(void *motor);

static int zlac_8015d_read_rpm(void *motor, float *left_rpm, float *right_rpm);
static int zlac_8015d_velocity_zero_check(void *motor, bool *zero);

static int  zlac_8015d_pos_count(void *motor, int *left_count,
                                 int *right_count);
static int  zlac_8015d_status_code(void *motor, enum motor_err_e *motor_status);
static bool zlac_8015d_target_reached_check(void *motor);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* 读写数据的数据帧 */
const struct mc_cmd_s g_command_list[] = {
  { CODE_DEBUG_CC, { 0x43, 0x3f, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00 } },
  { CODE_CLEAN_ERROR, { 0x2b, 0x40, 0x60, 0x00, 0x80, 0x00, 0x00, 0x00 } },
  { CODE_MOTOR_STATUS, { 0x43, 0x41, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00 } },
  { CODE_CAN_ASYNC, { 0x2b, 0x0f, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00 } },
  { CODE_CAN_SYNC, { 0x2b, 0x0f, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00 } },
  { CODE_CAN_ENABLE_1, { 0x2b, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00 } },
  { CODE_CAN_ENABLE_2, { 0x2b, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00 } },
  { CODE_CAN_ENABLE_3, { 0x2b, 0x40, 0x60, 0x00, 0x0f, 0x00, 0x00, 0x00 } },
  { CODE_QUICK_STOP, { 0x2b, 0x40, 0x60, 0x00, 0x0f, 0x01, 0x00, 0x00 } },
  { CODE_SPEED_MODE, { 0x2f, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00 } },
  { CODE_POSITION_MODE, { 0x2f, 0x60, 0x60, 0x00, 0x01, 0x00, 0x00, 0x00 } },
  { CODE_SET_POS_LEFT, { 0x23, 0x7a, 0x60, 0x01, 0x00, 0x00, 0x00, 0x00 } },
  { CODE_SET_POS_RIGHT, { 0x23, 0x7a, 0x60, 0x02, 0x00, 0x00, 0x00, 0x00 } },
  { CODE_SET_POS_REL_START_PREPARE, { 0x2b, 0x40, 0x60, 0x00, 0x4f, 0x00, 0x00, 0x00 } },
  { CODE_SET_POS_REL_START_RUN, { 0x2b, 0x40, 0x60, 0x00, 0x5f, 0x00, 0x00, 0x00 } },
  { CODE_SET_SPEED_LEFT, { 0x23, 0xFF, 0x60, 0x01, 0x64, 0x00, 0x00, 0x00 } },
  { CODE_SET_SPEED_RIGHT, { 0x23, 0xFF, 0x60, 0x02, 0x64, 0x00, 0x00, 0x00 } },
  { CODE_SET_SPEED_LEFT_ACC, { 0x23, 0x83, 0x60, 0x01, 0x05, 0x00, 0x00, 0x00 } },
  { CODE_SET_SPEED_RIGHT_ACC, { 0x23, 0x83, 0x60, 0x02, 0x05, 0x00, 0x00, 0x00 } },
  { CODE_SET_SPEED_LEFT_DEC, { 0x23, 0x84, 0x60, 0x01, 0x05, 0x00, 0x00, 0x00 } },
  { CODE_SET_SPEED_RIGHT_DEC, { 0x23, 0x84, 0x60, 0x02, 0x05, 0x00, 0x00, 0x00 } },
  { CODE_SYNC_SPEED, { 0x23, 0xFF, 0x60, 0x03, 0x00, 0x00, 0x00, 0x00 } },
  { CODE_SPEED_READ, { 0x43, 0x6C, 0x60, 0x03, 0x00, 0x00, 0x00, 0x00 } },
  { CODE_LEFT_COUNT, { 0x43, 0x64, 0x60, 0x01, 0x00, 0x00, 0x00, 0x00 } },
  { CODE_RIGHT_COUNT, { 0x43, 0x64, 0x60, 0x02, 0x00, 0x00, 0x00, 0x00 } },
  { CODE_SPEED_LEFT_KP, { 0x2b, 0x1d, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00 } },
  { CODE_SPEED_RIGHT_KP, { 0x2b, 0x1d, 0x20, 0x02, 0x00, 0x00, 0x00, 0x00 } },
  { CODE_SPEED_LEFT_KI, { 0x2b, 0x1e, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00 } },
  { CODE_SPEED_RIGHT_KI, { 0x2b, 0x1e, 0x20, 0x02, 0x00, 0x00, 0x00, 0x00 } },
  { CODE_LEFT_POLE, { 0x2b, 0x0c, 0x20, 0x01, 0x0A, 0x00, 0x00, 0x00 } },
  { CODE_RIGHT_POLE, { 0x2b, 0x0c, 0x20, 0x02, 0x0A, 0x00, 0x00, 0x00 } },
  { CODE_LEFT_ENCODER_RANGE, { 0x2b, 0x0E, 0x20, 0x01, 0x00, 0x01, 0x00, 0x00 } },
  { CODE_RIGHT_ENCODER_RANGE, { 0x2b, 0x0E, 0x20, 0x02, 0x00, 0x01, 0x00, 0x00 } },
  { CODE_DRIVER_VERSION, { 0x4b, 0x31, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00 } },
  { CODE_CAN_RESET, { 0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
  { CODE_POS_MAX_SPEED_LEFT, { 0x23, 0x81, 0x60, 0x01, 0x3C, 0x00, 0x00, 0x00 } },
  { CODE_POS_MAX_SPEED_RIGHT, { 0x23, 0x81, 0x60, 0x02, 0x3C, 0x00, 0x00, 0x00 } },
  { CODE_POS_ANGLE_MAX_SPEED_LEFT, { 0x23, 0x81, 0x60, 0x01, 0x0B, 0x00, 0x00, 0x00 } },
  { CODE_POS_ANGLE_MAX_SPEED_RIGHT, { 0x23, 0x81, 0x60, 0x02, 0x0B, 0x00, 0x00, 0x00 } },
  { CODE_SET_CURRENT_KP_LEFT, { 0x23, 0x19, 0x20, 0x01, 0x58, 0x02, 0x00, 0x00 } },
  { CODE_SET_CURRENT_KP_RIGHT, { 0x23, 0x19, 0x20, 0x02, 0x58, 0x02, 0x00, 0x00 } },
  { CODE_SET_CURRENT_KI_LEFT, { 0x23, 0x1A, 0x20, 0x01, 0x2C, 0x01, 0x00, 0x00 } },
  { CODE_SET_CURRENT_KI_RIGHT, { 0x23, 0x1A, 0x20, 0x02, 0x2C, 0x01, 0x00, 0x00 } },
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct motor_zlac_8015d_s g_zlac_8015d = {
  .initialized = false,
};

struct motor_hal_ops zlac_8015d_ops = {
  .motor_hal_init        = zlac_8015d_init,
  .motor_hal_deinit      = zlac_8015d_deinit,
  .switch_mode           = zlac_8015d_switch_mode,
  .set_pid               = zlac_8015d_set_pid,
  .set_speed             = zlac_8015d_sync_speed,
  .set_position          = zlac_8015d_set_position,
  .quick_stop            = zlac_8015d_quick_stop,
  .get_rpm               = zlac_8015d_read_rpm,
  .get_count             = zlac_8015d_pos_count,
  .get_motor_status_code = zlac_8015d_status_code,
  .check_pose_reach      = zlac_8015d_target_reached_check,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int zlac_8015d_write_single_opcode(struct motor_zlac_8015d_s *    motor,
                                          enum zlac_8015d_control_code_e opcode,
                                          int                            value)
{
  struct driver_sdo_data_s data;
  struct driver_sdo_data_s rec_buff;
  int                      fd       = motor->driver_fd;
  size_t                   data_len = sizeof(struct driver_sdo_data_s);
  int                      status;

  if (fd < 0)
    {
      syslog(LOG_ERR, "zlac_8015d file desc invalid %d \n", fd);
      return ERROR;
    }

  data = g_command_list[opcode].command;

  if (value != 0)
    {
      data.data1_l = value && 0xff;
      data.data1_h = (value >> 8) && 0xff;
    }

  status = pthread_mutex_lock(&motor->motor_mutex);
  if (status != 0)
    {
      printf("zlac_8015d_write_single_opcode: ERROR pthread_mutex_lock failed, status=%d\n",
             status);
      ASSERT(false);
    }

  canopen_send(fd, (char *)&data, data_len);
  usleep(2000);
  canopen_receive(fd, (char *)&rec_buff, data_len);

  status = pthread_mutex_unlock(&motor->motor_mutex);
  if (status != 0)
    {
      printf("zlac_8015d_write_single_opcode: ERROR pthread_mutex_unlock failed, status=%d\n",
             status);
      ASSERT(false);
    }

  /* check the opcode if matched */
  if (rec_buff.index_l == data.index_l && rec_buff.index_h == data.index_h)
    {
      return OK;
    }
  else
    {
      return ERROR;
    }
}

static int zlac_8015d_read_single_opcode(struct motor_zlac_8015d_s *    motor,
                                         enum zlac_8015d_control_code_e opcode,
                                         int16_t *low, int16_t *high)
{
  struct driver_sdo_data_s data;
  struct driver_sdo_data_s rec_buff;
  int                      fd       = motor->driver_fd;
  size_t                   data_len = sizeof(struct driver_sdo_data_s);
  int                      status;

  if (fd < 0)
    {
      syslog(LOG_ERR, "zlac_8015d file desc invalid %d \n", fd);
      return ERROR;
    }

  data = g_command_list[opcode].command;

  status = pthread_mutex_lock(&motor->motor_mutex);
  if (status != 0)
    {
      printf("zlac_8015d_read_single_opcode: ERROR pthread_mutex_lock failed, status=%d\n",
             status);
      ASSERT(false);
    }
  usleep(2000);
  canopen_send(fd, (char *)&data, data_len);
  usleep(2000);
  canopen_receive(fd, (char *)&rec_buff, data_len);

  status = pthread_mutex_unlock(&motor->motor_mutex);
  if (status != 0)
    {
      printf("zlac_8015d_read_single_opcode: ERROR pthread_mutex_unlock failed, status=%d\n",
             status);
      ASSERT(false);
    }

  if (rec_buff.index_l == data.index_l && rec_buff.index_h == data.index_h)
    {
      *low  = rec_buff.data1_h << 8 | rec_buff.data1_l;
      *high = rec_buff.data2_h << 8 | rec_buff.data2_l;
      return OK;
    }
  else
    {
      syslog(LOG_ERR, "zlac_8015d read dump:data:%d,%d,%d,%d,%d,%d,%d,%d\n", rec_buff.sdo_cmd, rec_buff.index_l, rec_buff.index_h, rec_buff.sub_index, rec_buff.data1_l, rec_buff.data1_h, rec_buff.data2_l, rec_buff.data2_h);
      syslog(LOG_ERR, "zlac_8015d read opcode check error \n");
      return ERROR;
    }
}

static int zlac_8015d_set_motor_pole(struct motor_zlac_8015d_s *motor)
{
  int result;
  /* Init poloe */
  result = zlac_8015d_write_single_opcode(motor, CODE_LEFT_POLE, 0);
  usleep(2000);
  syslog(LOG_INFO, "zlac_8015d_enable  CODE_LEFT_POLE =%d\n", result);

  result |= zlac_8015d_write_single_opcode(motor, CODE_RIGHT_POLE, 0);
  syslog(LOG_INFO, "zlac_8015d_enable  CODE_RIGHT_POLE =%d\n", result);

  return result;
}

static int zlac_8015d_enable_can(struct motor_zlac_8015d_s *motor)
{
  int result = 0;

  syslog(LOG_INFO, "ENABLE zlac_8015d motor CAN BUS\n");

  result |= zlac_8015d_write_single_opcode(motor, CODE_CAN_ENABLE_1, 0);
  usleep(2000);
  syslog(LOG_INFO, "zlac_8015d_enable  CODE_CAN_ENABLE_1 =%d\n", result);

  result |= zlac_8015d_write_single_opcode(motor, CODE_CAN_ENABLE_2, 0);
  usleep(2000);
  syslog(LOG_INFO, "zlac_8015d_enable  CODE_CAN_ENABLE_2 =%d\n", result);

  result |= zlac_8015d_write_single_opcode(motor, CODE_CAN_ENABLE_3, 0);

  syslog(LOG_INFO, "zlac_8015d_enable  CODE_CAN_ENABLE_3 =%d\n", result);

  return result;
}

static int zlac_8015d_speed_config_acc(struct motor_zlac_8015d_s *motor)
{
  int result;
  /* Set acc & dec time */
  result = zlac_8015d_write_single_opcode(motor, CODE_SET_SPEED_LEFT_ACC, 0);
  usleep(2000);
  result |= zlac_8015d_write_single_opcode(motor, CODE_SET_SPEED_RIGHT_ACC, 0);
  usleep(2000);
  result |= zlac_8015d_write_single_opcode(motor, CODE_SET_SPEED_LEFT_DEC, 0);
  usleep(2000);
  result |= zlac_8015d_write_single_opcode(motor, CODE_SET_SPEED_RIGHT_DEC, 0);
  return result;
}

static int zlac_8015d_speed_mode_init(struct motor_zlac_8015d_s *motor)
{
  int result;

  syslog(LOG_INFO, "ENABLE zlac_8015d speed mode\n");

  /* Set speed mode */
  result = zlac_8015d_write_single_opcode(motor, CODE_SPEED_MODE, 0);

  return result;
}

/****************************************************************************
 * ops function
 ****************************************************************************/

static int zlac_8015d_switch_mode(void *motor, enum control_mode_e mode)
{
  bool speed_zero;
  /* check speed */
  zlac_8015d_velocity_zero_check(motor, &speed_zero);
  if (speed_zero)
    {
      /* do_switch_mode */
    }

  return ERROR;
}

static int zlac_8015d_set_position(void *motor, int left_count, int right_count)
{
  return ERROR;
}

static int zlac_8015d_pos_count(void *motor, int *left_count, int *right_count)
{
  return ERROR;
}

static int zlac_8015d_velocity_zero_check(void *motor, bool *zero)
{
  struct motor_zlac_8015d_s *zlac_8015d = (struct motor_zlac_8015d_s *)motor;

  if (!zlac_8015d->initialized)
    {
      syslog(LOG_ERR, "zlac_8015d uninitialized\n");
      return ERROR;
    }
  /* zero check  */
  *zero = false;
  syslog(LOG_ERR, "zlac_8015d zero check failed \n");
  return ERROR;
}

static int zlac_8015d_quick_stop(void *motor)
{
  struct motor_zlac_8015d_s *zlac_8015d = (struct motor_zlac_8015d_s *)motor;
  int                        result;

  if (!zlac_8015d->initialized)
    {
      syslog(LOG_ERR, "zlac_8015d uninitialized\n");
      return ERROR;
    }

  zlac_8015d->initialized = false;
  syslog(LOG_DEBUG, "motor quick stop\n");
  result = zlac_8015d_write_single_opcode(zlac_8015d, CODE_QUICK_STOP, 0);

  return result;
}

static int zlac_8015d_read_rpm(void *motor, float *left_rpm, float *right_rpm)
{
  struct motor_zlac_8015d_s *zlac_8015d = (struct motor_zlac_8015d_s *)motor;
  int16_t                    right;
  int16_t                    left;
  int                        result;
  static int                 count = 0;

  count ++;
  result = zlac_8015d_read_single_opcode(zlac_8015d, CODE_SPEED_READ, &left,
                                        &right);
  if (result == OK)
    {
      *left_rpm  = left * 0.1;
      *right_rpm = right * 0.1;
    }

  if (count %200 == 0)
    {
      syslog(LOG_DEBUG, "get odom rpm \t%f\t%f\n",*left_rpm,*right_rpm);
    }

  return result;
}

static int zlac_8015d_status_code(void *motor, enum motor_err_e *motor_status)
{
  struct motor_zlac_8015d_s *zlac_8015d = (struct motor_zlac_8015d_s *)motor;
  int16_t                    right;
  int16_t                    left;
  int                        status_code;
  int                        result;

  result      = zlac_8015d_read_single_opcode(zlac_8015d, CODE_DEBUG_CC, &left, &right);
  status_code = right | left;

  if (result == OK)
    {
      switch (status_code)
        {
            case 0x0000: {
              *motor_status = NORMAL;
              break;
            }
            case 0x0002: {
              *motor_status = LACK_POWER;
              break;
            }
          case 0x0001:
            case 0x0004: {
              *motor_status = OVER_POWER;
              break;
            }
            case 0x0008: {
              *motor_status = OVER_LOAD;
              break;
            }
            case 0x0100: {
              *motor_status = EEPROM_ERR;
              break;
            }
          case 0x0020:
            case 0x0200: {
              *motor_status = ENCODER_ERR;
              break;
            }
            default: {
              *motor_status = OTHER_ERR;
              syslog(LOG_ERR, "zlac_8015d status code left=%d,right=%d\n", left, right);
              break;
            }
        }
    }

  return result;
}

static bool zlac_8015d_target_reached_check(void *motor)
{
  return false;
}

static int zlac_8015d_sync_speed(void *motor, int16_t left_rpm, int16_t right_rpm)
{
  struct motor_zlac_8015d_s *zlac_8015d = (struct motor_zlac_8015d_s *)motor;
  struct driver_sdo_data_s   data;
  size_t                     data_len = (sizeof(struct driver_sdo_data_s));
  struct driver_sdo_data_s   rec_buff;
  int                        fd = zlac_8015d->driver_fd;
  int                        status;

  if (!zlac_8015d->initialized)
    {
      syslog(LOG_ERR, "zlac_8015d uninitialized\n");
      return ERROR;
    }

  data = g_command_list[CODE_SYNC_SPEED].command;

  data.data1_l = left_rpm & 0xff;
  data.data1_h = left_rpm >> 8;

  data.data2_l = right_rpm & 0xff;
  data.data2_h = right_rpm >> 8;

  status = pthread_mutex_lock(&zlac_8015d->motor_mutex);
  if (status != 0)
    {
      syslog(LOG_ERR, "zlac_8015d_sync_speed: ERROR pthread_mutex_lock failed, status=%d\n",
             status);
      ASSERT(false);
    }

  canopen_send(fd, (char *)&data, data_len);
  usleep(2000);
  canopen_receive(fd, (char *)&rec_buff, data_len);
  status = pthread_mutex_unlock(&zlac_8015d->motor_mutex);
  if (status != 0)
    {
      syslog(LOG_ERR, "zlac_8015d_sync_speed: ERROR pthread_mutex_unlock failed, status=%d\n",
             status);
      ASSERT(false);
    }

  if (rec_buff.index_l == data.index_l && rec_buff.index_h == data.index_h)
    {
      return OK;
    }
  else
    {
      syslog(LOG_ERR, "zlac_8015d read dump:data:%d,%d,%d,%d,%d,%d,%d,%d\n", rec_buff.sdo_cmd, rec_buff.index_l, rec_buff.index_h, rec_buff.sub_index, rec_buff.data1_l, rec_buff.data1_h, rec_buff.data2_l, rec_buff.data2_h);
      syslog(LOG_ERR, "zlac_8015d write speed check error %d  %d \n", rec_buff.index_h, rec_buff.index_l);
      return ERROR;
    }
}

static int zlac_8015d_set_pid(void *motor, enum control_mode_e mode,
                              struct motion_pid_s pid)
{
  struct motor_zlac_8015d_s *zlac_8015d = (struct motor_zlac_8015d_s *)motor;
  int                        kp, ki; /* kd is useless */
  int                        result = ERROR;

  if (zlac_8015d->initialized)
    {
      syslog(LOG_ERR, "ERROR: set pid zlac_8015d initialized\n");
      return ERROR;
    }

  if (abs(pid.kp) > 30000)
    {
      kp = 30000;
    }
  else
    {
      kp = abs(pid.kp);
    }

  if (abs(pid.ki) > 30000)
    {
      ki = 30000;
    }
  else
    {
      ki = abs(pid.ki);
    }

  syslog(LOG_INFO, "mode %d:SET PID\n", mode);
  switch (mode)
    {
        case SPEED: {
          result = zlac_8015d_write_single_opcode(zlac_8015d, CODE_SPEED_LEFT_KP, kp);
          usleep(2000);
          result |= zlac_8015d_write_single_opcode(zlac_8015d, CODE_SPEED_RIGHT_KP, kp);
          usleep(2000);
          result |= zlac_8015d_write_single_opcode(zlac_8015d, CODE_SPEED_LEFT_KI, ki);
          usleep(2000);
          result |= zlac_8015d_write_single_opcode(zlac_8015d, CODE_SPEED_RIGHT_KI, ki);
          syslog(LOG_DEBUG, "8015d set speed pid result=%d\n", result);
          break;
        }
      case POSITION:
        case TORQUE: {
          syslog(LOG_ERR, "mode %d:SET PID ERROR \n", mode);
          break;
        }
        default: {
          syslog(LOG_INFO, "invalid control mode %d\n", mode);
        }
    }

  return result;
}

/* current pid */
static int zlac_8015d_set_current_pid(void *motor, struct motion_pid_s pid)
{
  struct motor_zlac_8015d_s *zlac_8015d = (struct motor_zlac_8015d_s *)motor;
  int                        kp, ki; /* kd is useless */
  int                        result = ERROR;

  if (zlac_8015d->initialized)
    {
      syslog(LOG_ERR, "ERROR: set pid zlac_8015d initialized\n");
      return ERROR;
    }

  if (abs(pid.kp) > 30000)
    {
      kp = 30000;
    }
  else
    {
      kp = abs(pid.kp);
    }

  if (abs(pid.ki) > 30000)
    {
      ki = 30000;
    }
  else
    {
      ki = abs(pid.ki);
    }

  syslog(LOG_INFO, "SET current PID done\n");

  result = zlac_8015d_write_single_opcode(zlac_8015d, CODE_SET_CURRENT_KP_LEFT, kp);
  usleep(2000);
  result |= zlac_8015d_write_single_opcode(zlac_8015d, CODE_SET_CURRENT_KP_RIGHT, kp);
  usleep(2000);
  result |= zlac_8015d_write_single_opcode(zlac_8015d, CODE_SET_CURRENT_KI_LEFT, ki);
  usleep(2000);
  result |= zlac_8015d_write_single_opcode(zlac_8015d, CODE_SET_CURRENT_KI_RIGHT, ki);

  syslog(LOG_DEBUG, "8015d set current pid result=%d\n", result);

  return result;
}

static bool zlac_8015d_init(void *motor)
{
  int                        fd;
  struct motor_zlac_8015d_s *zlac_8015d = (struct motor_zlac_8015d_s *)motor;
  int                        status;
  struct motion_pid_s        pid;

  if (NULL == zlac_8015d)
    {
      syslog(LOG_ERR, "zlac_8015d failed invalid pointer \n");
      return ERROR;
    }

  if (zlac_8015d->initialized)
    {
      syslog(LOG_ERR, "ERROR: zlac_8015d_init  has initialized \n");
      return ERROR;
    }

  fd = open(MOTOR_DRIVER_DEV, O_RDWR | O_NONBLOCK);
  if (fd < 0)
    {
      syslog(LOG_ERR, "ERROR: open %s failed: %d\n", MOTOR_DRIVER_DEV, errno);
      close(fd);
      return ERROR;
    }
  zlac_8015d->driver_fd = fd;
  zlac_8015d->bus_mode  = MOTOR_BUS_MODE;

  /* Initialize the mutex */

  status = pthread_mutex_init(&zlac_8015d->motor_mutex, NULL);
  if (status != 0)
    {
      syslog(LOG_ERR, "start_thread: "
                      "ERROR pthread_mutex_init failed, status=%d\n",
             status);
      ASSERT(false);
    }

  /* Enable motor driver */

  /* clean driver error code */
  zlac_8015d_write_single_opcode(motor, CODE_CLEAN_ERROR, 0);
  usleep(2000);

  /*Default mode is SPEED*/

  status = zlac_8015d_write_single_opcode(motor, CODE_CAN_SYNC, 0);
  usleep(2000);

  status = zlac_8015d_speed_mode_init(zlac_8015d);
  syslog(LOG_DEBUG, "8015d init  mode init=%d\n", status);
  usleep(2000);
  status |= zlac_8015d_speed_config_acc(zlac_8015d);
  syslog(LOG_DEBUG, "8015d init  config ACC=%d\n", status);
  usleep(2000);
  status |= zlac_8015d_enable_can(zlac_8015d);
  syslog(LOG_DEBUG, "8015d init  enable CAN=%d\n", status);
  usleep(2000);
  status |= zlac_8015d_set_motor_pole(zlac_8015d);
  syslog(LOG_DEBUG, "8015d init  set pole=%d\n", status);
  usleep(2000);

  /* Set pid */
  pid.kp = SPEED_KP;
  pid.ki = SPEED_KI;
  status |= zlac_8015d_set_pid(zlac_8015d, SPEED, pid);
  syslog(LOG_DEBUG, "8015d init  set speed pid=%d\n", status);

  /* set current pid */
  pid.kp = CURRENT_KP;
  pid.ki = CURRENT_KI;
  status |= zlac_8015d_set_current_pid(zlac_8015d, pid);
  syslog(LOG_DEBUG, "8015d init  set speed pid=%d\n", status);

  if (status == OK)
    {
      zlac_8015d->initialized = true;
    }
  else
    {
      zlac_8015d->initialized = false;
    }

  return zlac_8015d->initialized;
}

static void zlac_8015d_deinit(void *motor)
{
  struct motor_zlac_8015d_s *zlac_8015d = (struct motor_zlac_8015d_s *)motor;

  close(zlac_8015d->driver_fd);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/