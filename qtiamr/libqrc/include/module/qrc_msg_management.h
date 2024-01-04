/***************************************************************************
* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
****************************************************************************/

#ifndef __QRC_MSG_MANAGEMENT_H
#define __QRC_MSG_MANAGEMENT_H

typedef void (*qrc_msg_cb)(struct qrc_pipe_s *pipe,void * data, size_t len, bool response);  /* message callback function format */

struct qrc_pipe_s
{
  char name[12];
  uint8_t session_id;
}__attribute__((aligned(4)));

enum qrc_write_status_e
{
  SUCCESS = 0,
  TIMEOUT,
  FAILED
};

struct qrc_pipe_s *qrc_get_pipe(const char *pipe_name);
bool qrc_register_message_cb(struct qrc_pipe_s *pipe, qrc_msg_cb fun_cb);
enum qrc_write_status_e qrc_write(struct qrc_pipe_s *pipe , void *data, size_t len, bool ack);
enum qrc_write_status_e qrc_sync_write(struct qrc_pipe_s *pipe , void *data, size_t len,void *respond_data, size_t res_len);

enum qrc_write_status_e qrc_write_fast(struct qrc_pipe_s *pipe , void *data, size_t len, bool ack);
enum qrc_write_status_e qrc_response(struct qrc_pipe_s *pipe , void *data, size_t len);

#endif