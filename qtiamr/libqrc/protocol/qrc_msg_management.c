/***************************************************************************
* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
****************************************************************************/
/****************************************************************************
 * Included Files
 ****************************************************************************/
#include "qrc_msg_management.h"

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

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: qrc_get_pipe
 ****************************************************************************/

void init_qrc_management()
{
  qrc_init();
}

qrc_pipe_s *qrc_get_pipe(const char *pipe_name)
{
  int pipe_name_len = (int)strlen(pipe_name);
  if (pipe_name_len > 10)
  {
    printf("\npipe name is too long!\n");
    return NULL;
  }
  qrc_pipe_s *p = qrc_pipe_insert(pipe_name);
  if(NULL == p)
  {
    printf("pipe(%s) create failed!\n", pipe_name);
    return NULL;
  }
  qrc_write_request(pipe_name, p->pipe_id, QRC_REQUEST);
  return p;
}

bool qrc_register_message_cb(qrc_pipe_s *pipe, qrc_msg_cb fun_cb)
{
  if (pipe == NULL)
  {
    printf("pipe is NULL! callback register failed!\n");
    return false;
  }
  pipe->cb = fun_cb;
  return true;
}

/*
* wait for implement
*/
enum qrc_write_status_e qrc_write(const qrc_pipe_s *pipe , const void *data, const size_t len, const bool ack)
{
  return FAILED;
}

/*
* wait for implement
*/
enum qrc_write_status_e qrc_sync_write(const qrc_pipe_s *pipe , const void *data, const size_t len, const void *respond_data, const size_t res_len)
{
  return FAILED;
}

enum qrc_write_status_e qrc_write_fast(const qrc_pipe_s *pipe , const void *data, const size_t len, const bool ack)
{
  while(255 == pipe->peer_pipe_id || 0 == pipe->peer_pipe_id) /*haven't got peer pipe id*/
  {
    usleep(1);
  }
  qrc_frame *qrcf = (qrc_frame*)malloc(sizeof(qrc_frame));
  qrcf->receiver_id = pipe->peer_pipe_id;
  if (true == ack)
  {
    qrcf->ack = 1;
  }
  else
  {
    qrcf->ack = 0;
  }

  bool send_result = qrc_frame_send(qrcf, data, len);
  free(qrcf);
  if(true == send_result)
  {
    return SUCCESS;
  }
  return FAILED;
}

/*
* wait for implement
*/
enum qrc_write_status_e qrc_response(const qrc_pipe_s *pipe , const void *data, const size_t len)
{
  return FAILED;
}
