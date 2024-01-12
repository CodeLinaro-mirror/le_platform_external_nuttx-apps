/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/
#ifndef __QRC_H
#define __QRC_H

#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stddef.h>
#include <unistd.h>
#include <errno.h>
#include "TinyFrame.h"

#define DEFAULT_TF_MSG_TYPE 0x22
#define QRC_REQUEST 1
#define QRC_RESPONSE 2

typedef struct qrc_msg
{
    uint8_t cmd;
    uint8_t pipe_id;
    char pipe_name[10];
} qrc_msg;

typedef struct qrc_frame
{
    uint8_t sync_mode: 1;
    uint8_t ack: 1;
    uint8_t receiver_id: 6;
} qrc_frame;

typedef struct qrc_pipe_s
{
    char pipe_name[10];
    pthread_cond_t pipe_cond;
    pthread_mutex_t pipe_mutex;
    uint8_t pipe_id;
    uint8_t peer_pipe_id;
    void (*cb)(struct qrc_pipe_s *pipe, void *data, size_t len, bool response);
} qrc_pipe_s;

typedef void (*qrc_msg_cb)(struct qrc_pipe_s *pipe, void *data, size_t len, bool response);

bool qrc_write_request(const char *pipe_name, const uint8_t pipe_id, const uint8_t cmd);
void qrc_init(void);
qrc_pipe_s *qrc_pipe_node_init(void);
void qrc_pipe_list_init(void);
qrc_pipe_s *qrc_pipe_insert(const char *pipe_name);
qrc_pipe_s *qrc_pipe_find_by_name(const char *pipe_name);
qrc_pipe_s *qrc_pipe_find_by_pipeid(const uint8_t pipe_id);
qrc_pipe_s *qrc_pipe_modify_by_name(const char *pipe_name, const qrc_pipe_s *new_data);
bool qrc_frame_send(const qrc_frame *qrcf, const void *data, const size_t len);


/* qrc thread pool */
struct qrc_msg_cb_args_s
{
  qrc_msg_cb fun_cb;  /* qrc_msg_cb */
  struct qrc_pipe_s *pipe;
  void *data;
  size_t len;
  bool response;
};

typedef struct qrc_thread_pool_s * qrc_thread_pool;
typedef void (*qrc_work)(struct qrc_msg_cb_args_s args);

struct qrc_thread_pool_s * qrc_thread_pool_init(int num);
int qrc_threadpool_add_work(struct qrc_thread_pool_s * thpool, qrc_work work_fun, struct qrc_msg_cb_args_s args);
void qrc_threadpool_wait(struct qrc_thread_pool_s * thpool);
void qrc_threadpool_destroy(struct qrc_thread_pool_s * thpool);

#endif