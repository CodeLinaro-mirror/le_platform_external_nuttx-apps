/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/
#include "qrc.h"

#ifdef QRC_RB5
#define QRC_IOC_MAGIC 'q'
#define QRC_FIONREAD _IO(QRC_IOC_MAGIC, 5)
#define QRC_FD ("/dev/qrc")
#define IOTAG QRC_FIONREAD
#endif

#ifdef QRC_MCB
#define QRC_FD ("/dev/ttyS2")
#define IOTAG FIONREAD
#endif

static pthread_mutex_t pipe_list_mutex = PTHREAD_MUTEX_INITIALIZER;
static qrc_pipe_s *pipe_list[64] = {NULL};
static int fd;

void TF_WriteImpl(TinyFrame *tf, const uint8_t *buff, uint32_t len);
TF_Result read_response_listener(TinyFrame *tf, TF_Msg *msg);
void *read_response();

void TF_WriteImpl(TinyFrame *tf, const uint8_t *buff, uint32_t len)
{
    // printf("--------------------\n");
    // printf("\033[32mWriter sending frame:\033[0m\n");
    
    int write_cnt = 0;
    write_cnt = write(fd, buff, len);
    if (write_cnt == (int)len) {
        printf("\033[94mWrite successfully!\033[0m\n");
    }
    else {
        printf("\033[94mWrite failed: write_cnt = %d\033[0m\n", write_cnt);
    }
}

bool qrc_write_request(const char *pipe_name, const uint8_t pipe_id, const uint8_t cmd)
{
  qrc_frame *qrcf = (qrc_frame*)malloc(sizeof(qrc_frame));
  qrcf->receiver_id = 0;

  qrc_msg *msg = (qrc_msg*)malloc(sizeof(qrc_msg));
  msg->cmd = cmd;
  msg->pipe_id = pipe_id;
  memset(msg->pipe_name, '\0', 10);
  memcpy(msg->pipe_name, pipe_name, strlen(pipe_name) * sizeof(char));
  
  bool send_result = qrc_frame_send(qrcf, (void*)msg, sizeof(qrc_msg));
  free(msg);
  free(qrcf);
  return send_result;
}

TF_Result read_response_listener(TinyFrame *tf, TF_Msg *msg)
{
  qrc_frame *qrcf = (qrc_frame*)malloc(sizeof(qrc_frame));
  memcpy(qrcf, msg->data, sizeof(qrc_frame));
  
  if(qrcf->receiver_id == 0) /*msg from qrc node*/
  {
    qrc_msg *qmsg = (qrc_msg*)malloc(sizeof(qrc_msg));
    memcpy(qmsg, msg->data + sizeof(qrc_frame), sizeof(qrc_msg));
    uint8_t cmd = qmsg->cmd;
    uint8_t pipe_id = qmsg->pipe_id;
    char *pipe_name = (char*)malloc(10);
    memcpy(pipe_name, qmsg->pipe_name, 10);

    if(cmd == QRC_REQUEST)
    {
      printf("\n-------------receiving QRC_REQUEST msg-------------\n");
      qrc_pipe_s *p = qrc_pipe_insert(pipe_name); /*create a corresponding pipe*/
      if(p == NULL)
      {
        printf("corresponding pipe(%s) create failed!\n", pipe_name);
      }
      else
      {
        p->peer_pipe_id = pipe_id;
        qrc_write_request(p->pipe_name, p->pipe_id, QRC_RESPONSE); /*send ack*/
      }
    }
    else if(cmd == QRC_RESPONSE)
    {
      printf("\n-------------receiving QRC_RESPONSE msg-------------\n");
      qrc_pipe_s *p = qrc_pipe_find_by_name(pipe_name);
      if(p == NULL)
      {
        printf("pipe name(%s) doesn't exit, can not handle QRC_RESPONSE!\n", pipe_name);
      }
      else
      {
        p->peer_pipe_id = pipe_id;
      }
    }
    free(qmsg);
    free(pipe_name);
  }
  else /*msg from app*/
  {
    printf("\n-------------receiving user msg-------------\n");
    qrc_pipe_s *p = qrc_pipe_find_by_pipeid(qrcf->receiver_id);
    if(NULL == p)
    {
      printf("here is no pipe with peer pipe id %u, receive failed!\n", qrcf->receiver_id);
    }
    else
    {
      if(NULL != p->cb)
      {
        uint8_t cb_len = msg->len - sizeof(qrc_frame);
        unsigned char *cb_data = (unsigned char*)malloc(cb_len);
        memcpy(cb_data, msg->data + sizeof(qrc_frame), cb_len);
        p->cb(p, (void*)cb_data, (size_t)cb_len, false);
        free(cb_data);
      }
    }
  }
  free(qrcf);
  return TF_STAY;
}

void *read_response()
{
  TinyFrame *tf = TF_Init(TF_MASTER);
  TF_AddGenericListener(tf, read_response_listener);

  while(1)
  {
    int readable_len = 0;
    if(ioctl(fd, IOTAG, &readable_len) < 0)
    {
      printf("\nqrc get readable size fail!\n");
      exit(-1);
    }
    if(readable_len > 0)
    {
      void *buf = malloc(readable_len * sizeof(char));
      int read_len = read(fd, buf, readable_len);
      if(read_len > 0) {
        TF_Accept(tf, (uint8_t*)buf, (uint32_t)read_len);
      }
      free(buf);
    }
    else
      usleep(10);
  }
}

void qrc_init(void)
{
    fd = open(QRC_FD, O_RDWR);
    pthread_t t;
    pthread_create(&t, NULL, read_response, NULL);
    if(-1 == fd)
    {
        printf("%s open failed!\n", QRC_FD);
        exit(-1);
    }
    qrc_pipe_list_init();
}

/*
* init a pipe node and return it
*/
qrc_pipe_s *qrc_pipe_node_init(void)
{
    qrc_pipe_s *node = (qrc_pipe_s*)malloc(sizeof(qrc_pipe_s));
    memset(node->pipe_name, '\0', 10);

    if(0 != pthread_cond_init(&node->pipe_cond, NULL))
    {
        printf("\npipe cond initalize failed!\n");
        return NULL;
    }
    if(0 != pthread_mutex_init(&node->pipe_mutex, NULL))
    {
        printf("\npipe mutex initalize failed!\n");
        return NULL;
    }
    node->pipe_id = 255;
    node->peer_pipe_id = 255;
    node->cb = NULL;
    return node;
}

/*
* init the pipe list, pipe_list[1] is only for qrc
* return the head of list
*/
void qrc_pipe_list_init(void)
{
    pthread_mutex_lock(&pipe_list_mutex);
    pipe_list[0] = qrc_pipe_node_init();
    pipe_list[0]->pipe_id = 1;

    pipe_list[1] = qrc_pipe_node_init();
    pipe_list[1]->pipe_id = 0;
    pipe_list[1]->peer_pipe_id = 0;

    pthread_mutex_unlock(&pipe_list_mutex);
}

/*
* insert a new pipe behind head of pipe list
* return NULL/new pipe/found
*/
qrc_pipe_s *qrc_pipe_insert(const char *pipe_name)
{
    pthread_mutex_lock(&pipe_list_mutex);
    qrc_pipe_s **lt = pipe_list;
    if(lt[0] == NULL || lt[0]->pipe_id >= 63)
    {
        return NULL;
    }
    
    qrc_pipe_s *find_res = qrc_pipe_find_by_name(pipe_name);
    if(NULL != find_res)
    {
        pthread_mutex_unlock(&pipe_list_mutex);
        return find_res;
    }

    lt[0]->pipe_id = (lt[0]->pipe_id + 1) % 64;
    uint8_t new_pipe_index = lt[0]->pipe_id;
    lt[new_pipe_index] = qrc_pipe_node_init();
    lt[new_pipe_index]->pipe_id = new_pipe_index;
    memcpy(lt[new_pipe_index]->pipe_name, pipe_name, strlen(pipe_name) * sizeof(char));
    pthread_mutex_unlock(&pipe_list_mutex);

    return lt[new_pipe_index];
}

/*
* find by pipe name
* if success return found pipe, else return NULL
*/
qrc_pipe_s *qrc_pipe_find_by_name(const char *pipe_name)
{
    qrc_pipe_s **lt = pipe_list;
    if(NULL == lt[0])
    {
        printf("pipe list is NULL! find by name failed!");
        exit(-1);
    }
    int i;
    for(i = 0; i < 64; i++)
    {
        if(NULL == lt[i] || 0 == strcmp(lt[i]->pipe_name, pipe_name))
        {
            break;
        }
    }

    return lt[i];
}

/*
* find by peer pipe id
* if success return found pipe, else return NULL
*/
qrc_pipe_s *qrc_pipe_find_by_pipeid(const uint8_t pipe_id)
{
    qrc_pipe_s **lt = pipe_list;
    if(NULL == lt[0])
    {
        printf("pipe list is NULL! find by peer pipe id failed!");
        exit(-1);
    }

    return lt[pipe_id];
}

/*
* modify by name
* return modified pipe
*/
qrc_pipe_s *qrc_pipe_modify_by_name(const char *pipe_name, const qrc_pipe_s *new_data)
{
    return NULL;
}

/*
* msg.data = qrc frame = qrc_frame + data
*/
bool qrc_frame_send(const qrc_frame *qrcf, const void *data, const size_t len)
{
    TF_Msg msg;
    TF_ClearMsg(&msg);
    msg.type = DEFAULT_TF_MSG_TYPE;
    uint8_t *msg_data = (uint8_t*)malloc(sizeof(qrc_frame) + len);
    memcpy(msg_data, qrcf, sizeof(qrc_frame));
    memcpy(msg_data + sizeof(qrc_frame), data, len);
    msg.data = msg_data;
    msg.len = sizeof(qrc_frame) + len;
    //printf("in qrc_frame_send(), msg.len = %u", msg.len);

    TinyFrame *tf;
    tf = TF_Init(TF_MASTER);
    return TF_Send(tf, &msg);
}