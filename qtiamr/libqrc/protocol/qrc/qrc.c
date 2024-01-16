/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/
#include "qrc.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/
typedef struct g_vars
{
  pthread_mutex_t pipe_list_mutex;
  pthread_mutex_t qrc_write_mutex;
  qrc_pipe_s pipe_list[64];
  int fd;
  TinyFrame *tf;
  qrc_thread_pool g_qrc_threadpool;
  uint8_t pipe_cnt;
} g_vars;

static g_vars g_data;

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define QRC_THREAD_NUM (2)
#define MCB_RESET_MAGIC_CMD 0x7102
#define DEFAULT_TF_MSG_TYPE 0x22

#ifdef QRC_RB5
#define QRC_IOC_MAGIC 'q'
#define QRC_FIONREAD _IO(QRC_IOC_MAGIC, 5)
#define QRC_FD ("/dev/qrc")
#define IOTAG QRC_FIONREAD
static void sig_handler(int sig)
{
  close(g_data.fd);
  printf("actually close..........\n");
}
#endif

#ifdef QRC_MCB
#define QRC_FD ("/dev/ttyS2")
#define IOTAG FIONREAD
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/
void TF_WriteImpl(TinyFrame *tf, const uint8_t *buff, uint32_t len);
TF_Result read_response_listener(TinyFrame *tf, TF_Msg *msg);
void *read_response(void *args);
void qrc_control_pipe_callback(qrc_pipe_s *pipe, void * data, size_t len, bool response);
void end_timeout(const uint8_t pipe_id);
static void qrc_msg_cb_work(struct qrc_msg_cb_args_s args);

/****************************************************************************
 * @intro: send TF frame
 * @param tf: tf
 * @param buff: TF frame
 * @param len: length of buff
 ****************************************************************************/
void TF_WriteImpl(TinyFrame *tf, const uint8_t *buff, uint32_t len)
{
  uint32_t write_cnt = 0;
  write_cnt = write(g_data.fd, buff, len);
  if (write_cnt != len)
  {
    printf("ERROR: Write failed!\n");
  }
}

/****************************************************************************
 * @intro: for connection establishment of new pipe, can be only used by pipe_list[1]
 * @param pipe_name: pipe name of caller
 * @param pipe_id: pipe id of caller
 * @param cmd: enum qrc_msg_cmd
 * @return: result of TF_Send()
 ****************************************************************************/
bool qrc_write_request(const char *pipe_name, const uint8_t pipe_id, const enum qrc_msg_cmd cmd)
{
  qrc_frame qrcf;
  qrcf.receiver_id = 0;
  qrcf.ack = NO_ACK;

  qrc_msg msg;
  msg.cmd = cmd;
  msg.pipe_id = pipe_id;
  memset(msg.pipe_name, '\0', 10);
  memcpy(msg.pipe_name, pipe_name, strlen(pipe_name) * sizeof(char));

  bool send_result = qrc_frame_send(&qrcf, (void*)(&msg), sizeof(qrc_msg), true);
  if(true == send_result && (QRC_REQUEST== cmd || QRC_WRITE_LOCK == cmd || QRC_WRITE_UNLOCK == cmd))
  {
    g_data.pipe_list[0].timeout_happen = false;
    start_timeout(0);
  }
  return send_result;
}

/****************************************************************************
 * @intro: this function will be called when tf receive msg
 * @param tf: receiver
 * @param msg: msg received
 * @return: TF_STAY
 ****************************************************************************/
TF_Result read_response_listener(TinyFrame *tf, TF_Msg *msg)
{
  qrc_frame qrcf;
  memcpy(&qrcf, msg->data, sizeof(qrc_frame));

  qrc_pipe_s *p = qrc_pipe_find_by_pipeid(qrcf.receiver_id);
  if(NULL == p)
  {
    printf("ERROR: here is no pipe with peer pipe id %u, receive failed!\n", qrcf.receiver_id);
  }
  else
  {
    if(ACK == qrcf.ack)
    {
      qrc_write_request(p->pipe_name, p->peer_pipe_id, QRC_ACK);
    }
    if(NULL != p->cb)
    {
      struct qrc_msg_cb_args_s args;
      args.fun_cb = p->cb;
      args.pipe = p;
      args.data = (void*)(msg->data + sizeof(qrc_frame));
      args.len = msg->len - sizeof(qrc_frame);
      args.response = false;
      qrc_threadpool_add_work(g_data.g_qrc_threadpool, qrc_msg_cb_work, args);
    }
  }

  return TF_STAY;
}

/****************************************************************************
 * @intro: callback function of pipelist[1], whose pipe id is 0
 * @param pipe: pipelist[1]
 * @param data: data received
 * @param len: length of data
 * @param response: no use
 ****************************************************************************/
void qrc_control_pipe_callback(qrc_pipe_s *pipe, void * data, size_t len, bool response)
{
  qrc_msg qmsg;
  memcpy(&qmsg, data, sizeof(qrc_msg));
  uint8_t cmd = qmsg.cmd;
  uint8_t pipe_id = qmsg.pipe_id; /*pipe id of receiver*/
  char pipe_name[10] = "\0";
  memcpy(pipe_name, qmsg.pipe_name, 10);
  if(QRC_REQUEST == cmd)
  {
    qrc_pipe_s *p = qrc_pipe_insert(pipe_name);
    if(p == NULL)
    {
      printf("ERROR: corresponding pipe(%s) create failed!\n", pipe_name);
    }
    else
    {
      p->peer_pipe_id = pipe_id;
      qrc_write_request(p->pipe_name, p->pipe_id, QRC_RESPONSE);
    }
  }
  else if(QRC_RESPONSE == cmd)
  {
    qrc_pipe_s *p = qrc_pipe_find_by_name(pipe_name);
    if(p == NULL)
    {
      printf("ERROR: pipe name(%s) doesn't exit, can not handle QRC_RESPONSE!\n", pipe_name);
    }
    else
    {
      p->peer_pipe_id = pipe_id;
      end_timeout(0);
    }
  }
  else if(QRC_WRITE_LOCK == cmd)
  {
    qrc_pipe_s *p = qrc_pipe_find_by_name(pipe_name);
    qrc_write_request(p->pipe_name, p->pipe_id, QRC_WRITE_LOCK_ACK);
    qrc_frame_send_lock();
  }
  else if(QRC_WRITE_UNLOCK == cmd)
  {
    qrc_pipe_s *p = qrc_pipe_find_by_name(pipe_name);
    qrc_write_request(p->pipe_name, p->pipe_id, QRC_WRITE_UNLOCK_ACK);
    qrc_frame_send_unlock();
  }
  else if(QRC_ACK == cmd)
  {
    qrc_pipe_s *p = qrc_pipe_find_by_pipeid(pipe_id);
    end_timeout(p->pipe_id); /*pipe_id == user id*/
  }
  else if(cmd == QRC_WRITE_LOCK_ACK || cmd == QRC_WRITE_UNLOCK_ACK)
  {
    end_timeout(0);
  }
}

/****************************************************************************
 * @intro: initilize a new pipe
 * @return: new pipe
 ****************************************************************************/
qrc_pipe_s qrc_pipe_node_init(void)
{
  qrc_pipe_s node;
  memset(node.pipe_name, '\0', 10);

  if(0 != pthread_cond_init(&node.pipe_cond, NULL))
  {
    printf("\nERROR: pipe cond initalize failed!\n");
    exit(-1);
  }
  if(0 != pthread_mutex_init(&node.pipe_mutex, NULL))
  {
    printf("\nERROR: pipe mutex initalize failed!\n");
    exit(-1);
  }
  node.pipe_id = 255;
  node.peer_pipe_id = 255;
  node.timeout_happen = false;
  node.cb = NULL;
  return node;
}

/****************************************************************************
 * @intro: initilize the pipe list
 ****************************************************************************/
void qrc_pipe_list_init(void)
{
  pthread_mutex_lock(&g_data.pipe_list_mutex);

  g_data.pipe_list[0] = qrc_pipe_node_init();
  g_data.pipe_list[0].pipe_id = 0;
  g_data.pipe_list[0].peer_pipe_id = 0;
  char *pipe_name = "QRC_ctl";
  memcpy(g_data.pipe_list[0].pipe_name, pipe_name, strlen(pipe_name) * sizeof(char));
  g_data.pipe_list[0].cb = qrc_control_pipe_callback;
  g_data.pipe_cnt = 1;

  pthread_mutex_unlock(&g_data.pipe_list_mutex);
}

/****************************************************************************
 * @intro: create a new pipe named pipe_name
 * @return: pointer of new pipe or exited pipe
 ****************************************************************************/
qrc_pipe_s *qrc_pipe_insert(const char *pipe_name)
{
  pthread_mutex_lock(&g_data.pipe_list_mutex);
  qrc_pipe_s *lt = g_data.pipe_list;
  if(g_data.pipe_cnt >= 63)
  {
    pthread_mutex_unlock(&g_data.pipe_list_mutex);
    return NULL;
  }
  qrc_pipe_s *find_res = qrc_pipe_find_by_name(pipe_name);
  if(NULL == find_res)
  {
    uint8_t new_pipe_index = g_data.pipe_cnt;
    g_data.pipe_cnt = (g_data.pipe_cnt + 1) % 64;
    lt[new_pipe_index] = qrc_pipe_node_init();
    lt[new_pipe_index].pipe_id = new_pipe_index;
    memcpy(lt[new_pipe_index].pipe_name, pipe_name, strlen(pipe_name) * sizeof(char));
    find_res = &lt[new_pipe_index];
  }
  pthread_mutex_unlock(&g_data.pipe_list_mutex);
  return find_res;
}

/****************************************************************************
 * @intro: find a pipe named pipe_name
 * @return: pointer of pipe or NULL
 ****************************************************************************/
qrc_pipe_s *qrc_pipe_find_by_name(const char *pipe_name)
{
  for(uint8_t i = 1; i < g_data.pipe_cnt; i++)
  {
    if(0 == strcmp(g_data.pipe_list[i].pipe_name, pipe_name))
    {
      return &g_data.pipe_list[i];
    }
  }
  return NULL;
}

/****************************************************************************
 * @intro: find a pipe by its pipe_id
 * @return: pointer of pipe or NULL
 ****************************************************************************/
qrc_pipe_s *qrc_pipe_find_by_pipeid(const uint8_t pipe_id)
{
  if(g_data.pipe_cnt <= pipe_id)
  {
    return NULL;
  }
  return &g_data.pipe_list[pipe_id];
}

/****************************************************************************
 * @intro: modify the data of pipe
 * @return: pointer of pipe or NULL
 ****************************************************************************/
qrc_pipe_s *qrc_pipe_modify_by_name(const char *pipe_name, const qrc_pipe_s *new_data)
{
  return NULL;
}

/****************************************************************************
 * @intro: send TF frame
 * @param qrcf: qrcf_frame(sync_mode + ack + receiver_id)
 * @param data: qrc_msg(qrc_msg_cmd + pipe id + pipe name) or user data
 * @param len: length of data
 * @param qrc_write_lock: whether hold lock to ensure the integrity of the frame, default is true
 * @return: result of TF_Send()
 ****************************************************************************/
bool qrc_frame_send(const qrc_frame *qrcf, const void *data, const size_t len, const bool qrc_write_lock)
{   
  if(true == qrc_write_lock)
  {
    pthread_mutex_lock(&g_data.qrc_write_mutex);
  }
  TF_Msg msg;
  TF_ClearMsg(&msg);
  msg.type = DEFAULT_TF_MSG_TYPE;
  uint8_t *msg_data = (uint8_t*)malloc(sizeof(qrc_frame) + len);
  memcpy(msg_data, qrcf, sizeof(qrc_frame));
  memcpy(msg_data + sizeof(qrc_frame), data, len);
  msg.data = msg_data;
  msg.len = sizeof(qrc_frame) + len;

  bool send_res = TF_Send(g_data.tf, &msg);
  free(msg_data);
  if(true == qrc_write_lock)
  {
    pthread_mutex_unlock(&g_data.qrc_write_mutex);
  }
  return send_res;
}

/****************************************************************************
 * @intro: start the timeout of pipe whose pipe id is pipe_id
 * @param pipe_id: pipe id
 ****************************************************************************/
void start_timeout(const uint8_t pipe_id)
{
  qrc_pipe_s *p = qrc_pipe_find_by_pipeid(pipe_id);
  p->timeout_happen = false;
  struct timeval now;
  gettimeofday(&now, NULL);

  struct timespec outtime;
  outtime.tv_sec = now.tv_sec;
  outtime.tv_nsec = now.tv_usec + 500000000; /*500ms*/

  pthread_mutex_lock(&p->pipe_mutex);
  if(0 != pthread_cond_timedwait(&p->pipe_cond, &p->pipe_mutex, &outtime))
  {
    printf("\nERROR: pipe(%s) TIMEOUT!\n", p->pipe_name);
    p->timeout_happen = true;
  }
  pthread_mutex_unlock(&p->pipe_mutex);
}

/****************************************************************************
 * @intro: wake up the timeout of pipe whose pipe id is pipe_id
 * @param pipe_id: pipe id
 ****************************************************************************/
void end_timeout(const uint8_t pipe_id)
{
  qrc_pipe_s *p = qrc_pipe_find_by_pipeid(pipe_id);
  pthread_mutex_lock(&p->pipe_mutex);
  if(0 != pthread_cond_signal(&p->pipe_cond))
  {
    printf("\nERROR: Can not wake up main thread!\n");
    exit(-1);
  }
  pthread_mutex_unlock(&p->pipe_mutex);
}

/****************************************************************************
 * @intro: lock the qrc_write_mutex
 ****************************************************************************/
void qrc_frame_send_lock(void)
{
  pthread_mutex_lock(&g_data.qrc_write_mutex);
}

/****************************************************************************
 * @intro: unlock the qrc_write_mutex
 ****************************************************************************/
void qrc_frame_send_unlock(void)
{
  pthread_mutex_unlock(&g_data.qrc_write_mutex);
}

/****************************************************************************
 * @intro: query if pipe0's timeout happen
 ****************************************************************************/
bool qrc_cmd_timeout(void)
{
  return g_data.pipe_list[0].timeout_happen;
}

/****************************************************************************
 * @intro: thread of reading response
 ****************************************************************************/
void *read_response(void *args)
{
  while(1)
  {
    int readable_len = 0;
    if(ioctl(g_data.fd, IOTAG, &readable_len) < 0)
    {
      printf("\nERROR: qrc get readable size fail!\n");
      exit(-1);
    }
    if(readable_len > 0)
    {
      void *buf = malloc(readable_len * sizeof(char));
      int read_len = read(g_data.fd, buf, readable_len);
      if(read_len > 0) {
        TF_Accept(g_data.tf, (uint8_t*)buf, (uint32_t)read_len);
      }
      free(buf);
    }
    else
      usleep(10);
  }
}

/****************************************************************************
 * @intro: execute the function args.fun_cb
 * @param args: thread holder
 ****************************************************************************/
static void qrc_msg_cb_work(struct qrc_msg_cb_args_s args)
{
  args.fun_cb(args.pipe, args.data, args.len, args.response);
}

uint8_t get_pipe_number(void)
{
  return g_data.pipe_cnt;
}

/****************************************************************************
 * @intro: initilial
 ****************************************************************************/
void qrc_init(void)
{
  g_data.fd = open(QRC_FD, O_RDWR);
  if(-1 == g_data.fd)
  {
    printf("ERROR: %s open failed!\n", QRC_FD);
    exit(-1);
  }
  if(0 != pthread_mutex_init(&g_data.pipe_list_mutex, NULL))
  {
    printf("\nERROR: pipe mutex initalize failed!\n");
    exit(-1);
  }
  if(0 != pthread_mutex_init(&g_data.qrc_write_mutex, NULL))
  {
    printf("\nERROR: pipe mutex initalize failed!\n");
    exit(-1);
  }
  g_data.g_qrc_threadpool = qrc_thread_pool_init(QRC_THREAD_NUM);
  g_data.tf = TF_Init(TF_MASTER);
  TF_AddGenericListener(g_data.tf, read_response_listener);

  #ifdef QRC_RB5
  signal(SIGINT, sig_handler);
  #endif
  pthread_t t;
  pthread_create(&t, NULL, read_response, NULL);
  qrc_pipe_list_init();
}