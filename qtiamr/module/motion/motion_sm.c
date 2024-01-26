/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <signal.h>
#include <pthread.h>
#include <time.h>

#include "motion_sm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* motion sm structure */

struct motion_sm_s
{
  enum motion_sm_state_e state;
  pthread_mutex_t mutex;
  motion_cb switch_done_cb;
  motion_cb position_done_cb;
  motion_cb drv_err_cb;
  void *pose_cb_data;
  void *switch_cb_data;
  void *drv_err_cb_data;

  struct motion_thread_pool_s *threadpool;
}__attribute__((aligned(4)));

struct motion_sm_transform_s
{
  enum motion_sm_event_e event;
  enum motion_sm_state_e current_state;
  enum motion_sm_state_e next_state;
  do_action_fun action_fun;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int motion_sm_do_action(struct motion_sm_transform_s *statetrans, union motion_control_data_u data);

static bool acquire_motion_sm_lock(void);
static void release_motion_sm_lock(void);

static int do_action_switch(union motion_control_data_u data);
static int do_action_emergency(union motion_control_data_u data);
static int do_action_drv_error(union motion_control_data_u data);
static int do_action_speed(union motion_control_data_u data);
static int do_action_switch_done(union motion_control_data_u data);

/* need to do: position done & switch done callback function */

/* state machine action work queue */
static void motion_action_work(struct motion_args_s args);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct motion_sm_s g_motion_sm;

/* inactive */
struct motion_sm_transform_s statetrans_inactive[]={
  {EV_CMD_SWITCH_SPEED, ST_INACTIVE, ST_SWITCHING,  do_action_switch},
  {EV_CMD_SWITCH_POS,   ST_INACTIVE, ST_SWITCHING,  do_action_switch},
  {EV_SPEED_SWITCH_DONE,ST_INACTIVE, ST_INACTIVE,   NULL},
  {EV_POS_SWITCH_DONE,  ST_INACTIVE, ST_INACTIVE,   NULL},
  {EV_CMD_SPEED,        ST_INACTIVE, ST_INACTIVE,   NULL},
  {EV_CMD_POSITION,     ST_INACTIVE, ST_INACTIVE,   NULL},
  {EV_POSITION_ATTACHED,ST_INACTIVE, ST_INACTIVE,   NULL},
  {EV_ENTER_EMERGENCY,  ST_INACTIVE, ST_INACTIVE,   NULL},
  {EV_EXIT_EMERGENCY,   ST_INACTIVE, ST_INACTIVE,   NULL},
  {EV_DRI_ERR,          ST_INACTIVE, ST_DRIVER_ERR, NULL},
};

/* switching */
struct motion_sm_transform_s statetrans_switching[]={
  {EV_CMD_SWITCH_SPEED, ST_SWITCHING, ST_SWITCHING,     NULL},
  {EV_CMD_SWITCH_POS,   ST_SWITCHING, ST_SWITCHING,     NULL},
  {EV_SPEED_SWITCH_DONE,ST_SWITCHING, ST_SPEED,         do_action_switch_done},
  {EV_POS_SWITCH_DONE,  ST_SWITCHING, ST_POSITION_IDLE, do_action_switch_done},
  {EV_CMD_SPEED,        ST_SWITCHING, ST_SWITCHING,     NULL},
  {EV_CMD_POSITION,     ST_SWITCHING, ST_SWITCHING,     NULL},
  {EV_POSITION_ATTACHED,ST_SWITCHING, ST_SWITCHING,     NULL},
  {EV_ENTER_EMERGENCY,  ST_SWITCHING, ST_EMERGENCY,     do_action_emergency},
  {EV_EXIT_EMERGENCY,   ST_SWITCHING, ST_SWITCHING,     NULL},
  {EV_DRI_ERR,          ST_SWITCHING, ST_INACTIVE,      do_action_drv_error},
};

/* speed */
struct motion_sm_transform_s statetrans_speed[]={
  {EV_CMD_SWITCH_SPEED, ST_SPEED, ST_SWITCHING,     do_action_switch},
  {EV_CMD_SWITCH_POS,   ST_SPEED, ST_SWITCHING,     do_action_switch},
  {EV_SPEED_SWITCH_DONE,ST_SPEED, ST_SPEED,         NULL},
  {EV_POS_SWITCH_DONE,  ST_SPEED, ST_SPEED,         NULL},
  {EV_CMD_SPEED,        ST_SPEED, ST_SPEED,         do_action_speed},
  {EV_CMD_POSITION,     ST_SPEED, ST_SPEED,         NULL},
  {EV_POSITION_ATTACHED,ST_SPEED, ST_SPEED,         NULL},
  {EV_ENTER_EMERGENCY,  ST_SPEED, ST_EMERGENCY,     do_action_emergency},
  {EV_EXIT_EMERGENCY,   ST_SPEED, ST_SPEED,         NULL},
  {EV_DRI_ERR,          ST_SPEED, ST_INACTIVE,      do_action_drv_error},
};

/* emergency */
struct motion_sm_transform_s statetrans_emergency[]={
  {EV_CMD_SWITCH_SPEED, ST_EMERGENCY, ST_EMERGENCY,     NULL},
  {EV_CMD_SWITCH_POS,   ST_EMERGENCY, ST_EMERGENCY,     NULL},
  {EV_SPEED_SWITCH_DONE,ST_EMERGENCY, ST_EMERGENCY,     NULL},
  {EV_POS_SWITCH_DONE,  ST_EMERGENCY, ST_EMERGENCY,     NULL},
  {EV_CMD_SPEED,        ST_EMERGENCY, ST_EMERGENCY,     NULL},
  {EV_CMD_POSITION,     ST_EMERGENCY, ST_EMERGENCY,     NULL},
  {EV_POSITION_ATTACHED,ST_EMERGENCY, ST_EMERGENCY,     NULL},
  {EV_ENTER_EMERGENCY,  ST_EMERGENCY, ST_EMERGENCY,     NULL},
  {EV_EXIT_EMERGENCY,   ST_EMERGENCY, ST_INACTIVE,      do_action_emergency},
  {EV_DRI_ERR,          ST_EMERGENCY, ST_INACTIVE,      do_action_drv_error},
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void motion_action_work(struct motion_args_s args)
{
  args.fun_cb(args.data);
}

static bool acquire_motion_sm_lock(void)
{
  int status;

  status = pthread_mutex_lock(&g_motion_sm.mutex);

  if (status == 0)
    return true;
  else
    return false;
}

static void release_motion_sm_lock(void)
{
  int status;
  status = pthread_mutex_unlock(&g_motion_sm.mutex);
  if (status != 0)
    {
      syslog(LOG_ERR,"pthread_rwlock:"
                      "ERROR Failed to unlock lock. Status: %d\n", status);
      ASSERT(false);
    }
}

static int motion_sm_do_action(struct motion_sm_transform_s *statetrans, union motion_control_data_u data)
{
  enum motion_sm_state_e *curr_state;

  if(NULL == statetrans)
    {
      syslog(LOG_ERR,"pointer statetrans is invalid \n");
      return ERROR;
    }

  /*lock sm */
  if (!acquire_motion_sm_lock())
    {
      release_motion_sm_lock();
      syslog(LOG_ERR,"acquire motion sm lock failed\n");
      return ERROR;
    }

  curr_state = &g_motion_sm.state;
  /* check again the present state if matched */
  if (*curr_state == statetrans->current_state)
    {
      *curr_state = statetrans->next_state;
      release_motion_sm_lock();
    }
  else
    {
      release_motion_sm_lock();
      syslog(LOG_INFO,"motion sm current state is changed=%d \n", *curr_state);
      return ERROR;
    }

  /* do action */

  if(statetrans->action_fun != NULL)
    {
      /* call action function */
      //result = statetrans->action_fun(data);
      struct motion_args_s motion_args;
      motion_args.fun_cb = statetrans->action_fun;
      motion_args.data = data;
      motion_threadpool_add_work(g_motion_sm.threadpool, motion_action_work, motion_args);
      //need check return value
      syslog(LOG_DEBUG,"motion sm action_fun executed \n");
      return OK;
    }
  else
    {
      syslog(LOG_INFO,"motion sm no action \n");
      return OK;
    }
}


/****************************************************************************
 * Action functions
 ****************************************************************************/

static int do_action_switch(union motion_control_data_u data)
{
  enum control_mode_e mode = data.mode;
  int result;

  /* call motor api to set motor mode */
  result = motor_switch_mode(mode);
syslog(LOG_INFO,"do_action_switch:  result = %d mode=%d \n",result,mode);
  if (OK == result)
    {
      if (SPEED == mode)
        {
          /* send event to notify switch done */
          syslog(LOG_INFO,"switch motion speed done change state\n");
          motion_sm_event(EV_SPEED_SWITCH_DONE, data);
        }
      else if (POSITION == mode)
        {
          motion_sm_event(EV_POS_SWITCH_DONE, data);
        }
      else
        {
          syslog(LOG_ERR,"ERROR:do_action_switch mode=%d failed\n",mode);
        }
    }
  else
    {
      syslog(LOG_INFO,"switch motion mode failed %d\n",mode);
    }
  syslog(LOG_INFO,"do_action_switch: EXECUTED \n");
  return result;
}

static int do_action_emergency(union motion_control_data_u data)
{
  bool emergency = data.emergency;
  
  syslog(LOG_INFO,"do_action_emergency: EXECUTED emergency = %d\n",emergency);
  return motor_quick_stop(emergency);

}

static int do_action_drv_error(union motion_control_data_u data)
{
  /* try stop motor driver */
  syslog(LOG_INFO,"do_action_drv_error: EXECUTED \n");
  return motor_quick_stop(true);

}

static int do_action_speed(union motion_control_data_u data)
{
  float vx = data.speed_cmd.vx;
  float vz = data.speed_cmd.vz;

  syslog(LOG_INFO,"do_action_speed: EXECUTED vx =%f, vz =%f \n",vx,vz);

  return motor_set_speed(vx, vz);
}

static int do_action_switch_done(union motion_control_data_u data)
{
  /* switch done call back*/
  if (NULL != g_motion_sm.switch_done_cb)
    {
      g_motion_sm.switch_done_cb(g_motion_sm.switch_cb_data);
    }
  syslog(LOG_INFO,"do_action_switch_done: EXECUTED \n");
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: get_motion_sm_state
 ****************************************************************************/

enum motion_sm_state_e get_motion_sm_state(void)
{
  enum motion_sm_state_e present_state;

	  /*lock sm */
  if (!acquire_motion_sm_lock())
    {
      release_motion_sm_lock();
      syslog(LOG_ERR,"acquire motion sm lock failed\n");
      return ERROR;
    }
  present_state = g_motion_sm.state;
  release_motion_sm_lock();
  return present_state;
}

int motion_sm_event(enum motion_sm_event_e event ,union motion_control_data_u data)
{
  enum motion_sm_state_e present_state;
  int result = ERROR;

  present_state = get_motion_sm_state();

  switch (present_state)
    {
      case ST_INACTIVE:
        {
          result = motion_sm_do_action(&statetrans_inactive[event],data);
          break;
        }
      case ST_SWITCHING:
        {
          result = motion_sm_do_action(&statetrans_switching[event],data);
          break;
        }
      case ST_SPEED:
        {
          result = motion_sm_do_action(&statetrans_speed[event],data);
          break;
        }
      case ST_POSITION_IDLE:
      case ST_POSITION_RUNNING:
        {
		      syslog(LOG_ERR, "motion state in state POSITION\n");
          break;
        }
      case ST_EMERGENCY:
        {
          result = motion_sm_do_action(&statetrans_emergency[event],data);
          break;
        }
      case ST_DRIVER_ERR:
        {
          syslog(LOG_ERR, "motion state in state ST_DRIVER_ERR\n");
          break;
        }
      default:
        {
          syslog(LOG_ERR, "motion state invalid state=%d\n", present_state);
          break;
        }
    }
//syslog(LOG_ERR, "motion state state=%d  done \n", present_state);
  return result;
}

void register_motion_switch_done_cb(motion_cb cb_fun, void *arg)
{
  g_motion_sm.switch_done_cb = cb_fun;
  g_motion_sm.switch_cb_data = arg;
}

void register_motion_odom_done_cb(motion_cb cb_fun, void *arg)
{
  return ;
}

int motion_sm_init(void)
{
  int status;

  status = pthread_mutex_init(&g_motion_sm.mutex, NULL);
  if (status != 0)
    {
      syslog(LOG_ERR,"ERROR pthread_mutex_init failed, status=%d\n",status);
      ASSERT(false);
    }

  /* init state as inactive */
  g_motion_sm.state = ST_INACTIVE;
  /* init motion threadpool*/
  g_motion_sm.threadpool = motion_thread_pool_init(1);
  return OK;
}
void motion_sm_join(void)
{
  motion_threads_join(g_motion_sm.threadpool);

}

/* Motion  threadpool */

#ifdef motion_MCB
#define motion_THREAD_PRIORITY SCHED_PRIORITY_DEFAULT
#define motion_THREAD_STACKSIZE (1024*4)
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* semaphore   */
struct  work_sem_s
{
  pthread_mutex_t mutex;
  pthread_cond_t cond;
  int value;
};

/* motion work */
struct motion_work_s
{
  struct motion_work_s *previous;
  motion_work  work_fun;
  struct motion_args_s args;
};

/* motion work queue */
struct motion_workqueue_s
{
  pthread_mutex_t queue_mutex;
  struct motion_work_s *work_front;
  struct motion_work_s  *work_rear;
  struct work_sem_s *work_sem;
  int len;
};

/* motion thread */
struct motion_thread_s
{
  int id;
  pthread_t pthread;
  struct motion_thread_pool_s *motion_tp;
};

/* motion thread pool */
struct motion_thread_pool_s
{
  struct motion_thread_s ** threads;	/* thread list */
  volatile int num_threads_alive;
  volatile int num_threads_working;
  pthread_mutex_t  thread_count_lock;
  pthread_cond_t  threads_all_idle;
  struct motion_workqueue_s workqueue;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  thread_init(struct motion_thread_pool_s * motion_tp, struct motion_thread_s **threads, int id);
static void* thread_run(struct motion_thread_s *motion_thread);
static void  thread_hold(int sig_id);
static void  thread_destroy(struct motion_thread_s *motion_thread);

static int   workqueue_init(struct motion_workqueue_s *workqueue);
static void  workqueue_clear(struct motion_workqueue_s *workqueue);
static void  workqueue_push(struct motion_workqueue_s *workqueue, struct motion_work_s *work);
static struct motion_work_s *workqueue_pull(struct motion_workqueue_s *workqueue);
static void  workqueue_destroy(struct motion_workqueue_s *workqueue_p);


static void  work_sem_init(struct work_sem_s *sem, int value);
static void  work_sem_reset(struct work_sem_s *sem);
static void  work_sem_post(struct work_sem_s *sem);
static void  work_sem_post_all(struct work_sem_s *sem);
static void  work_sem_wait(struct work_sem_s *sem);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static volatile int g_threads_keepalive;
static volatile int g_threads_on_hold;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int thread_init(struct motion_thread_pool_s * motion_tp, struct motion_thread_s **threads, int id)
{
	*threads = (struct motion_thread_s*)malloc(sizeof(struct motion_thread_s));
	if (*threads == NULL){
		printf("thread_init(): Could not allocate memory for thread\n");
		return -1;
	}

	(*threads)->motion_tp = motion_tp;
	(*threads)->id       = id;
printf("thread_init: start  thread_init\n");
#ifdef motion_MCB
	pthread_attr_t attr;
	struct sched_param sparam;
	int status;

	status = pthread_attr_init(&attr);
	if (status != 0)
    {
      printf("thread_init: ERROR pthread_attr_init failed, status=%d\n",
             status);
      ASSERT(false);
    }

	status = pthread_attr_setstacksize(&attr, motion_THREAD_STACKSIZE);
	if (status != 0)
    {
      printf("thread_init: "
             "ERROR pthread_attr_setstacksize failed, status=%d\n",
             status);
      ASSERT(false);
    }

	sparam.sched_priority = motion_THREAD_PRIORITY;
	status = pthread_attr_setschedparam(&attr, &sparam);
	if (status != 0)
    {
      printf("thread_init: "
             "ERROR pthread_attr_setschedparam failed, status=%d\n",
             status);
      ASSERT(false);
    }
    printf("pthread_create: start  thread_run\n");
	status = pthread_create(&(*threads)->pthread, &attr, (void * (*)(void *)) thread_run, (*threads));
	if (status != 0)
    {
      printf("thread_init: "
             "ERROR pthread_create failed, status=%d\n", status);
      ASSERT(false);
    }
#else
	pthread_create(&(*threads)->pthread, NULL, (void * (*)(void *)) thread_run, (*threads));
#endif

	//pthread_detach((*threads)->pthread);
	return 0;
}

/* Sets the calling thread on hold */
static void thread_hold(int sig_id) {
    (void)sig_id;
	g_threads_on_hold = 1;
	while (g_threads_on_hold){
		sleep(1);
	}
}

static void* thread_run(struct motion_thread_s *motion_thread)
{
	struct motion_thread_pool_s *motion_tp = motion_thread->motion_tp;
	struct sigaction act;

	sigemptyset(&act.sa_mask);
	act.sa_flags = SA_ONSTACK;
	act.sa_handler = thread_hold;
	if (sigaction(SIGUSR1, &act, NULL) == -1)
	{
		printf("thread_run(): cannot handle SIGUSR1");
	}

	pthread_mutex_lock(&motion_tp->thread_count_lock);
	motion_tp->num_threads_alive += 1;
	pthread_mutex_unlock(&motion_tp->thread_count_lock);
printf("thread_run(): start run while");
	while(g_threads_keepalive)
	{
		work_sem_wait(motion_tp->workqueue.work_sem);
		if (g_threads_keepalive)
		{
			pthread_mutex_lock(&motion_tp->thread_count_lock);
			motion_tp->num_threads_working++;
			pthread_mutex_unlock(&motion_tp->thread_count_lock);

			/* execute motion function */
			motion_work work_fun;
			struct motion_args_s *args;


			struct motion_work_s *work_p = workqueue_pull(&motion_tp->workqueue);
			if (work_p) {
				work_fun = work_p->work_fun;
				args  = &work_p->args;
				work_fun(*args);
				free(work_p);
			}

			pthread_mutex_lock(&motion_tp->thread_count_lock);
			motion_tp->num_threads_working--;
			if (!motion_tp->num_threads_working) {
				pthread_cond_signal(&motion_tp->threads_all_idle);
			}
			pthread_mutex_unlock(&motion_tp->thread_count_lock);

		}
	}
	pthread_mutex_lock(&motion_tp->thread_count_lock);
	motion_tp->num_threads_alive --;
	pthread_mutex_unlock(&motion_tp->thread_count_lock);

	return NULL;
}

static void thread_destroy(struct motion_thread_s *motion_thread)
{
	free(motion_thread);
}

static int workqueue_init(struct motion_workqueue_s *workqueue)
{
	workqueue->len = 0;
	workqueue->work_front = NULL;
	workqueue->work_rear  = NULL;

	workqueue->work_sem = (struct work_sem_s*)malloc(sizeof(struct work_sem_s));
	if (workqueue->work_sem == NULL){
		return -1;
	}

	pthread_mutex_init(&(workqueue->queue_mutex), NULL);
	work_sem_init(workqueue->work_sem, 0);

	return 0;
}

static void workqueue_clear(struct motion_workqueue_s *workqueue)
{

	while(workqueue->len){
		free(workqueue_pull(workqueue));
	}

	workqueue->work_front = NULL;
	workqueue->work_rear  = NULL;
	work_sem_reset(workqueue->work_sem);
	workqueue->len = 0;
}

static void  workqueue_push(struct motion_workqueue_s *workqueue, struct motion_work_s *work)
{

	pthread_mutex_lock(&workqueue->queue_mutex);
	work->previous = NULL;

	switch(workqueue->len)
	  {

		case 0:
		  {
			workqueue->work_front = work;
			workqueue->work_rear  = work;
			break;
		  }
					

		default:
		  {
			workqueue->work_rear->previous = work;
			workqueue->work_rear = work;
		  }

	  }
	workqueue->len++;

	work_sem_post(workqueue->work_sem);
	pthread_mutex_unlock(&workqueue->queue_mutex);
}

static struct motion_work_s *workqueue_pull(struct motion_workqueue_s *workqueue)
{

	pthread_mutex_lock(&workqueue->queue_mutex);
	struct motion_work_s *work_p = workqueue->work_front;

	switch(workqueue->len)
	  {
		case 0:
		  {
			break;
		  }
		case 1:
		  {
			workqueue->work_front = NULL;
			workqueue->work_rear  = NULL;
			workqueue->len = 0;
			break;
		  }
		default:
		  {
			workqueue->work_front = work_p->previous;
			workqueue->len--;
			work_sem_post(workqueue->work_sem);
		  }
	  }

	pthread_mutex_unlock(&workqueue->queue_mutex);
	return work_p;
}

static void workqueue_destroy(struct motion_workqueue_s *workqueue)
{
  workqueue_clear(workqueue);
  free(workqueue->work_sem);
}

static void work_sem_init(struct work_sem_s *sem, int value)
{
	if (value < 0 || value > 1)
	{
		printf("work_sem_init(): value invalid\n");
		exit(1);
	}
	pthread_mutex_init(&(sem->mutex), NULL);
	pthread_cond_init(&(sem->cond), NULL);
	sem->value = value;
}

static void work_sem_reset(struct work_sem_s *sem)
{
	pthread_mutex_destroy(&(sem->mutex));
	pthread_cond_destroy(&(sem->cond));
	work_sem_init(sem, 0);
}

static void work_sem_post(struct work_sem_s *sem)
{
	pthread_mutex_lock(&sem->mutex);
	sem->value = 1;
	pthread_cond_signal(&sem->cond);
	pthread_mutex_unlock(&sem->mutex);
}

static void work_sem_post_all(struct work_sem_s *sem)
{
	pthread_mutex_lock(&sem->mutex);
	sem->value = 1;
	pthread_cond_broadcast(&sem->cond);
	pthread_mutex_unlock(&sem->mutex);
}

static void work_sem_wait(struct work_sem_s *sem)
{
	pthread_mutex_lock(&sem->mutex);
	while (sem->value != 1) {
		pthread_cond_wait(&sem->cond, &sem->mutex);
	}
	sem->value = 0;
	pthread_mutex_unlock(&sem->mutex);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Initialise thread pool */
struct motion_thread_pool_s * motion_thread_pool_init(int num)
{
  int n;

  g_threads_on_hold   = 0;
  g_threads_keepalive = 1;

  if (num < 0)
    {
      num = 0;
	}

  struct motion_thread_pool_s * thpool;
  thpool = (struct motion_thread_pool_s *)malloc(sizeof(struct motion_thread_pool_s));
  if (thpool == NULL)
    {
	  printf("motion_thread_pool_init(): Could not allocate memory for thread pool\n");
	  return NULL;
	}
  thpool->num_threads_alive   = 0;
  thpool->num_threads_working = 0;

  /* Initialise the work queue */
  if (workqueue_init(&thpool->workqueue) == -1)
    {
	  printf("motion_thread_pool_init(): Could not allocate memory for work queue\n");
	  free(thpool);
	  return NULL;
    }

  /* Make threads in pool */
  thpool->threads = (struct motion_thread_s**)malloc(num * sizeof(struct motion_thread_s *));
  if (thpool->threads == NULL)
    {
	  printf("motion_thread_pool_init(): Could not allocate memory for threads\n");
	  workqueue_destroy(&thpool->workqueue);
	  free(thpool);
	  return NULL;
	}

  pthread_mutex_init(&(thpool->thread_count_lock), NULL);
  pthread_cond_init(&thpool->threads_all_idle, NULL);

  /* Thread init */
  for (n=0; n<num; n++)
    {
	  thread_init(thpool, &thpool->threads[n], n);
    }

  /* Wait for threads to initialize */
  while (thpool->num_threads_alive != num) {
	sleep(1);
	printf("wait thread_alive \n");
  }

  return thpool;
}

/* Add work to the thread pool */
int motion_threadpool_add_work(struct motion_thread_pool_s * thpool, motion_work work_fun, struct motion_args_s args)
{
  struct motion_work_s* newwork;

  newwork=(struct motion_work_s *)malloc(sizeof(struct motion_work_s));
  if (newwork==NULL)
    {
	  printf("motion_threadpool_add_work(): Could not allocate memory for new work\n");
	  return -1;
	}

  /* add function and argument */
  newwork->work_fun=work_fun;
  memcpy(&newwork->args, &args, sizeof(struct motion_args_s));
  /* add work to queue */
  workqueue_push(&thpool->workqueue, newwork);

  return 0;
}

void motion_threadpool_wait(struct motion_thread_pool_s * thpool)
{
  pthread_mutex_lock(&thpool->thread_count_lock);
  while (thpool->workqueue.len || thpool->num_threads_working)
    {
	  pthread_cond_wait(&thpool->threads_all_idle, &thpool->thread_count_lock);
	}
  pthread_mutex_unlock(&thpool->thread_count_lock);
}

/* Destroy the threadpool */
void motion_threadpool_destroy(struct motion_thread_pool_s * thpool)
{
	/* No need to destroy if it's NULL */
	if (thpool == NULL) return ;

	volatile int threads_total = thpool->num_threads_alive;

	/* End each thread 's infinite loop */
	g_threads_keepalive = 0;

	/* Give one second to kill idle threads */
	double TIMEOUT = 1.0;
	time_t start, end;
	double tpassed = 0.0;
	time (&start);
	while (tpassed < TIMEOUT && thpool->num_threads_alive){
		work_sem_post_all(thpool->workqueue.work_sem);
		time (&end);
		tpassed = difftime(end,start);
	}

	/* Poll remaining threads */
	while (thpool->num_threads_alive){
		work_sem_post_all(thpool->workqueue.work_sem);
		sleep(2);
	}

	/* work queue cleanup */
	workqueue_destroy(&thpool->workqueue);
	/* Deallocs */
	int n;
	for (n=0; n < threads_total; n++){
		thread_destroy(thpool->threads[n]);
	}
	free(thpool->threads);
	free(thpool);
}

void motion_threads_join(struct motion_thread_pool_s * thpool)
{

	pthread_t *threads;
	int i;
	int thread_num = thpool->num_threads_alive;

	threads = (pthread_t *)malloc(thread_num*sizeof(pthread_t));

	for (i =0; i < thread_num; i++)
	{
		threads[i] = thpool->threads[0]->pthread;
	}

	for (i =0; i < thread_num; i++)
	{
		printf(" motion_threads_join =%d \n",i);
		pthread_join(threads[i], NULL);
		sleep(1);
	}
	free(threads);
}