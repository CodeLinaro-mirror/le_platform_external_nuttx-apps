/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#ifndef __INCLUDE_AMR_SIGNAL_H
#define __INCLUDE_AMR_SIGNAL_H

#include <nuttx/signal.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#ifndef CONFIG_CHARGER_SM_SIGNO
#  define CONFIG_CHARGER_SM_SIGNO SIGUSR1
#endif

#ifndef CONFIG_EXAMPLE1_TASK_SIGNO
#  define CONFIG_EXAMPLE1_TASK_SIGNO SIGUSR1
#endif

#ifndef CONFIG_EXAMPLE2_TASK_SIGNO
#  define CONFIG_EXAMPLE2_TASK_SIGNO SIGUSR2
#endif

#ifndef TRUE
#  define TRUE 1
#endif

#ifndef FALSE
#  define FALSE 0
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/
struct amr_signal_s
{
  pid_t            pid;   /* The task to be signaled */
  struct sigevent  event; /* Signal number and value */
  struct sigwork_s work;  /* Signal work */
  bool             init;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
int amr_signal_init(struct amr_signal_s *amr_signal, pid_t pid, uint8_t signo);
int amr_signal_wait(struct amr_signal_s *amr_signal, struct siginfo *info, FAR const struct timespec *timeout);
int amr_signal_send(struct amr_signal_s *amr_signal, union sigval sigev_value);

#endif /* __INCLUDE_AMR_SIGNAL_H */