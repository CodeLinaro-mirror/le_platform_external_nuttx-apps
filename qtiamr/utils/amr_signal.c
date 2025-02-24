/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#include <syslog.h>
#include <signal.h>
#include <assert.h>

#include "amr_signal.h"

/****************************************************************************
 * Privite Functions
 ****************************************************************************/

/****************************************************************************
 * Name: amr_signo_avail
 *
 * Description:
 *   check if the signal number is available.
 *
 * Input Parameters:
 *   signo - signal number that to be checked.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated (ERROR) value is returned on failure.
 *
 ****************************************************************************/

int amr_signo_avail(uint8_t signo)
{
  if (signo != SIGUSR1 && signo != SIGUSR2)
    {
      return ERROR;
    }
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: amr_signal_init
 *
 * Description:
 *   Initialize amr signal.
 *
 * Input Parameters:
 *   amr_signal  - amr signal struct that include the important signal information.
 *   pid   - The task/thread ID a the client thread to be signaled.
 *   signo - signal number that to be used for the task.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated (ERROR) value is returned on failure.
 *
 ****************************************************************************/

int amr_signal_init(struct amr_signal_s *amr_signal, pid_t pid, uint8_t signo)
{
  int ret = OK;
  DEBUGASSERT(amr_signal);

  if (amr_signal->init == TRUE || amr_signal->event.sigev_signo == signo)
    {
      syslog(LOG_ERR, "amr_signal_init: The signal(%d) has been initialized.\n", signo);
      ret = ERROR;
      goto errout;
    }
  if (amr_signo_avail(signo) != OK)
    {
      syslog(LOG_ERR, "amr_signal_send_int: The signal number is not valid.\n");
      ret = ERROR;
      goto errout;
    }
  amr_signal->pid                = pid;
  amr_signal->event.sigev_signo  = signo;
  amr_signal->event.sigev_notify = SIGEV_SIGNAL;
  amr_signal->init               = TRUE;
  syslog(LOG_INFO, "amr_signal_init:amr signal init done! addr:%p,pid = %d, signo = %d, sigev_notify = %d\n", amr_signal,
         amr_signal->pid, amr_signal->event.sigev_signo, amr_signal->event.sigev_notify);
errout:
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: amr_signal_wait
 *
 * Description:
 *   wait for signal.
 *
 * Input Parameters:
 *   amr_signal  - amr signal struct that include the important signal information.
 *   info   - The returned value (may be NULL).
 *   timeout - The amount of time to wait (may be NULL)
 *  
 * Returned Value:
 *   return signal number when success.  A negated (ERROR) value is returned on failure.
 *
 ****************************************************************************/

int amr_signal_wait(struct amr_signal_s *amr_signal, struct siginfo *info, FAR const struct timespec *timeout)
{
  sigset_t set;
  int      ret;

  DEBUGASSERT(amr_signal);
  if (amr_signal->init != TRUE)
    {
      syslog(LOG_ERR, "amr_signal_wait: The signal hasn't been initialized.\n");
      ret = ERROR;
      goto errout;
    }
  if (amr_signo_avail(amr_signal->event.sigev_signo) != OK)
    {
      syslog(LOG_ERR, "amr_signal_send_int: The signal number is not valid.\n");
      ret = ERROR;
      goto errout;
    }
  sigemptyset(&set);
  sigaddset(&set, amr_signal->event.sigev_signo);
  if (timeout == NULL)
    ret = sigwaitinfo(&set, info);
  else
    ret = sigtimedwait(&set, info, timeout);
  if (ret < 0)
    {
      int errcode = errno;
      syslog(LOG_ERR, "amr_signal: ERROR: sigwaitinfo() failed: %d\n", errcode);
    }

errout:
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: amr_signal_send
 *
 * Description:
 *   wait for signal.
 *
 * Input Parameters:
 *   amr_signal  - amr signal struct that include the important signal information.
 *   sigev_value   - the value(may be INT or Pointer) that to be sent. Zero value if ignore it.
 *  
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated (ERROR) value is returned on failure.
 *
 ****************************************************************************/

int amr_signal_send(struct amr_signal_s *amr_signal, union sigval sigev_value)
{
  int ret;
  DEBUGASSERT(amr_signal);
  if (amr_signal->init != TRUE)
    {
      syslog(LOG_ERR, "amr_signal_send_int: The signal hasn't been initialized.\n");
      ret = ERROR;
      goto errout;
    }
  if (amr_signo_avail(amr_signal->event.sigev_signo) != OK)
    {
      syslog(LOG_ERR, "amr_signal_send_int: The signal number is not valid.\n");
      ret = ERROR;
      goto errout;
    }
  amr_signal->event.sigev_value = sigev_value;
  ret                           = nxsig_notification(amr_signal->pid, &amr_signal->event, SI_QUEUE, &amr_signal->work);
  if (ret < 0)
    {
      syslog(LOG_WARNING, "ERROR: nxsig_notification event ID=%d failed: %d\n", (int)amr_signal->event.sigev_signo, ret);
      ret = ERROR;
    }
errout:
  return ret;
}
