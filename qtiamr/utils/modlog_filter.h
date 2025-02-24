/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#ifndef __MODLOG_FILTER_H
#define __MODLOG_FILTER_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LOG_MOTION     0  /* motion module log mask */
#define LOG_CHARGER    1  /* charger module log mask */
#define LOG_ULEMERG    2  /* ultrasound emergency stop module log mask */
#define LOG_RMCTL      3  /* remote controller module log mask */
#define LOG_RESVE1     4  /* reserved */
#define LOG_RESVE2     5  /* reserved */
#define LOG_RESVE3     6  /* reserved */
#define LOG_RESVE4     7  /* reserved */


#define LOG_MODULE_MASK(p)   (1 << (p))
#define LOG_MODULE_UPTO(p)   ((1 << ((p)+1)) - 1)
#define LOG_ALL_DISABLED     0x00
/****************************************************************************
 * Public Types
 ****************************************************************************/

extern uint8_t g_module_mask;


/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

extern int modulemask(int mask);
extern int moduleunmask(int mask);

#define modlog_dbg(mod, fmt, args...)                 \
  do {                                                \
         if (g_module_mask & LOG_MODULE_MASK(mod)) {  \
             syslog(LOG_DEBUG, fmt, ##args);          \
         }                                            \
  } while (0)

#define modlog_info(mod, fmt, args...)                \
  do {                                                \
         if (g_module_mask & LOG_MODULE_MASK(mod)) {  \
             syslog(LOG_INFO, fmt, ##args);           \
         }                                            \
  } while (0)

#endif
