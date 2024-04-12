/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/
#ifndef _MODULE_AVOIDANCE_H
#define _MODULE_AVOIDANCE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/list.h>

/****************************************************************************
 * Public types
 ****************************************************************************/
struct avoid_client
{
  char *           name;
  int              trigger;
  struct list_node node;
  uint16_t         thres_front;
  uint16_t         thres_side;
  uint16_t         thres_bottom;
  void (*cb)(uint8_t addr, uint16_t dist, bool enter);
} __attribute__((aligned(4)));

/****************************************************************************
 * Public Function prototypes
 ****************************************************************************/
bool              is_ultra_enabled(void);
int               get_ultra_num(void);
bool              is_avoidance_inited(void);
struct list_node *get_avoid_client_list(void);
void              register_ultra_client(struct avoid_client *client);
void              unregister_ultra_client(struct avoid_client *client);

int avoidance_main(int argc, char *argv[]);

#endif
