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
 struct avoid_client {
	char* name;
	struct list_node node;
	uint16_t thres_front;
	uint16_t thres_side;
	uint16_t thres_bottom;
	void (*cb)(uint8_t addr, uint16_t dist, bool enter);
 }__attribute__((aligned(4)));


/****************************************************************************
 * Public data
 ****************************************************************************/
extern struct list_node g_avoid_client;
extern int avoidance_inited;

 
/****************************************************************************
 * Public Function prototypes
 ****************************************************************************/
void register_ultra_client(struct avoid_client * client);
void unregister_ultra_client(struct avoid_client * client);

int avoidance_main(int argc, char *argv[]);


 #endif

