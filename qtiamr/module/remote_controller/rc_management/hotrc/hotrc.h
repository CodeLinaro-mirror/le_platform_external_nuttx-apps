/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#ifndef __APP_QTIAMR_RC_HAL_ZL_H
#define __APP_QTIAMR_RC_HAL_ZL_H



int init_rc_hal_data(void);
int set_max_speed(struct speed_req_s *speed);
int get_speed(struct speed_req_s *speed);
#endif
