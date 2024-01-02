/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#ifndef __APP_CHARGER_MANAGEMENT_H
#define __APP_CHARGER_MANAGEMENT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include "charger_device.h"
#include "charger_sm.h"

/********************************************************************************
 * Pre-processor Definitions
 ********************************************************************************/


bool notification_ops_register_intf(const chr_notify_ops_t *ops);
float get_battery_voltage_intf(void);
float get_charging_current_intf(void);
bool get_charger_pile_signal_stats_intf(void);
uint8_t get_charger_sm_stats_intf(void);




#endif /* __APP_CHARGER_MANAGEMENT_H */
