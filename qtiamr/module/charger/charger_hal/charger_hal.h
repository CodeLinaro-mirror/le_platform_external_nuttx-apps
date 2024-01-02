/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#ifndef __APP_CHARGER_HAL_H
#define __APP_CHARGER_HAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/



/********************************************************************************
 * Pre-processor Definitions
 ********************************************************************************/

typedef struct chr_dev_table_s{
	char *name;
	bool (*drv_init_func)();
}chr_dev_tab_t;



bool get_voltage_hal(charger_dev_t *charger_dev, float *voltage);
bool get_current_hal(charger_dev_t *charger_dev, float *current);
bool get_wheel_speed_hal(charger_dev_t *charger_dev, float *vx, float *vz);
bool get_pile_signal_stats(charger_dev_t *charger_dev, bool *stats);
bool charger_if_charging(charger_dev_t *charger_dev, bool *stats);



#endif /* __APP_CHARGER_HAL_H */

