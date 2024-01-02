/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#include <syslog.h>
#include <signal.h>
#include <assert.h>
#include <pthread.h>
#include <stdio.h>
#include <errno.h>


#include "charger_sm.h"
#include "charger_qrc_common.h"

static bool g_sm_task_started = false;
static charger_dev_t *charger_dev_p = &g_charger_dev;



uint32_t get_curr_state(void){

}

static void set_curr_state(enum chr_sm_st_e state){

}

void sm_state_notify(){


}

void charging_currnet_notify(float current){


}

void sm_motion_speed_notify(float vx, float vz){
    if(charger_dev_p->initialized)
        charger_dev_p->nfy_ops.motion_speed_nofity(vx, vz);
}


int state_available(int state){
	

}

int set_charger_state(enum chr_sm_st_e state){
	


}

void charging_stop_move_forward(){


}

void start_searching(){

}

void start_controlling(){


}

void start_force_charging(){
	
}

void start_charging(){

}

void charging_done(){

}




void sm_init(void){

}



static int sm_task(int argc, FAR char *argv[]){

}




int start_sm_task(void)
{

}

  
