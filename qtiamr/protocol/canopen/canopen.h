
/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#ifndef __INCLUDE_QTIAMR_CANOPEN_H	
#define __INCLUDE_QTIAMR_CANOPEN_H 

#include <stdio.h>
#include <stdint.h>
#include <nuttx/config.h>
#include <nuttx/fs/fs.h>

#define COB_ID_SSDO  (0x601)  //SDO server
#define COB_ID_CSDO  (0x581)  //SDO client
#define CAN_MSDGLC   (8)      //CAN message length

enum sdo_cw    //SDO command word
{
    M2S_1B  = 0x2f,  //M->S request, set 1 bit
    M2S_2B  = 0x2b,  //M->S request, set 2 bits
    M2S_3B  = 0x27,  //M->S request, set 3 bits
    M2S_4B  = 0x23,  //M->S request, set 4 bits
    S2M_ACK = 0x60,  //S-M acknowledge, set feedback
    M2S_0B  = 0x40,  //M->S request, read 0 bit
    
    S2M_1B  = 0x4f,  //S->M response, read feedback 1 bit
    S2M_2B  = 0x4b,  //S->M response, read feedback 2 bits
    S2M_3B  = 0x47,  //S->M response, read feedback 3 bits
    S2M_4B  = 0x43,  //S->M response, read feedback 4 bits
       
    S2M_ERR  = 0x80,  //S->M response, error feedback 4 bits
};

enum sdo_obj_sub      //SDO object sub-index
{
    SUBINDEX_0 = 0x00,   //Object sub-index 0
    SUBINDEX_1 = 0x01,   //Object sub-index 1
    SUBINDEX_2 = 0x02,   //Object sub-index 2
    SUBINDEX_3 = 0x03,   //Object sub-index 3
};

enum fac_def_param      //Factory-defined parameters: 0x2003 ~ 0x2030
{
    FAC_DEFIN = 0x20,    //Factory-defined parameters high bit

    MOT_MVEL  = 0x08,    //Motor max velocity
    SET_BAUD  = 0x0b,    //Set CAN baudrate
    SET_POLE  = 0x0c,    //Set motor pole number
    SET_ENCOD = 0x0e,    //Set encoder line number
    SET_ASYN  = 0x0f,    //Syn-Asyn control flag
    TO_EEPROM = 0x10,    //Save parameters to EEPROM
    HALL_ANGL = 0x11,    //Angle between hall sensor and motor
    OV_COEFFI = 0x12,    //Overload coefficient
    TEMP_PRO  = 0x13,    //Motor temperature protection threshold
    RATED_CUR = 0x14,    //Rated current
    MAX_CUR   = 0x15,    //Max current
    OV_PROTEC = 0x16,    //Driver overload protection time
    ENCO_THR  = 0x17,    //Encoder out-of-tolerance alarm threshold
    CURR_KP   = 0x19,    //Current Kp
    CURR_KI   = 0x1a,    //Current Ki
    FW_SMOOTH = 0x1b,    //Feed forward output smoothing coefficient
    VEL_KP    = 0x1d,    //Velocity Kp
    VEL_KI    = 0x1e,    //Velocity Ki
    MOTOR_TEM = 0x32,    //Motor temperature
    BUS_VOL   = 0x35,    //Motor bus voltage
};

enum cia_402_vm_od    //cia 402 sub-protocol object dictionary, 0x603f ~ 0x60ff
{
    CIA402_PRO = 0x60,   //cia 402 sub-protocol high bit

    DRIVER_FAU = 0x3f,   //Driver fault information
    DRIVER_CW  = 0x40,   //Driver start stop control instruction
    DRIVER_SW  = 0x41,   //Driver start status word
    DRIVER_OM  = 0x60,   //Driver operating mode
    M_CUR_VEL  = 0x6c,   //Motor current velocity
    MOTER_CUR  = 0x77,   //Motor current feedback 
    PMAX_VEL   = 0x81,   //Max velocity in position mode
    S_ACC_TIME = 0x83,   //S acceleration time
    S_DEC_TIME = 0x84,   //S deceleration time
    STOP_TIME  = 0x85,   //Emergency stop deceleration time
    TAR_VEL    = 0xff,   //Target velocity     
};

int canopen_send(int fd, char* buffer, size_t msgsize);
int canopen_receive(int fd, char* buffer, size_t len);

#endif