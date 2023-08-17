############################################################################
# apps/examples/gpio/Makefile
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
# 
# Changes from Qualcomm Innovation Center are provided under the following license:
# Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear
#
############################################################################

include $(APPDIR)/Make.defs

CAR_DIR := $(APPDIR)/qtiamr
PROGNAME  = qtiamr
PRIORITY  = SCHED_PRIORITY_DEFAULT
STACKSIZE = $(CONFIG_DEFAULT_TASK_STACKSIZE)
MODULE    = $(CONFIG_APP_QTIAMR)

DEPPATH += --dep-path motion
DEPPATH += --dep-path ultrasound
DEPPATH += --dep-path imu
DEPPATH += --dep-path roscom
DEPPATH += --dep-path rc
DEPPATH += --dep-path auto_charging
DEPPATH += --dep-path misc

C_INCLUDES +=  \
    -I$(CAR_DIR) \
    -I$(CAR_DIR)/motion \
    -I$(CAR_DIR)/ultrasound \
    -I$(CAR_DIR)/imu \
    -I$(CAR_DIR)/roscom \
    -I$(CAR_DIR)/rc \
	-I$(CAR_DIR)/auto_charging \
	-I$(CAR_DIR)/misc \

MAINSRC = amr_main.c
CSRCS += $(CAR_DIR)/motion/kinematics.c
CSRCS += $(CAR_DIR)/motion/motion_task.c
CSRCS += $(CAR_DIR)/motion/motor_driver.c
CSRCS += $(CAR_DIR)/motion/canopen.c

CSRCS += $(CAR_DIR)/ultrasound/ultrasound.c

CSRCS += $(CAR_DIR)/roscom/ros_com.c

CSRCS += $(CAR_DIR)/imu/imu.c

CSRCS += $(CAR_DIR)/rc/rc.c
CSRCS += $(CAR_DIR)/auto_charging/auto_charging.c
CSRCS += $(CAR_DIR)/misc/amr_adc.c

CFLAGS += $(C_INCLUDES)

CFLAGS += -Wl,-u_printf_float

include $(APPDIR)/Application.mk
