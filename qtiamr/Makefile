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

DEPPATH += --dep-path app

C_INCLUDES +=  \
    -I$(CAR_DIR) \
    -I$(CAR_DIR)/app \
    -I$(CAR_DIR)/app/* \
    -I$(CAR_DIR)/app/motion_odom \
    -I$(CAR_DIR)/app/robot_controller \
    -I$(CAR_DIR)/app/time_sync \
    -I$(CAR_DIR)/libqrc/include/app_msg \
    -I$(CAR_DIR)/libqrc/include/module \
    -I$(CAR_DIR)/libqrc/protocol/qrc \
    -I$(CAR_DIR)/libqrc/protocol/qrc/tinyframe \
    -I$(CAR_DIR)/libqrc/protocol/tinyframe \
    -I$(CAR_DIR)/module/qrc_msg_management/ \
    -I$(CAR_DIR)/module/motion/ \
    -I$(CAR_DIR)/module/motion/kinematic \
    -I$(CAR_DIR)/module/motion/kinematic/diff_car \
    -I$(CAR_DIR)/module/motion/motor_management \
    -I$(CAR_DIR)/module/motion/motor_management/zlac8015d \
    -I$(CAR_DIR)/protocol/canopen/ \


MAINSRC = $(CAR_DIR)/app/main.c

# app source
CSRCS += $(CAR_DIR)/app/configuration/robot_configuration.c
CSRCS += $(CAR_DIR)/app/motion_odom/motion_odom.c
CSRCS += $(CAR_DIR)/app/robot_controller/client.c
CSRCS += $(CAR_DIR)/app/robot_controller/robot_controller.c
CSRCS += $(CAR_DIR)/app/time_sync/time_sync.c

# protocol/libqrc source
CSRCS += $(CAR_DIR)/protocol/canopen/canopen.c

CSRCS += $(CAR_DIR)/libqrc/protocol/qrc/qrc.c
#CSRCS += $(CAR_DIR)/libqrc/protocol/qrc/tinyframe/TF_Config.c
CSRCS += $(CAR_DIR)/libqrc/protocol/tinyframe/TinyFrame.c
CSRCS += $(CAR_DIR)/libqrc/protocol/qrc_msg_management.c

# module source
CSRCS += $(CAR_DIR)/module/motion/motion_management.c
CSRCS += $(CAR_DIR)/module/motion/control_sm.c
CSRCS += $(CAR_DIR)/module/motion/motion_sm.c
CSRCS += $(CAR_DIR)/module/motion/kinematic/kinematics.c
CSRCS += $(CAR_DIR)/module/motion/kinematic/diff_car/diff_car.c
CSRCS += $(CAR_DIR)/module/motion/motor_management/motor_management.c
CSRCS += $(CAR_DIR)/module/motion/motor_management/zlac8015d/8015d.c


# add define
CFLAGS += -DQRC_MCB

CFLAGS += $(C_INCLUDES)

CFLAGS += -Wl,-u_printf_float

include $(APPDIR)/Application.mk
