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

QTIARM_DIR := $(APPDIR)/qtiamr
PROGNAME  = qtiamr
PRIORITY  = SCHED_PRIORITY_DEFAULT
STACKSIZE = $(CONFIG_DEFAULT_TASK_STACKSIZE)
MODULE    = $(CONFIG_APP_QTIAMR)

DEPPATH += --dep-path app

C_INCLUDES +=  \
    -I$(QTIARM_DIR) \
    -I$(QTIARM_DIR)/app \
    -I$(QTIARM_DIR)/app/* \
    -I$(QTIARM_DIR)/app/motion_odom \
    -I$(QTIARM_DIR)/app/robot_controller \
    -I$(QTIARM_DIR)/app/time_sync \
    -I$(QTIARM_DIR)/libqrc/include/app_msg \
    -I$(QTIARM_DIR)/libqrc/include/module \
    -I$(QTIARM_DIR)/libqrc/protocol/qrc \
    -I$(QTIARM_DIR)/libqrc/protocol/qrc/tinyframe \
    -I$(QTIARM_DIR)/libqrc/protocol/tinyframe \
    -I$(QTIARM_DIR)/module/qrc_msg_management/ \
    -I$(QTIARM_DIR)/module/motion/ \
    -I$(QTIARM_DIR)/module/motion/kinematic \
    -I$(QTIARM_DIR)/module/motion/kinematic/diff_car \
    -I$(QTIARM_DIR)/module/motion/motor_management \
    -I$(QTIARM_DIR)/module/motion/motor_management/zlac8015d \
    -I$(QTIARM_DIR)/protocol/canopen/ \


MAINSRC = $(QTIARM_DIR)/app/main.c

# app source
CSRCS += $(QTIARM_DIR)/app/configuration/robot_configuration.c
CSRCS += $(QTIARM_DIR)/app/motion_odom/motion_odom.c
CSRCS += $(QTIARM_DIR)/app/robot_controller/client.c
CSRCS += $(QTIARM_DIR)/app/robot_controller/robot_controller.c
CSRCS += $(QTIARM_DIR)/app/time_sync/time_sync.c
CSRCS += $(QTIARM_DIR)/app/imu/imu.c

# protocol/libqrc source
CSRCS += $(QTIARM_DIR)/protocol/canopen/canopen.c

CSRCS += $(QTIARM_DIR)/libqrc/protocol/qrc/qrc.c
CSRCS += $(QTIARM_DIR)/libqrc/protocol/qrc/qrc_threadpool.c
CSRCS += $(QTIARM_DIR)/libqrc/protocol/tinyframe/TinyFrame.c
CSRCS += $(QTIARM_DIR)/libqrc/protocol/qrc_msg_management.c

# module source
CSRCS += $(QTIARM_DIR)/module/motion/motion_management.c
CSRCS += $(QTIARM_DIR)/module/motion/control_sm.c
CSRCS += $(QTIARM_DIR)/module/motion/motion_sm.c
#CSRCS += $(QTIARM_DIR)/module/motion/motion_threadpool.c
CSRCS += $(QTIARM_DIR)/module/motion/kinematic/kinematics.c
CSRCS += $(QTIARM_DIR)/module/motion/kinematic/diff_car/diff_car.c
CSRCS += $(QTIARM_DIR)/module/motion/motor_management/motor_management.c
CSRCS += $(QTIARM_DIR)/module/motion/motor_management/zlac8015d/8015d.c


# add define
CFLAGS += -DQRC_MCB

CFLAGS += $(C_INCLUDES)

CFLAGS += -Wl,-u_printf_float,-o0

include $(APPDIR)/Application.mk
