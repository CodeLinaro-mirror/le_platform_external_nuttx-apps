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
    -I$(CAR_DIR)/modules/qrc_msg_management/ \


MAINSRC = $(CAR_DIR)/app/main.c

#CSRCS += $(CAR_DIR)/modules/qrc_msg_management/qrc_msg_management.c


CFLAGS += $(C_INCLUDES)

CFLAGS += -Wl,-u_printf_float

include $(APPDIR)/Application.mk
