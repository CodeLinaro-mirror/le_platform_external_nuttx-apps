
include $(APPDIR)/Make.defs

CAR_DIR := $(APPDIR)/amrboot
PROGNAME  = qcamrboot
PRIORITY  = SCHED_PRIORITY_DEFAULT
STACKSIZE = 32768
MODULE    = $(CONFIG_APP_QCAMRBOOT)

DEPPATH += --dep-path src

C_INCLUDES +=  \
        -I$(CAR_DIR) \
        -I$(CAR_DIR)/inc \

MAINSRC = amrboot_main.c
CSRCS += $(CAR_DIR)/src/flash.c
CSRCS += $(CAR_DIR)/src/xmodem.c


CFLAGS += $(C_INCLUDES)

CFLAGS += -Wl,-u_printf_float

include $(APPDIR)/Application.mk
