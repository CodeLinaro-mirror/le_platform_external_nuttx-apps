include $(APPDIR)/Make.defs

QTIUAR_TEST_DIR := $(APPDIR)/uart_test_app
PROGNAME  = uart_test_app
PRIORITY  = SCHED_PRIORITY_DEFAULT
STACKSIZE = $(CONFIG_DEFAULT_TASK_STACKSIZE)
MODULE    = $(CONFIG_QTI_UART_TEST_APP)

DEPPATH += --dep-path app

C_INCLUDES +=  \
    -I$(QTIUAR_TEST_DIR) \

MAINSRC = $(QTIUAR_TEST_DIR)/main.c

# app source

# add define
CFLAGS += -DQRC_MCB
CFLAGS += -DDEBUG_MCB

CFLAGS += $(C_INCLUDES)

CFLAGS += -Wl,-u_printf_float,-o0

include $(APPDIR)/Application.mk
