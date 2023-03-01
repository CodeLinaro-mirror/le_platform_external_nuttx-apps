
include $(APPDIR)/Make.defs

CAR_DIR := $(APPDIR)/ackerman
PROGNAME  = ackerman
PRIORITY  = SCHED_PRIORITY_DEFAULT
STACKSIZE = $(CONFIG_DEFAULT_TASK_STACKSIZE)
MODULE    = $(CONFIG_APP_ACKERMAN)

DEPPATH += --dep-path motion
DEPPATH += --dep-path imu
DEPPATH += --dep-path roscom
DEPPATH += --dep-path rc

C_INCLUDES +=  \
	-I$(CAR_DIR) \
	-I$(CAR_DIR)/motion \
	-I$(CAR_DIR)/imu \
	-I$(CAR_DIR)/roscom \
	-I$(CAR_DIR)/oled \
	-I$(CAR_DIR)/odom \
	-I$(CAR_DIR)/misc \
	-I$(CAR_DIR)/rc \

MAINSRC = car_main.c
CSRCS += $(CAR_DIR)/motion/motion_task.c
CSRCS += $(CAR_DIR)/motion/car_pwm.c
CSRCS += $(CAR_DIR)/motion/car_encoder.c
CSRCS += $(CAR_DIR)/motion/car_gpio.c
CSRCS += $(CAR_DIR)/motion/kinematics.c

CSRCS += $(CAR_DIR)/imu/car_imu.c

CSRCS += $(CAR_DIR)/oled/oled.c

CSRCS += $(CAR_DIR)/roscom/ros_com.c

CSRCS += $(CAR_DIR)/odom/odom.c

CSRCS += $(CAR_DIR)/misc/car_adc.c

CSRCS += $(CAR_DIR)/rc/rc.c

CFLAGS += $(C_INCLUDES)

CFLAGS += -Wl,-u_printf_float

include $(APPDIR)/Application.mk