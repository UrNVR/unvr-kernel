KBUILD_CFLAGS += -DAL_DEV_ID_ALPINE_V1=0
KBUILD_CFLAGS += -DAL_DEV_ID_ALPINE_V2=1
KBUILD_CFLAGS += -DAL_DEV_ID_ALPINE_V3=2
KBUILD_CFLAGS += -DAL_DEV_ID_ALPINE_V4=3

KBUILD_CFLAGS += -DAL_DEV_REV_ID_V2=0
KBUILD_CFLAGS += -DAL_DEV_REV_ID_V3=1

ifeq ($(CONFIG_ALPINE_PLATFORM),"ALPINE_V2")
KBUILD_CFLAGS += -DAL_DEV_REV_ID=AL_DEV_REV_ID_V2
KBUILD_CFLAGS += -DAL_DEV_ID=AL_DEV_ID_ALPINE_V2
KBUILD_CFLAGS += $(HAL_PLATFORM_INCLUDE_PATH_ALPINE_V2)
else ifeq ($(CONFIG_ALPINE_PLATFORM),"ALPINE_V3")
KBUILD_CFLAGS += -DAL_DEV_REV_ID=AL_DEV_REV_ID_V3
KBUILD_CFLAGS += -DAL_DEV_ID=AL_DEV_ID_ALPINE_V3
KBUILD_CFLAGS += $(HAL_PLATFORM_INCLUDE_PATH_ALPINE_V3)
endif

HAL_TOP=$(srctree)/drivers/soc/alpine/HAL
include $(HAL_TOP)/file_list.mk

KBUILD_CFLAGS += -I$(srctree)/drivers/soc/alpine
KBUILD_CFLAGS += -I$(srctree)/include/soc/alpine

KBUILD_CFLAGS += $(HAL_USER_INCLUDE_PATH)
KBUILD_CFLAGS += $(HAL_DRIVER_INCLUDE_PATH)
KBUILD_CFLAGS += $(HAL_INIT_INCLUDE_PATH)
