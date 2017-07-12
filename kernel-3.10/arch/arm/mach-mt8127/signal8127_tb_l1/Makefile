
include $(srctree)/drivers/misc/mediatek/Makefile.custom

ifneq ($(CONFIG_ARCH_MTK_PROJECT),)
ARCH_MTK_PROJECT := $(shell echo $(CONFIG_ARCH_MTK_PROJECT))
obj-y   += core/
obj-y   += dct/dct/
endif
ifeq ($(CONFIG_MTK_MPU6050G_NEW),y)
obj-y   +=  accelerometer/mpu6050g-new/
endif
ifeq ($(CONFIG_MTK_MPU6050GY_NEW),y)
obj-y   +=  gyroscope/mpu6050gy-new/
endif
ifeq ($(CONFIG_MTK_AKM09911_NEW),y)
obj-y	+=  magnetometer/akm09911-new/
endif
ifeq ($(CONFIG_CUSTOM_KERNEL_ALSPS),y)
obj-y   +=  alsps/
endif
ifeq ($(CONFIG_CC2530_ZIGBEE),y)
obj-y   +=  cc2530_zigbee/
endif
ifeq ($(CONFIG_IR_SCM),y)
obj-y   +=  ir_scm/
endif
ifeq ($(CONFIG_MTK_HDMI_SUPPORT),y)
obj-y	+=	hdmi/
endif

obj-y   +=  cam_cal/
obj-y   +=  camera/camera/
obj-y   +=  eeprom/
###obj-y   +=  flashlight/
obj-y   +=  flashlight/dummy_flashlight/
obj-y   +=  headset/
###zbj-y   +=  imgsensor/
obj-y   +=  keypad/
obj-y   +=  leds/
obj-y   +=  lens/
obj-y   +=  sound/
###obj-y   +=  ssw/
obj-y   +=  touchpanel/
obj-y   +=  vibrator/
#obj-y   +=  lcm/
#obj-y   +=  wmt/
#obj-y   +=  ir_sc/
