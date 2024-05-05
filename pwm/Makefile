#
# Copyright (C) 2008-2012 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=airoha_pwm
PKG_RELEASE:=1

include $(INCLUDE_DIR)/package.mk

define KernelPackage/airoha_pwm
  CATEGORY:=Econet Properties
  SUBMENU:=ECNT_Drivers
  TITLE:=Econet 7523 PWM driver
  FILES:=$(PKG_BUILD_DIR)/pwm-en7523.ko
  AUTOLOAD:=$(call AutoLoad,30,pwm-en7523,1)
  KCONFIG:=CONFIG_PWM=y CONFIG_GPIOLIB=y
endef

define KernelPackage/airoha_pwm/description
  Kernel module for PWM driver
endef

MAKE_OPTS:= \
	ARCH="$(LINUX_KARCH)" \
	CROSS_COMPILE="$(TARGET_CROSS)" \
	M="$(PKG_BUILD_DIR)"

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

define Build/Compile
	$(MAKE) -C "$(LINUX_DIR)" \
		$(MAKE_OPTS) \
		modules
endef

$(eval $(call KernelPackage,airoha_pwm))
