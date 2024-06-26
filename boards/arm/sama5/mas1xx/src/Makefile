############################################################################
# boards/arm/sama5/mas1xx/src/Makefile
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
############################################################################

include $(TOPDIR)/Make.defs

CSRCS = sam_boot.c sam_gpio.c mas1xx_led.c mas1xx_lte.c mas1xx_param.c mas1xx_xio.c

ifeq ($(CONFIG_SAMA5_SPI0),y)
CSRCS += sam_spi.c
else
ifeq ($(CONFIG_SAMA5_SPI1),y)
CSRCS += sam_spi.c
endif
endif

ifeq ($(CONFIG_SAMA5_TWI0),y)
CSRCS += sam_i2c.c
else
ifeq ($(CONFIG_SAMA5_TWI1),y)
CSRCS += sam_i2c.c
endif
endif

ifeq ($(CONFIG_MTD_AT25),y)
ifeq ($(CONFIG_SAMA5_SPI0),y)
CSRCS += sam_at25.c
endif
endif

ifeq ($(CONFIG_SAMA5_SDMMC0),y)
CSRCS += sam_sdmmc.c
else
ifeq ($(CONFIG_SAMA5_SDMMC1),y)
CSRCS += sam_sdmmc.c
endif

endif

ifeq ($(CONFIG_SAMA5_UHPHS),y)
CSRCS += sam_usb.c
else
ifeq ($(CONFIG_SAMA5_UDPHS),y)
CSRCS += sam_usb.c
endif
endif

ifeq ($(CONFIG_SAMA5_EMACB),y)
CSRCS += sam_ethernet.c
endif

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += sam_appinit.c
CSRCS += sam_bringup.c
endif

ifeq ($(CONFIG_ADC),y)
CSRCS += sam_adc.c
endif

ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += sam_autoleds.c
endif
CSRCS += sam_userleds.c

ifeq ($(CONFIG_ARCH_BUTTONS),y)
CSRCS += sam_buttons.c
endif

ifeq ($(CONFIG_BOARDCTL_RESET),y)
CSRCS += sam_reset.c
endif

ifeq ($(CONFIG_LIBC_ZONEINFO_ROMFS),y)
CSRCS += sam_zoneinfo.c
endif

ifeq ($(CONFIG_ETC_ROMFS),y)
  RCSRCS = etc/init.d/rc.sysinit etc/init.d/rcS
  RCRAWS = etc/zoneinfo/Asia/Tokyo
endif

include $(TOPDIR)/boards/Board.mk
