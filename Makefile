# SPDX-License-Identifier: GPL-2.0-or-later
#
# Kbuild Makefile for SolidRun SolidSense AIOT System Controller Driver
#
# Copyright (C) 2026 Josua Mayer <josua@solid-run.com>
#

obj-m += ssaiot-sc-mfd.o
ssaiot-sc-mfd-y := core.o irq.o mfd.o transport.o

obj-m += ssaiot-sc-gnss.o
ssaiot-sc-gnss-y := gnss.o

obj-m += ssaiot-sc-rtc.o
ssaiot-sc-rtc-y := rtc.o

obj-m += ssaiot-sc-charger.o
ssaiot-sc-charger-y := charger.o

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
PWD := $(CURDIR)

all modules:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) clean
