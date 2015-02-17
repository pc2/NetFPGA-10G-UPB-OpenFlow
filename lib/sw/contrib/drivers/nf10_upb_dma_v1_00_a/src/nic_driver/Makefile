################################################################################
#
#  NetFPGA-10G http://www.netfpga.org
#
#  File:
#        Makefile
#
#  Project:
#        nic
#
#  Author:
#        Mario Flajslik
#
#        Modifications for the UPB OpenFlow Switch project:
#         Jörg Niklas, osjsn@niklasfamily.de
#         Project Group "On-the-Fly Networking for Big Data"
#         Computer Engineering Group, University of Paderborn
#
#  Description:
#        make : Make the nic host driver. Load the driver with "insmod nf10.ko"
#
#
#  Copyright notice:
#        Copyright (C) 2010, 2011 The Board of Trustees of The Leland Stanford
#                                 Junior University
#
#        Modifications for the UPB OpenFlow Switch project:
#          Copyright (c) 2014, 2015 Jörg Niklas, osjsn@niklasfamily.de
#
#  Licence:
#        This file is part of the NetFPGA 10G development base package.
#
#        This file is free code: you can redistribute it and/or modify it under
#        the terms of the GNU Lesser General Public License version 2.1 as
#        published by the Free Software Foundation.
#
#        This package is distributed in the hope that it will be useful, but
#        WITHOUT ANY WARRANTY; without even the implied warranty of
#        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#        Lesser General Public License for more details.
#
#        You should have received a copy of the GNU Lesser General Public
#        License along with the NetFPGA source package.  If not, see
#        http://www.gnu.org/licenses/.
#
#

obj-m += nf10.o
nf10-objs += nf10driver.o
nf10-objs += nf10iface.o
nf10-objs += nf10fops.o
nf10-objs += nf10priv.o
nf10-objs += nf10_phy_conf.o

ifeq ($(DEBUG),1)
$(info *************** Building debug version of kernel module... ***************)
ccflags-y=-g -ggdb -gdwarf-4 -fvar-tracking-assignments -O0
endif

ifndef $(KERNEL_VERSION)
KERNEL_VERSION=$(shell uname -r)
endif

.PHONY: all modules clean install modules_install

all: modules

modules:
	make -C /lib/modules/$(KERNEL_VERSION)/build M=$(CURDIR) modules

clean:
	make -C /lib/modules/$(KERNEL_VERSION)/build M=$(CURDIR) clean

install: modules_install
modules_install:
	make -C /lib/modules/$(KERNEL_VERSION)/build M=$(CURDIR) modules_install
	/sbin/depmod -a $(KERNEL_VERSION)

