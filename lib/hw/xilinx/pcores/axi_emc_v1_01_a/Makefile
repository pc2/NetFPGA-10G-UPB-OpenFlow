################################################################################
#
#  NetFPGA-10G http://www.netfpga.org
#
#  File:
#        Makefile
#
#  Library:
#        hw/xilinx/pcores/axi_emc_v1_01_a
#
#  Description:
#        make install : Copy Xilinx files into NetFPGA-10G library
#
#        For more information about how Xilinx EDK works, please visit
#        http://www.xilinx.com/support/documentation/dt_edk.htm
#
#  Copyright notice:
#        Copyright (C) 2010, 2011 The Board of Trustees of The Leland Stanford
#                                 Junior University
#
#  Modifications for the UPB OpenFlow Switch project:
#       Copyright (c) 2014, 2015 Jörg Niklas, osjsn@niklasfamily.de
#       Project Group "On-the-Fly Networking for Big Data"
#       Computer Engineering Group, University of Paderborn
#
#       DISCLAIMER NOTICE
#       We are not affiliated, associated, authorized, endorsed by, or in any way
#       officially connected with Xilinx, Inc. or any of its subsidiaries or its
#       affiliates.  In no event whatsoever shall Xilinx, Inc. or any of its
#       subsidiaries or its affiliates have any warranty or support commitment
#       for this software or liability for loss, injury or damage in connection
#       with this software, including but not limited to the use or display thereof.
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
#        This file was developed by SRI International and the University of
#        Cambridge Computer Laboratory under DARPA/AFRL contract (FA8750-10-C-0237)
#        ("CTSRD"), as part of the DARPA CRASH research programme.
#

XILINX_HW_LIB_DIR = $(XILINX_EDK)/hw/XilinxProcessorIPLib/pcores
HW_LIB_INSTANCE = $(shell basename $(shell pwd))

all: install

install:
	false | cp -ri $(XILINX_HW_LIB_DIR)/$(HW_LIB_INSTANCE)/* . > /dev/null 2>&1
	sed -i '/OPTION ARCH_SUPPORT_MAP .*/d' data/axi_emc_v2_1_0.mpd

clean:
	rm -rf data doc hdl

