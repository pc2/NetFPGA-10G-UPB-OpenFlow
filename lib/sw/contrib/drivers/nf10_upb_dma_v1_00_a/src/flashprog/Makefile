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
#        Jong Han
#
#  Description:
#        NF10 Netfpga Configuration.
#
#
#  Copyright notice:
#        Copyright (C) 2013 University of Cambridge
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

all:	nf10_configure

nf10_configure:
	gcc nf10_configure.c ../../../../../xilinx/drivers/emc_v4_00_a/src/emc_flash_lib.c reg_lib.c -I . -I ../../../../../xilinx/drivers/emc_v4_00_a/src -o ../../bin/nf10_configure
	gcc nf10_flash_a.c ../../../../../xilinx/drivers/emc_v4_00_a/src/emc_flash_lib.c reg_lib.c -I . -I ../../../../../xilinx/drivers/emc_v4_00_a/src -o ../../bin/nf10_flash_a
	gcc nf10_flash_b.c ../../../../../xilinx/drivers/emc_v4_00_a/src/emc_flash_lib.c reg_lib.c -I . -I ../../../../../xilinx/drivers/emc_v4_00_a/src -o ../../bin/nf10_flash_b

clean:
	rm -f ../../bin/nf10_configure ../../bin/nf10_flash_a ../../bin/nf10_flash_b

