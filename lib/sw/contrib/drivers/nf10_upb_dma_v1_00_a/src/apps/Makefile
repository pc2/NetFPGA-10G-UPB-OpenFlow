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
#        Jong Hun Han
#
#  Description:
#        make : Make example applications to read stats and read/write AXI
#               registers
#
#
#  Copyright notice:
#        Copyright (C) 2010, 2011 The Board of Trustees of The Leland Stanford
#                                 Junior University
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

all:
	@mkdir -p ../../bin
	gcc -o ../../bin/stats stats.c
	gcc -o ../../bin/rdaxi rdaxi.c
	gcc -o ../../bin/wraxi wraxi.c
	gcc -o ../../bin/ael2005_conf ael2005_conf.c

clean:
	rm -f ../../bin/stats ../../bin/rdaxi ../../bin/wraxi ../../bin/ael2005_conf
