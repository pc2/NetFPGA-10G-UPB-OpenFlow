# UPB ChipScope ICON core
#
# Copyright (c) 2014, 2015 Jörg Niklas
# osjsn@niklasfamily.de
#
# This file is part of the NetFPGA 10G UPB OpenFlow Switch project.
#
# Project Group "On-the-Fly Networking for Big Data"
# SFB 901 "On-The-Fly Computing"
#
# University of Paderborn
# Computer Engineering Group
# Pohlweg 47 - 49
# 33098 Paderborn
# Germany
#
#
# This file is free code: you can redistribute it and/or modify it under
# the terms of the GNU Lesser General Public License version 2.1 as
# published by the Free Software Foundation.
#
# This file is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this project. If not, see <http://www.gnu.org/licenses/>.
#

install: netlist/chipscope_icon_1_ports.ngc netlist/chipscope_icon_2_ports.ngc netlist/chipscope_icon_3_ports.ngc netlist/chipscope_icon_4_ports.ngc netlist/chipscope_icon_5_ports.ngc netlist/chipscope_icon_6_ports.ngc netlist/chipscope_ila_128.ngc netlist/chipscope_ila_256.ngc netlist/chipscope_ila_512.ngc netlist/chipscope_ila_1024.ngc

netlist/chipscope_icon_1_ports.ngc:
	@mkdir -p netlist
	@mkdir -p coregen_tmp/chipscope_icon_1_ports
	@cd coregen_tmp/chipscope_icon_1_ports && coregen -b ../../xco/chipscope_icon_1_ports.xco
	@mv coregen_tmp/chipscope_icon_1_ports/chipscope_icon_1_ports.ngc netlist

netlist/chipscope_icon_2_ports.ngc:
	@mkdir -p netlist
	@mkdir -p coregen_tmp/chipscope_icon_2_ports
	@cd coregen_tmp/chipscope_icon_2_ports && coregen -b ../../xco/chipscope_icon_2_ports.xco
	@mv coregen_tmp/chipscope_icon_2_ports/chipscope_icon_2_ports.ngc netlist

netlist/chipscope_icon_3_ports.ngc:
	@mkdir -p netlist
	@mkdir -p coregen_tmp/chipscope_icon_3_ports
	@cd coregen_tmp/chipscope_icon_3_ports && coregen -b ../../xco/chipscope_icon_3_ports.xco
	@mv coregen_tmp/chipscope_icon_3_ports/chipscope_icon_3_ports.ngc netlist

netlist/chipscope_icon_4_ports.ngc:
	@mkdir -p netlist
	@mkdir -p coregen_tmp/chipscope_icon_4_ports
	@cd coregen_tmp/chipscope_icon_4_ports && coregen -b ../../xco/chipscope_icon_4_ports.xco
	@mv coregen_tmp/chipscope_icon_4_ports/chipscope_icon_4_ports.ngc netlist

netlist/chipscope_icon_5_ports.ngc:
	@mkdir -p netlist
	@mkdir -p coregen_tmp/chipscope_icon_5_ports
	@cd coregen_tmp/chipscope_icon_5_ports && coregen -b ../../xco/chipscope_icon_5_ports.xco
	@mv coregen_tmp/chipscope_icon_5_ports/chipscope_icon_5_ports.ngc netlist

netlist/chipscope_icon_6_ports.ngc:
	@mkdir -p netlist
	@mkdir -p coregen_tmp/chipscope_icon_6_ports
	@cd coregen_tmp/chipscope_icon_6_ports && coregen -b ../../xco/chipscope_icon_6_ports.xco
	@mv coregen_tmp/chipscope_icon_6_ports/chipscope_icon_6_ports.ngc netlist

netlist/chipscope_ila_128.ngc:
	@mkdir -p netlist
	@mkdir -p coregen_tmp/chipscope_ila_128
	@cd coregen_tmp/chipscope_ila_128 && coregen -b ../../xco/chipscope_ila_128.xco
	@mv coregen_tmp/chipscope_ila_128/chipscope_ila_128.ngc netlist

netlist/chipscope_ila_256.ngc:
	@mkdir -p netlist
	@mkdir -p coregen_tmp/chipscope_ila_256
	@cd coregen_tmp/chipscope_ila_256 && coregen -b ../../xco/chipscope_ila_256.xco
	@mv coregen_tmp/chipscope_ila_256/chipscope_ila_256.ngc netlist

netlist/chipscope_ila_512.ngc:
	@mkdir -p netlist
	@mkdir -p coregen_tmp/chipscope_ila_512
	@cd coregen_tmp/chipscope_ila_512 && coregen -b ../../xco/chipscope_ila_512.xco
	@mv coregen_tmp/chipscope_ila_512/chipscope_ila_512.ngc netlist

netlist/chipscope_ila_1024.ngc:
	@mkdir -p netlist
	@mkdir -p coregen_tmp/chipscope_ila_1024
	@cd coregen_tmp/chipscope_ila_1024 && coregen -b ../../xco/chipscope_ila_1024.xco
	@mv coregen_tmp/chipscope_ila_1024/chipscope_ila_1024.ngc netlist

clean:
	rm -rf netlist coregen_tmp

