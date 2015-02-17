#
# UPB Output Queue core
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

.PHONY: prepare clean install

OUTPUT_DIRS = netlist/2_ports netlist/3_ports netlist/4_ports netlist/5_ports netlist/6_ports netlist/7_ports

all: install

install: 
	$(MAKE) prepare 
	$(MAKE) $(addsuffix /nf10_upb_output_queue.edf, $(OUTPUT_DIRS))

prepare:
	mkdir -p $(OUTPUT_DIRS)

netlist/%_ports/nf10_upb_output_queue.edf: hdl/verilog/nf10_upb_output_queue_%.prj
	$(SYNPLIFY_CMD) -batch $<

clean:
	rm -rf netlist/*







