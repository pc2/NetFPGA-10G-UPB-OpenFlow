# UPB AXI-4 Stream ChipScope core
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

.PHONY: clean install

all: install

install: netlist/nf10_upb_axi_stream_chipscope.edf

prepare:
	mkdir -p $(OUTPUT_DIRS)

netlist/nf10_upb_axi_stream_chipscope.edf: prj/nf10_upb_axi_stream_chipscope.prj
	$(SYNPLIFY_CMD) -batch $<

clean:
	find netlist -mindepth 1 ! -name chipscope_ila_512.ngc -delete
	find prj -mindepth 1 ! -name nf10_upb_axi_stream_chipscope.prj -delete

