################################################################################
#
#  NetFPGA-10G http://www.netfpga.org
#
#  File:
#        Makefile
#
#  Library:
#        hw/contrib/pcores/upb_dma_v1_00_a
#
#  Author:
#        Mario Flajslik
#        Modified by Michael Lass
#
#  Description:
#        make synthesize125: regen netlist for 125MHz
#        make synthesize250: regen netlist for 250MHz
#        make install : Copy Xilinx files into NetFPGA-10G library
#
#        For more information about how Xilinx EDK works, please visit
#        http://www.xilinx.com/support/documentation/dt_edk.htm
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

COREGEN_DIR:= coregen

all: synthesize hdl/verilog/xilinx/pcie_top.v

synthesize:
	mkdir -p hdl/vhdl
	cd prj && ${SYNPLIFY_CMD} -batch dma_engine.prj
	sed -i '/library synplify;/d;/use synplify.components.all;/d' prj/dma_engine.vhm

hdl/verilog/xilinx/pcie_top.v: xco/endpoint_blk_plus_v1_15.xco
	@mkdir -p $(COREGEN_DIR);
	@cd $(COREGEN_DIR) && coregen -b ../xco/endpoint_blk_plus_v1_15.xco
	@cp $(COREGEN_DIR)/endpoint_blk_plus_v1_15/source/*.v hdl/verilog/xilinx/
	@patch hdl/verilog/xilinx/pcie_top.v <hdl/verilog/xilinx/pcie_top.v.patch
	@echo "/////////////////////////////////////////";
	@echo "//Xilinx PCIe core installed.";
	@echo "/////////////////////////////////////////";
	@rm -rf $(COREGEN_DIR);

clean:
	rm -f hdl/verilog/xilinx/*.v prj/.recordref_modgen hdl/vhdl
	-find prj/* ! -name "dma_engine.prj" ! -name "dma_engine_250.prj" ! -name "dma_engine.sdc" -delete
	-find netlist/* -type f -delete
