################################################################################
#
#  NetFPGA-10G http://www.netfpga.org
#
#  File:
#        Makefile
#
#  Library:
#        hw/std/pcores/nf10_10g_interface_v1_10_a
#
#  Author:
#        James Hongyi Zeng
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

install: netlist/xaui.ngc netlist/xgmac.ngc

netlist/xaui.ngc: xco/xaui.xco
	@mkdir -p netlist;
	@mkdir -p $(COREGEN_DIR)/xaui;
	@cd $(COREGEN_DIR)/xaui && coregen -b ../../xco/xaui.xco
	@cp $(COREGEN_DIR)/xaui/xaui.v hdl/verilog/xilinx
	@cp $(COREGEN_DIR)/xaui/xaui.ngc netlist/
	@cp $(COREGEN_DIR)/xaui/xaui/example_design/xaui_tx_sync.v hdl/verilog/xilinx
	@cp $(COREGEN_DIR)/xaui/xaui/example_design/xaui_chanbond_monitor.v hdl/verilog/xilinx
	@cp $(COREGEN_DIR)/xaui/xaui/example_design/xaui_cc_2b_1skp.v hdl/verilog/xilinx
	@cp $(COREGEN_DIR)/xaui/xaui/example_design/xaui_block.v hdl/verilog/xilinx
	@cp $(COREGEN_DIR)/xaui/xaui/example_design/xaui_rocketio_wrapper.v hdl/verilog/xilinx
	@cp $(COREGEN_DIR)/xaui/xaui/example_design/xaui_rocketio_wrapper_tile.v hdl/verilog/xilinx
	@patch hdl/verilog/xilinx/xaui_cc_2b_1skp.v hdl/verilog/xilinx/cc_2b_1skp.diff
	@patch hdl/verilog/xilinx/xaui_block.v hdl/verilog/xilinx/xaui_block.diff
	@patch hdl/verilog/xilinx/xaui_rocketio_wrapper.v hdl/verilog/xilinx/rocketio_wrapper.diff
	@patch hdl/verilog/xilinx/xaui_rocketio_wrapper_tile.v hdl/verilog/xilinx/rocketio_wrapper_tile.diff;
	@echo "/////////////////////////////////////////";
	@echo "//Xilinx XAUI core installed.";
	@echo "/////////////////////////////////////////";
	@rm -rf $(COREGEN_DIR)/xaui;

netlist/xgmac.ngc: xco/xgmac.xco
	@mkdir -p netlist;
	@mkdir -p $(COREGEN_DIR)/xgmac;
	@cd $(COREGEN_DIR)/xgmac &&	coregen -b ../../xco/xgmac.xco
	@cp $(COREGEN_DIR)/xgmac/xgmac.v hdl/verilog/xilinx
	@cp $(COREGEN_DIR)/xgmac/xgmac.ngc netlist/
	@echo "/////////////////////////////////////////";
	@echo "//Xilinx 10G Ethernet MAC core installed.";
	@echo "/////////////////////////////////////////";
	@rm -rf $(COREGEN_DIR)/xgmac;

clean:
	@rm -rf $(COREGEN_DIR);
	@rm -rf netlist;
	@rm -rf hdl/verilog/xilinx/*.v
	@echo "/////////////////////////////////////////";
	@echo "//Xilinx files removed.";
	@echo "/////////////////////////////////////////";
