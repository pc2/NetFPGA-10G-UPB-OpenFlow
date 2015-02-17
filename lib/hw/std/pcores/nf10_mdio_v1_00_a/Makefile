################################################################################
#
#  NetFPGA-10G http://www.netfpga.org
#
#  File:
#        Makefile
#
#  Library:
#        hw/std/pcores/nf10_mdio_v1_00_a
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

XILINX_HW_LIB_DIR = $(XILINX_EDK)/hw/XilinxProcessorIPLib/pcores

install: hdl/vhdl/axi_interface.vhd

hdl/vhdl/axi_interface.vhd: $(XILINX_HW_LIB_DIR)/axi_ethernetlite_v1_00_a/hdl/vhdl/axi_interface.vhd \
																hdl/vhdl/mdio_if.diff
	@cp -f $(XILINX_HW_LIB_DIR)/axi_ethernetlite_v1_00_a/hdl/vhdl/axi_interface.vhd hdl/vhdl/axi_interface.vhd;
	@cp -f $(XILINX_HW_LIB_DIR)/axi_ethernetlite_v1_00_a/hdl/vhdl/mdio_if.vhd hdl/vhdl/mdio_if.vhd;
	@cp -f $(XILINX_HW_LIB_DIR)/axi_ethernetlite_v1_00_a/hdl/vhdl/axi_ethernetlite.vhd hdl/vhdl/nf10_mdio.vhd;
	@cp -f $(XILINX_HW_LIB_DIR)/axi_ethernetlite_v1_00_a/hdl/vhdl/emac.vhd hdl/vhdl/mdio_ipif.vhd;
	@patch hdl/vhdl/mdio_if.vhd hdl/vhdl/mdio_if.diff;
	@patch hdl/vhdl/nf10_mdio.vhd hdl/vhdl/nf10_mdio.diff;
	@patch hdl/vhdl/mdio_ipif.vhd hdl/vhdl/mdio_ipif.diff;
	@echo "/////////////////////////////////////////";
	@echo "//Xilinx MDIO interface - Hardware installed.";
	@echo "/////////////////////////////////////////";

clean:
	@rm -rf hdl/vhdl/*.vhd
	@echo "/////////////////////////////////////////";
	@echo "//Xilinx files removed.";
	@echo "/////////////////////////////////////////";

