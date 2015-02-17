################################################################################
#
#  NetFPGA-10G http://www.netfpga.org
#
#  File:
#        Makefile
#
#  Project:
#        upb_openflowswitch
#
#  Author:
#        James Hongyi Zeng (Original author)
#        Michael Lass
#
#  Description:
#        This file is based on the loopback_test Makefile by James Hongyi 
#        Zeng in the NetFPGA-10G project which in turn is based on the 
#        standard Xilinx XPS Makefile.
#        Please note that the software flow is deprecated by Xilinx.
#        Use Xilinx SDK instead.
#
#        For more information about how Xilinx EDK works, please visit
#        http://www.xilinx.com/support/documentation/dt_edk.htm
#        See inline comments for usage
#
#  Copyright notice:
#        Copyright (C) 2010, 2011 The Board of Trustees of The Leland Stanford
#                                 Junior University
#
#  Licence:
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

.NOTPARALLEL:

include system_incl.make

SYNTH_CMD   = ./synthesis.sh

#################################################################
# PHONY TARGETS
#################################################################
.PHONY: netlistclean
.PHONY: bitsclean

#################################################################
# EXTERNAL TARGETS
#################################################################
bits: $(SYSTEM_BIT)

netlist: $(POSTSYN_NETLIST)

clean: hwclean
	rm -f _impact.cmd
	rm -rf __xps
	rm -rf blockdiagram
	rm -rf etc
	rm -f *.opt
	rm -f *.log
	rm -f *.make
	rm -f system.mss

hwclean: netlistclean bitsclean
	rm -rf implementation synthesis xst hdl
	rm -rf xst.srp $(SYSTEM).srp
	rm -f __xps/ise/_xmsgs/bitinit.xmsgs

netlistclean:
	rm -f $(POSTSYN_NETLIST)
	rm -f platgen.log
	rm -f __xps/ise/_xmsgs/platgen.xmsgs
	rm -f $(BMM_FILE)

bitsclean:
	rm -f $(SYSTEM_BIT)
	rm -f implementation/$(SYSTEM).ncd
	rm -f implementation/$(SYSTEM)_bd.bmm
	rm -f implementation/$(SYSTEM)_map.ncd
	rm -f implementation/download.bit
	rm -f __xps/$(SYSTEM)_routed
	rm -rf bitfiles

#################################################################
# HARDWARE IMPLEMENTATION FLOW
#################################################################


$(BMM_FILE) \
$(WRAPPER_NGC_FILES): $(MHSFILE) __xps/platgen.opt \
                      $(CORE_STATE_DEVELOPMENT_FILES)
	@echo "****************************************************"
	@echo "Creating system netlist for hardware specification.."
	@echo "****************************************************"
	platgen $(PLATGEN_OPTIONS) $(MHSFILE)

#James: AXI interconnect workaround. Please note that the SCR is associated with the instance name.
$(POSTSYN_NETLIST): $(WRAPPER_NGC_FILES)
	@echo "****************************************************************************"
	@echo "Patching"
	@echo " system_axi_interconnect_0_wrapper_xst.scr"
	@echo "to synthesize for Virtex 6"
	@echo "****************************************************************************"
	sed -i 's/.*-p .*/-p virtex6/g' ./synthesis/system_axi_interconnect_0_wrapper_xst.scr
	mkdir implementation/system_axi_interconnect_0_wrapper
	@echo "********************"
	@echo "Running synthesis..."
	@echo "********************"
	bash -c "cd synthesis; $(SYNTH_CMD)"

#James: Use multi-core map and par. Parameters in nf10/xflow.opt
__xps/$(SYSTEM)_routed: $(FPGA_IMP_DEPENDENCY)
	@echo "*********************************************"
	@echo "Running Xilinx Implementation tools.."
	@echo "*********************************************"
	@cp -f $(UCF_FILE) implementation/$(SYSTEM).ucf
	@cp -f nf10/xflow.opt implementation/xflow.opt
	xflow -wd implementation -p $(DEVICE) -implement xflow.opt $(SYSTEM).ngc
	touch __xps/$(SYSTEM)_routed

#Shahbaz: Use configuration clock. Parameters in nf10/bitgen.ut
$(SYSTEM_BIT): __xps/$(SYSTEM)_routed nf10/bitgen.ut
	xilperl $(XILINX_EDK_DIR)/data/fpga_impl/observe_par.pl $(OBSERVE_PAR_OPTIONS) implementation/$(SYSTEM).par
	@echo "*********************************************"
	@echo "Running Bitgen.."
	@echo "*********************************************"
	@cp -f nf10/bitgen.ut implementation/bitgen.ut
	cd implementation ; bitgen -w -f bitgen.ut $(SYSTEM) ; cd ..
	@mkdir -p bitfiles
	cp ./implementation/system.bit ./bitfiles/upb_openflowswitch.bit
	@promgen -c -u 0 implementation/system.bit -p bin -w -data_width 8 -s 8192
	cp ./implementation/system.bin ./bitfiles/upb_openflowswitch.bin
	@echo "**********************************"
	@echo "Unconstrained IO Pins:"
	@echo "**********************************"
	@grep UNLOCATED implementation/system.pad || true

system_incl.make:
	@echo exit|xps -nw system.xmp
