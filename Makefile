################################################################################
#
#  NetFPGA-10G http://www.netfpga.org
#
#  File:
#        Makefile
#
#  Description:
#        make upb_cores:   Build only IP cores
#        make upb_project: Build only HW projects (IP cores required)
#        make clean:       Remove build files
#
#        For more information about how Xilinx EDK works, please visit
#        http://www.xilinx.com/support/documentation/dt_edk.htm
#
#  Copyright notice:
#        Copyright (C) 2010, 2011 The Board of Trustees of The Leland Stanford
#                                 Junior University
#        Copyright (C) 2014, 2015 Jörg Niklas  <osjsn@niklasfamily.de>
#                                 Michael Lass <bevan@bi-co.net>
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



UPB_HW_XILINX = \
	axi_emc_v1_01_a \
	axi_gpio_v1_01_b \
	axi_interconnect_v1_05_a \
	diff_input_buf_v1_00_a

UPB_HW_STD = \
	nf10_mdio_v1_00_a

UPB_HW_CONTRIB = \
	nf10_upb_10g_input_v1_00_a \
	nf10_upb_10g_interface_v1_00_a \
	nf10_upb_axi_lite_chipscope_v1_00_a \
	nf10_upb_axi_stream_chipscope_v1_00_a \
	nf10_upb_chipscope_icon_v1_00_a \
	nf10_upb_clock_generator_v1_00_a \
	nf10_upb_dma_input_v1_00_a \
	nf10_upb_dma_v1_00_a \
	nf10_upb_dummy_port_v1_00_a \
	nf10_upb_input_arbiter_v1_00_a \
	nf10_upb_interconnect_v1_00_a \
	nf10_upb_lib \
	nf10_upb_ofswitch_v1_00_a \
	nf10_upb_output_queue_v1_00_a \
	nf10_upb_switch_v1_00_a

UPB_SW_CONTRIB = \
	nf10_upb_dma_v1_00_a \
	nf10_upb_sdn_dataplane_v1_00_a

UPB_CORE_LIST = $(addprefix lib/hw/xilinx/pcores/,$(UPB_HW_XILINX)) $(addprefix lib/hw/std/pcores/,$(UPB_HW_STD)) $(addprefix lib/hw/contrib/pcores/,$(UPB_HW_CONTRIB)) $(addprefix lib/sw/contrib/drivers/,$(UPB_SW_CONTRIB))

UPB_PROJECT_LIST = contrib-projects/upb_l2switch/hw contrib-projects/upb_openflowswitch/hw

.PHONY: $(UPB_CORE_LIST) $(UPB_PROJECT_LIST) upb_cores upb_cores_clean upb_projects upb_projects_clean clean upb

upb: upb_projects

upb_cores: check-env $(UPB_CORE_LIST)

$(UPB_CORE_LIST):
	if test -f $@/Makefile; \
		then $(MAKE) -C $@; \
	fi; \

upb_cores_clean:
	for lib in $(UPB_CORE_LIST) ; do \
		if test -f $$lib/Makefile; \
			then $(MAKE) -C $$lib clean; \
		fi; \
	done;

upb_projects: check-env $(UPB_PROJECT_LIST)

$(UPB_PROJECT_LIST): upb_cores
	$(MAKE) -C $@ bits

upb_projects_clean:
	for prj in $(UPB_PROJECT_LIST) ; do \
		$(MAKE) -C $$prj clean; \
	done;

clean: upb_projects_clean upb_cores_clean

check-env:
ifndef XILINX_EDK
    $(error XILINX_EDK is undefined)
endif

