# Synopsys, Inc. constraint file
# /home/bevan/pgotfnetworking-netfpga/lib/hw/contrib/pcores/upb_dma_v1_00_a/prj/dma_engine.sdc
# Written on Mon Mar 17 22:42:46 2014
# by Synplify Premier with Design Planner, G-2012.09-SP1  Scope Editor

#
# Collections
#

#
# Clocks
#
define_clock   {pcie_clk} -name {pcie_clk}  -freq 125 -clockgroup default_clkgroup_0
define_clock   {axi_clk} -name {axi_clk}  -freq 100 -clockgroup default_clkgroup_1
define_clock   {tx_clk} -name {tx_clk}  -freq 160 -clockgroup default_clkgroup_2
define_clock   {rx_clk} -name {rx_clk}  -freq 160 -clockgroup default_clkgroup_3

#
# Clock to Clock
#

#
# Inputs/Outputs
#

#
# Registers
#

#
# Delay Paths
#

#
# Attributes
#

#
# I/O Standards
#

#
# Compile Points
#

#
# Other
#
derive_pll_clocks  
