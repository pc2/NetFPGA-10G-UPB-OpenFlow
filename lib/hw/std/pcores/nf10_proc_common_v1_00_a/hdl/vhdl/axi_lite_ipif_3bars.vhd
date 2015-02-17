------------------------------------------------------------------------
--
--  NetFPGA-10G http://www.netfpga.org
--
--  File:
--        axi_lite_ipif_3bars.vhd
--
--  Library:
--        std/pcores/nf10_proc_common_v1_00_a
--
--  Module:
--        axi_lite_ipif_3bars
--
--  Author:
--        Muhammad Shahbaz
--
--  Description:
--        AXILITE IPIF wrapper with 3 memory bars
--
--  Copyright notice:
--        Copyright (C) 2010, 2011 The Board of Trustees of The Leland Stanford
--                                 Junior University
--
--  Licence:
--        This file is part of the NetFPGA 10G development base package.
--
--        This file is free code: you can redistribute it and/or modify it under
--        the terms of the GNU Lesser General Public License version 2.1 as
--        published by the Free Software Foundation.
--
--        This package is distributed in the hope that it will be useful, but
--        WITHOUT ANY WARRANTY; without even the implied warranty of
--        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
--        Lesser General Public License for more details.
--
--        You should have received a copy of the GNU Lesser General Public
--        License along with the NetFPGA source package.  If not, see
--        http://www.gnu.org/licenses/.
--
-----------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;

library proc_common_v3_00_a;
use proc_common_v3_00_a.proc_common_pkg.all;
use proc_common_v3_00_a.ipif_pkg.all;

library axi_lite_ipif_v1_01_a;
use axi_lite_ipif_v1_01_a.axi_lite_ipif;


entity axi_lite_ipif_3bars is
  generic
  (
    C_S_AXI_DATA_WIDTH             : integer              := 32;
    C_S_AXI_ADDR_WIDTH             : integer              := 32;
	C_USE_WSTRB                    : integer              := 0; -- Enable(1),   Disable(0, byte enables fixed to '1111')
	C_DPHASE_TIMEOUT               : integer              := 8; -- Enable(!=0), Disbale(0, data phase timeout not implemented)
    C_BAR0_BASEADDR                : std_logic_vector     := X"FFFFFFFF";
    C_BAR0_HIGHADDR                : std_logic_vector     := X"00000000";
    C_BAR1_BASEADDR                : std_logic_vector     := X"FFFFFFFF";
    C_BAR1_HIGHADDR                : std_logic_vector     := X"00000000";
    C_BAR2_BASEADDR                : std_logic_vector     := X"FFFFFFFF";
    C_BAR2_HIGHADDR                : std_logic_vector     := X"00000000"
  );
  port
  (
    S_AXI_ACLK                     : in  std_logic;
    S_AXI_ARESETN                  : in  std_logic;
    S_AXI_AWADDR                   : in  std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
    S_AXI_AWVALID                  : in  std_logic;
    S_AXI_WDATA                    : in  std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
    S_AXI_WSTRB                    : in  std_logic_vector((C_S_AXI_DATA_WIDTH/8)-1 downto 0);
    S_AXI_WVALID                   : in  std_logic;
    S_AXI_BREADY                   : in  std_logic;
    S_AXI_ARADDR                   : in  std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
    S_AXI_ARVALID                  : in  std_logic;
    S_AXI_RREADY                   : in  std_logic;
    S_AXI_ARREADY                  : out std_logic;
    S_AXI_RDATA                    : out std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
    S_AXI_RRESP                    : out std_logic_vector(1 downto 0);
    S_AXI_RVALID                   : out std_logic;
    S_AXI_WREADY                   : out std_logic;
    S_AXI_BRESP                    : out std_logic_vector(1 downto 0);
    S_AXI_BVALID                   : out std_logic;
    S_AXI_AWREADY                  : out std_logic;
	-- Controls to the IP/IPIF modules
    Bus2IP_Clk                     : out std_logic;
    Bus2IP_Resetn                  : out std_logic;
    Bus2IP_Addr                    : out std_logic_vector((C_S_AXI_ADDR_WIDTH-1) downto 0);
    Bus2IP_RNW                     : out std_logic;
    Bus2IP_BE                      : out std_logic_vector(((C_S_AXI_DATA_WIDTH/8)-1) downto 0);
    Bus2IP_CS                      : out std_logic_vector(2 downto 0);
    Bus2IP_Data                    : out std_logic_vector((C_S_AXI_DATA_WIDTH-1) downto 0);
    IP2Bus_Data                    : in  std_logic_vector((C_S_AXI_DATA_WIDTH-1) downto 0);
    IP2Bus_WrAck                   : in  std_logic;
    IP2Bus_RdAck                   : in  std_logic;
    IP2Bus_Error                   : in  std_logic
  );
  
end entity axi_lite_ipif_3bars;

------------------------------------------------------------------------------
-- Architecture section
------------------------------------------------------------------------------

architecture IMP of axi_lite_ipif_3bars is
  
  constant C_S_AXI_MIN_SIZE		          : std_logic_vector(31 downto 0)	:= X"FFFFFFFF"; -- need to calc exact min size
  
  constant ZERO_ADDR_PAD                  : std_logic_vector(0 to 31) := (others => '0');
  
  constant IPIF_ARD_ADDR_RANGE_ARRAY      : SLV64_ARRAY_TYPE     := 
    (
      ZERO_ADDR_PAD & C_BAR0_BASEADDR,
      ZERO_ADDR_PAD & C_BAR0_HIGHADDR,
	  ZERO_ADDR_PAD & C_BAR1_BASEADDR,
      ZERO_ADDR_PAD & C_BAR1_HIGHADDR,
	  ZERO_ADDR_PAD & C_BAR2_BASEADDR,
      ZERO_ADDR_PAD & C_BAR2_HIGHADDR
    );

  constant IPIF_ARD_NUM_CE_ARRAY          : INTEGER_ARRAY_TYPE   := 
    (
      1, -- CE count for BAR0 
	  1,  -- CE count for BAR1
	  1  -- CE count for BAR2
    );

begin

  ------------------------------------------
  -- instantiate axi_lite_ipif
  ------------------------------------------
  AXI_LITE_IPIF_I : entity axi_lite_ipif_v1_01_a.axi_lite_ipif
    generic map
    (
      C_S_AXI_DATA_WIDTH             => C_S_AXI_DATA_WIDTH,
      C_S_AXI_ADDR_WIDTH             => C_S_AXI_ADDR_WIDTH,
      C_S_AXI_MIN_SIZE               => C_S_AXI_MIN_SIZE,
      C_USE_WSTRB                    => C_USE_WSTRB,
      C_DPHASE_TIMEOUT               => C_DPHASE_TIMEOUT,
      C_ARD_ADDR_RANGE_ARRAY         => IPIF_ARD_ADDR_RANGE_ARRAY,
      C_ARD_NUM_CE_ARRAY             => IPIF_ARD_NUM_CE_ARRAY
    )
    port map
    (
      S_AXI_ACLK                     => S_AXI_ACLK,
      S_AXI_ARESETN                  => S_AXI_ARESETN,
      S_AXI_AWADDR                   => S_AXI_AWADDR,
      S_AXI_AWVALID                  => S_AXI_AWVALID,
      S_AXI_WDATA                    => S_AXI_WDATA,
      S_AXI_WSTRB                    => S_AXI_WSTRB,
      S_AXI_WVALID                   => S_AXI_WVALID,
      S_AXI_BREADY                   => S_AXI_BREADY,
      S_AXI_ARADDR                   => S_AXI_ARADDR,
      S_AXI_ARVALID                  => S_AXI_ARVALID,
      S_AXI_RREADY                   => S_AXI_RREADY,
      S_AXI_ARREADY                  => S_AXI_ARREADY,
      S_AXI_RDATA                    => S_AXI_RDATA,
      S_AXI_RRESP                    => S_AXI_RRESP,
      S_AXI_RVALID                   => S_AXI_RVALID,
      S_AXI_WREADY                   => S_AXI_WREADY,
      S_AXI_BRESP                    => S_AXI_BRESP,
      S_AXI_BVALID                   => S_AXI_BVALID,
      S_AXI_AWREADY                  => S_AXI_AWREADY,
      Bus2IP_Clk                     => Bus2IP_Clk,
      Bus2IP_Resetn                  => Bus2IP_Resetn,
      Bus2IP_Addr                    => Bus2IP_Addr,
      Bus2IP_RNW                     => Bus2IP_RNW,
      Bus2IP_BE                      => Bus2IP_BE,
      Bus2IP_CS                      => Bus2IP_CS,
      Bus2IP_RdCE                    => open,
      Bus2IP_WrCE                    => open,
      Bus2IP_Data                    => Bus2IP_Data,
      IP2Bus_WrAck                   => IP2Bus_WrAck,
      IP2Bus_RdAck                   => IP2Bus_RdAck,
      IP2Bus_Error                   => IP2Bus_Error,
      IP2Bus_Data                    => IP2Bus_Data
    );

end IMP;
