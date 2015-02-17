/*******************************************************************************
 *
 *  NetFPGA-10G http://www.netfpga.org
 *
 *  File:
 *        nf10_10g_interface.v
 *
 *  Library:
 *        hw/std/pcores/nf10_upb_10g_interface_v1_00_a
 *
 *  Module:
 *        nf10_upb_10g_interface
 *
 *  Author:
 *        James Hongyi Zeng
 *
 *  Modified :
 *        Neelakandan Manihatty Bojan 
 *        Thomas Löcke, University of Paderborn (2014-02-15)
 *
 *  Description:
 *        This is the combination of AXI interface, 10G MAC and XAUI
 *        C_XAUI_REVERSE=1 means the XAUI GTX lanes are reversed. This
 *        is used on NetFPGA-10G board Port 0, 1, 2. Please consult
 *        board schematic first before modifying the default value.
 *
 *        XAUI - 10G MAC - AXI
 *
 *  Copyright notice:
 *        Copyright (C) 2010, 2011 The Board of Trustees of The Leland Stanford
 *                                 Junior University
 *
 *  Licence:
 *        This file is part of the NetFPGA 10G development base package.
 *
 *        This file is free code: you can redistribute it and/or modify it under
 *        the terms of the GNU Lesser General Public License version 2.1 as
 *        published by the Free Software Foundation.
 *
 *        This package is distributed in the hope that it will be useful, but
 *        WITHOUT ANY WARRANTY; without even the implied warranty of
 *        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *        Lesser General Public License for more details.
 *
 *        You should have received a copy of the GNU Lesser General Public
 *        License along with the NetFPGA source package.  If not, see
 *        http://www.gnu.org/licenses/.
 *
 */
 
`define COLLECT_STATISTICS

`uselib lib=unisims_ver
`uselib lib=proc_common_v3_00_a

//`default_nettype none

module nf10_upb_10g_interface
#(
    parameter C_FAMILY = "virtex5",
    parameter C_XAUI_REVERSE=0,
    parameter C_XGMAC_CONFIGURATION = {5'b01000, 64'h358B000000000000},
    parameter C_XAUI_CONFIGURATION = 7'b0,
    parameter C_CHIPSCOPE_INSTANCE = 0
)
(
    // Part 1: System side signals
    // Global Ports
    input wire axi_resetn,

    input wire dclk,   //DRP Clock 50MHz
    input wire refclk, //GTX Clock 156.25MHz

    // Master Stream Ports
    output wire [63:0] m_axis_tdata,
    output wire [7:0] m_axis_tkeep, 
    output wire [0:0] m_axis_tuser,
    output wire m_axis_tvalid,
    input wire  m_axis_tready,
    output wire m_axis_tlast,

    // Slave Stream Ports
    input wire [63:0] s_axis_tdata,
    input wire [7:0] s_axis_tkeep,
    input wire [0:0] s_axis_tuser,
    input wire s_axis_tvalid,
    output wire s_axis_tready,
    input wire s_axis_tlast,


    // Part 2: PHY side signals
    // XAUI PHY Interface
    output wire        xaui_tx_l0_p,
    output wire        xaui_tx_l0_n,
    output wire        xaui_tx_l1_p,
    output wire        xaui_tx_l1_n,
    output wire        xaui_tx_l2_p,
    output wire        xaui_tx_l2_n,
    output wire        xaui_tx_l3_p,
    output wire        xaui_tx_l3_n,

    input wire         xaui_rx_l0_p,
    input wire         xaui_rx_l0_n,
    input wire         xaui_rx_l1_p,
    input wire         xaui_rx_l1_n,
    input wire         xaui_rx_l2_p,
    input wire         xaui_rx_l2_n,
    input wire         xaui_rx_l3_p,
    input wire         xaui_rx_l3_n,
    
    // flow control interface
    input wire pause_req,
    input wire [15:0] pause_val,

    output wire clk156
);

  wire txoutclk;
  wire clk156_locked;

  wire [63:0] xgmii_rxd, xgmii_txd;
  wire [ 7:0] xgmii_rxc, xgmii_txc;

  wire [63 : 0] tx_data;
  wire [7 : 0]  tx_data_valid;
  wire          tx_start;
  wire          tx_ack;

  wire [63 : 0] rx_data;
  wire [7 : 0]  rx_data_valid;

  wire          rx_good_frame;
  wire          rx_bad_frame;
  
  wire          tx_underrun;

  wire reset = ~axi_resetn;

  // =============================================================================
  // Module Instantiation
  // =============================================================================

  // Put system clocks on global routing
  BUFG clk156_bufg (
    .I(txoutclk),
    .O(clk156));

  xaui_block
  #(.WRAPPER_SIM_GTXRESET_SPEEDUP(1),
    .REVERSE_LANES(C_XAUI_REVERSE)
   ) xaui_block
  (
    .reset         (reset),
    .reset156      (reset),
    .clk156        (clk156),
    .dclk          (dclk),
    .refclk        (refclk),
    .txoutclk      (txoutclk),

    .xgmii_txd     (xgmii_txd),
    .xgmii_txc     (xgmii_txc),
    .xgmii_rxd     (xgmii_rxd),
    .xgmii_rxc     (xgmii_rxc),

    .xaui_tx_l0_p  (xaui_tx_l0_p),
    .xaui_tx_l0_n  (xaui_tx_l0_n),
    .xaui_tx_l1_p  (xaui_tx_l1_p),
    .xaui_tx_l1_n  (xaui_tx_l1_n),
    .xaui_tx_l2_p  (xaui_tx_l2_p),
    .xaui_tx_l2_n  (xaui_tx_l2_n),
    .xaui_tx_l3_p  (xaui_tx_l3_p),
    .xaui_tx_l3_n  (xaui_tx_l3_n),
    .xaui_rx_l0_p  (xaui_rx_l0_p),
    .xaui_rx_l0_n  (xaui_rx_l0_n),
    .xaui_rx_l1_p  (xaui_rx_l1_p),
    .xaui_rx_l1_n  (xaui_rx_l1_n),
    .xaui_rx_l2_p  (xaui_rx_l2_p),
    .xaui_rx_l2_n  (xaui_rx_l2_n),
    .xaui_rx_l3_p  (xaui_rx_l3_p),
    .xaui_rx_l3_n  (xaui_rx_l3_n),

    .txlock        (clk156_locked),
    .signal_detect (4'b1111),
    .drp_i         (16'h0),
    .drp_addr      (7'b0),
    .drp_en        (2'b0),
    .drp_we        (2'b0),
    .drp_o         (),
    .drp_rdy       (),
    .configuration_vector (C_XAUI_CONFIGURATION),
    .status_vector ()
  );


   ////////////////////////
   // Instantiate the MAC
   ////////////////////////
   xgmac xgmac
     (
      .reset                (reset),

      .tx_underrun          (tx_underrun),
      .tx_data              (tx_data),
      .tx_data_valid        (tx_data_valid),
      .tx_start             (tx_start),
      .tx_ack               (tx_ack),
      .tx_ifg_delay         (8'b0),
      .tx_statistics_vector (),
      .tx_statistics_valid  (),
      .pause_val            (pause_val),
      .pause_req            (pause_req),

      .rx_data              (rx_data),
      .rx_data_valid        (rx_data_valid),
      .rx_good_frame        (rx_good_frame),
      .rx_bad_frame         (rx_bad_frame),
      .rx_statistics_vector (),
      .rx_statistics_valid  (),

      .configuration_vector (C_XGMAC_CONFIGURATION),

      .tx_clk0(clk156),
      .tx_dcm_lock(clk156_locked),
      .xgmii_txd(xgmii_txd),
      .xgmii_txc(xgmii_txc),

      .rx_clk0(clk156),
      .rx_dcm_lock(clk156_locked),
      .xgmii_rxd(xgmii_rxd),
      .xgmii_rxc(xgmii_rxc)
      );

  // deleted rx_queue, tx_queue, converter_master, converter_slave, axi_lite_ipif_1bar, ipif_regs

  // Instantiate the xgmac_to_axi_converter
  xgmac_to_axi_converter xgmac_to_axi (
    .reset(reset), 
    .clk156(clk156), 
    .tx_data(tx_data), 
    .tx_data_valid(tx_data_valid), 
    .tx_start(tx_start), 
    .tx_ack(tx_ack), 
    .tx_underrun(tx_underrun), 
    .rx_data(rx_data), 
    .rx_data_valid(rx_data_valid), 
    .rx_good_frame(rx_good_frame), 
    .rx_bad_frame(rx_bad_frame), 
    .m_axis_tdata(m_axis_tdata), 
    .m_axis_tkeep(m_axis_tkeep), 
    .m_axis_tuser(m_axis_tuser), 
    .m_axis_tvalid(m_axis_tvalid), 
    .m_axis_tready(m_axis_tready), 
    .m_axis_tlast(m_axis_tlast), 
    .s_axis_tdata(s_axis_tdata), 
    .s_axis_tkeep(s_axis_tkeep), 
    .s_axis_tuser(s_axis_tuser), 
    .s_axis_tvalid(s_axis_tvalid), 
    .s_axis_tready(s_axis_tready), 
    .s_axis_tlast(s_axis_tlast)
  );
  
  `ifdef COLLECT_STATISTICS
    `include "../../../../nf10_upb_lib/hdl/verilog/tkeep_coder.v"
    localparam statistics_width = 64;
    
    reg [statistics_width-1:0] stat_good_frame = 0;
    reg [statistics_width-1:0] stat_bad_frame = 0;
    
    reg [statistics_width-1:0] stat_rx_pkts = 0;
    reg [statistics_width-1:0] stat_rx_bytes = 0;
    
    reg [statistics_width-1:0] stat_tx_pkts = 0;
    reg [statistics_width-1:0] stat_tx_bytes = 0;
	 
	 reg m_axis_tvalid_ff;
	 reg [7:0] m_axis_tkeep_ff;
	 reg m_axis_tready_ff;
	 reg m_axis_tlast_ff;
	 
	 reg s_axis_tvalid_ff;
	 reg [7:0] s_axis_tkeep_ff;
	 reg s_axis_tready_ff;
	 
	 reg rx_good_frame_ff;
	 reg rx_bad_frame_ff;
	 reg tx_start_ff;
    
    always @(posedge clk156) begin
		m_axis_tvalid_ff <= m_axis_tvalid;
		m_axis_tready_ff <= m_axis_tready;
		m_axis_tlast_ff <= m_axis_tlast;
	   m_axis_tkeep_ff <= m_axis_tkeep;
		
		s_axis_tvalid_ff <= s_axis_tvalid;
		s_axis_tready_ff <= s_axis_tready;
		s_axis_tkeep_ff <= s_axis_tkeep;
		
		rx_good_frame_ff <= rx_good_frame;
		rx_bad_frame_ff <= rx_bad_frame;		
		tx_start_ff <= tx_start;
		
      if (rx_good_frame_ff)
        stat_good_frame <= stat_good_frame + 1;
      if (rx_bad_frame_ff)
        stat_bad_frame <= stat_bad_frame + 1;
      if (m_axis_tvalid_ff && m_axis_tready_ff && m_axis_tlast_ff)
        stat_rx_pkts <= stat_rx_pkts + 1;
      if (m_axis_tvalid_ff && m_axis_tready_ff)
        stat_rx_bytes <= stat_rx_bytes + {59'b0, encode(m_axis_tkeep_ff)} + 64'b1;
      if (tx_start_ff)
        stat_tx_pkts <= stat_tx_pkts + 1;
      if (s_axis_tvalid_ff && s_axis_tready_ff)
        stat_tx_bytes <= stat_tx_bytes + {59'b0, encode(s_axis_tkeep_ff)} + 64'b1;
    end    
  `endif
	
endmodule

`undef COLLECT_STATISTICS
