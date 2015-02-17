/*******************************************************************************
 *
 *  NetFPGA-10G http://www.netfpga.org
 *
 *  File:
 *        dma.v
 *
 *  Library:
 *        hw/contrib/pcores/dma_v1_00_a
 *
 *  Module:
 *        dma
 *
 *  Author:
 *        Mario Flajslik
 *
 *        Modifications for the UPB OpenFlow Switch project:
 *         Jörg Niklas, osjsn@niklasfamily.de
 *         Project Group "On-the-Fly Networking for Big Data"
 *         Computer Engineering Group, University of Paderborn
 *
 *  Description:
 *        Top level module for the DMA engine. This module wraps PCIe TX/RX
 *        engine modules with the AXIS interface module. Only one AXIS interface
 *        is used to save on resources, instead TUSER signal is used to
 *        demultiplex between ethernet interfaces.
 *
 *  Copyright notice:
 *        Copyright (C) 2010, 2011 The Board of Trustees of The Leland Stanford
 *                                 Junior University
 *
 *        Modifications for the UPB OpenFlow Switch project:
 *          Copyright (c) 2014, 2015 Jörg Niklas, osjsn@niklasfamily.de
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

//`define DEBUG_CHIPSCOPE

`default_nettype none
`include "dma_defs.vh"

module dma_engine
  (
   // Tx
   output logic [63:0]  trn_td,
   output logic [7:0]   trn_trem_n,
   output logic         trn_tsof_n,
   output logic         trn_teof_n,
   output logic         trn_tsrc_rdy_n,
   output logic         trn_tsrc_dsc_n,
   input logic          trn_tdst_rdy_n,
   input logic          trn_tdst_dsc_n,
   output logic         trn_terrfwd_n,
   input logic [3:0]    trn_tbuf_av,
   
   // Rx
   input logic [63:0]   trn_rd,
   input logic [7:0]    trn_rrem_n,
   input logic          trn_rsof_n,
   input logic          trn_reof_n,
   input logic          trn_rsrc_rdy_n,
   input logic          trn_rsrc_dsc_n,
   output logic         trn_rdst_rdy_n,
   input logic          trn_rerrfwd_n,
   output logic         trn_rnp_ok_n,
   input logic [6:0]    trn_rbar_hit_n,
   input logic [7:0]    trn_rfc_nph_av,
   input logic [11:0]   trn_rfc_npd_av,
   input logic [7:0]    trn_rfc_ph_av,
   input logic [11:0]   trn_rfc_pd_av,
   output logic         trn_rcpl_streaming_n,
   
   // Host (CFG) Interface
   input logic [31:0]   cfg_do,
   input logic          cfg_rd_wr_done_n,
   output logic [31:0]  cfg_di,
   output logic [3:0]   cfg_byte_en_n,
   output logic [9:0]   cfg_dwaddr,
   output logic         cfg_wr_en_n,
   output logic         cfg_rd_en_n,
   output logic         cfg_err_cor_n,
   output logic         cfg_err_ur_n,
   output logic         cfg_err_ecrc_n,
   output logic         cfg_err_cpl_timeout_n,
   output logic         cfg_err_cpl_abort_n,
   output logic         cfg_err_cpl_unexpect_n,
   output logic         cfg_err_posted_n,
   output logic [47:0]  cfg_err_tlp_cpl_header,

   input logic          cfg_err_cpl_rdy_n,
   output logic         cfg_err_locked_n, 
   output logic         cfg_interrupt_n,
   input logic          cfg_interrupt_rdy_n,
   output logic         cfg_interrupt_assert_n,
   output logic [7:0]   cfg_interrupt_di,
   input logic [7:0]    cfg_interrupt_do,
   input logic [2:0]    cfg_interrupt_mmenable,
   input logic          cfg_interrupt_msienable,
   input logic          cfg_to_turnoff_n,
   output logic         cfg_pm_wake_n,
   input logic [2:0]    cfg_pcie_link_state_n,
   output logic         cfg_trn_pending_n,
   input logic [7:0]    cfg_bus_number,
   input logic [4:0]    cfg_device_number,
   input logic [2:0]    cfg_function_number,
   output logic [63:0]  cfg_dsn,
   input logic [15:0]   cfg_status,
   input logic [15:0]   cfg_command,
   input logic [15:0]   cfg_dstatus,
   input logic [15:0]   cfg_dcommand,
   input logic [15:0]   cfg_lstatus,
   input logic [15:0]   cfg_lcommand,
   
   // MAC tx
   output logic [63:0]  M_AXIS_TDATA,
   output logic [7:0]   M_AXIS_TSTRB,
   output logic         M_AXIS_TVALID,
   input logic          M_AXIS_TREADY,
   output logic         M_AXIS_TLAST,
   output logic [127:0] M_AXIS_TUSER,

   // MAC rx
   input logic [63:0]   S_AXIS_TDATA,
   input logic [7:0]    S_AXIS_TSTRB,
   input logic          S_AXIS_TVALID,
   output logic         S_AXIS_TREADY,
   input logic          S_AXIS_TLAST,
   input logic [127:0]  S_AXIS_TUSER,

   // AXI lite master core interface
   output logic         IP2Bus_MstRd_Req,
   output logic         IP2Bus_MstWr_Req,
   output logic [31:0]  IP2Bus_Mst_Addr,
   output logic [3:0]   IP2Bus_Mst_BE,
   output logic         IP2Bus_Mst_Lock,
   output logic         IP2Bus_Mst_Reset,
   input logic          Bus2IP_Mst_CmdAck,
   input logic          Bus2IP_Mst_Cmplt,
   input logic          Bus2IP_Mst_Error,
   input logic          Bus2IP_Mst_Rearbitrate,
   input logic          Bus2IP_Mst_Timeout,
   input logic [31:0]   Bus2IP_MstRd_d,
   input logic          Bus2IP_MstRd_src_rdy_n,
   output logic [31:0]  IP2Bus_MstWr_d,
   input logic          Bus2IP_MstWr_dst_rdy_n,
   
   // misc
   input logic          axi_clk,
   input logic          tx_clk,
   input logic          rx_clk,
   input logic          pcie_clk,
   input logic          rst,

   inout wire [35:0] chipscope_control_0,
   inout wire [35:0] chipscope_control_1
   );

   // pcie signals
   logic [15:0]          pcie_id;
   logic                 bus_master_en;
   logic [2:0]           max_read_req_size;
   logic [2:0]           max_payload_size;
   logic                 read_completion_bundary;

   always_ff @(posedge pcie_clk) begin
      pcie_id                 <= {cfg_bus_number, cfg_device_number, cfg_function_number};
      bus_master_en           <= cfg_command[2];
      max_read_req_size       <= cfg_dcommand[14:12];
      max_payload_size        <= cfg_dcommand[7:5];
      read_completion_bundary <= cfg_lcommand[3];
      cfg_dsn                 <= 'd1337;
   end

   // pcie_rx wires
   logic [1:0]                  wr_if_select;
   logic [3:0]                  wr_mem_select; 
   logic [`MEM_ADDR_BITS-1:0]   wr_addr_hi;
   logic [31:0]                 wr_data_hi;
   logic [3:0]                  wr_mask_hi;
   logic                        wr_en_hi;
   logic [`MEM_ADDR_BITS-1:0]   wr_addr_lo;
   logic [31:0]                 wr_data_lo;
   logic [3:0]                  wr_mask_lo;
   logic                        wr_en_lo;
   logic                        cm_q_wr_en;
   logic [`CM_Q_WIDTH-1:0]      cm_q_data;
   logic                        cm_q_almost_full;
   logic                        ort_req_v;
   logic [3:0]                  ort_req_tag;
   logic [1:0]                  ort_req_iface;
   logic [3:0]                  ort_req_mem;
   logic [`MEM_ADDR_BITS-1:0]   ort_req_addr;
   logic                        ort_next_tag_v;
   logic [3:0]                  ort_next_tag;
   logic                        iface_rdy;

   // pcie_tx wires
   logic [1:0]                  rd_if_select;
   logic [3:0]                  rd_mem_select;
   logic [`MEM_ADDR_BITS-1:0]   rd_addr_hi;
   logic                        rd_en_hi;
   logic [`MEM_ADDR_BITS-1:0]   rd_addr_lo;
   logic                        rd_en_lo;
   logic                        wr_q_req_v;
   logic [`PCIE_WR_Q_WIDTH-1:0] wr_q_req_data;
   logic                        wr_q_req_grant;
   logic                        rd_q_req_v;
   logic [`PCIE_RD_Q_WIDTH-1:0] rd_q_req_data;
   logic                        rd_q_req_grant;
   logic                        cm_q_req_v;
   logic [`PCIE_CM_Q_WIDTH-1:0] cm_q_req_data;
   logic                        cm_q_req_grant;

   // AXI lite master (axi_clk)
   logic [31:0]                 axi_rdwr_addr;
   logic [31:0]                 axi_rd_data;
   logic [31:0]                 axi_wr_data;
   logic                        axi_rd_go;
   logic                        axi_wr_go;
   logic                        axi_rd_done;
   logic                        axi_wr_done;
   logic                        axi_error;
   logic                        mem_cfg_rd_valid;

`ifdef DEBUG_PCIE
   // debug
   logic [63:0]                 debug_t_h1[15:0];
   logic [63:0]                 debug_t_h2[15:0];
   logic [63:0]                 debug_t_ts[15:0];
   logic [63:0]                 debug_r_h1[15:0];
   logic [63:0]                 debug_r_h2[15:0];
   logic [63:0]                 debug_r_ts[15:0];
`endif
   
   // stats
   logic [63:0]                 stat_pcie_rx_ts;
   logic [31:0]                 stat_pcie_rx_word_cnt;
   logic [31:0]                 stat_pcie_rx_wr_cnt;
   logic [31:0]                 stat_pcie_rx_rd_cnt;
   logic [31:0]                 stat_pcie_rx_cm_cnt;
   logic [31:0]                 stat_pcie_rx_err_cnt;
   logic [63:0]                 stat_pcie_tx_ts;
   logic [31:0]                 stat_pcie_tx_word_cnt;
   logic [31:0]                 stat_pcie_tx_wr_cnt;
   logic [31:0]                 stat_pcie_tx_rd_cnt;
   logic [31:0]                 stat_pcie_tx_cm_cnt;
   logic [31:0]                 stat_pcie_tx_err_cnt;

   logic [31:0]                 rd_data_lo;
   logic [31:0]                 rd_data_hi;

	// synchronization (I do not trust the async code in the dma...)

	wire [63:0]	m_axis_tdata_pcie_clk;
	wire [7:0]	m_axis_tstrb_pcie_clk;
	wire		m_axis_tvalid_pcie_clk;
	wire		m_axis_tready_pcie_clk;
	wire		m_axis_tlast_pcie_clk;
	wire [31:0]	m_axis_tuser_pcie_clk; // slightly reduced tuser... (upper bits are unused)

	wire [63:0]	s_axis_tdata_pcie_clk;
	wire [7:0]	s_axis_tstrb_pcie_clk;
	wire		s_axis_tvalid_pcie_clk;
	wire		s_axis_tready_pcie_clk;
	wire		s_axis_tlast_pcie_clk;
	wire [31:0]	s_axis_tuser_pcie_clk;

	wire tx_wr_full, tx_rd_empty, rx_wr_full, rx_rd_empty;

	simple_packet_fifo #(

		.data_width							($bits(m_axis_tdata_pcie_clk) + $bits(m_axis_tstrb_pcie_clk)),
		.metadata_width						($bits(m_axis_tuser_pcie_clk)),
		.store_and_forward					(0),
		.almost_full_offset					(9'h080),
		.almost_full_offset_metadata_fifo	(9'h080)

	) tx_fifo (

		.wr_clk				(pcie_clk),
		.wr_reset			(rst),
		.wr_data			({ m_axis_tdata_pcie_clk , m_axis_tstrb_pcie_clk }),
		.wr_metadata		(m_axis_tuser_pcie_clk),
		.wr_last			(m_axis_tlast_pcie_clk),
		.wr_dirty			(1'b0),
		.wr					(m_axis_tvalid_pcie_clk & m_axis_tready_pcie_clk),
		.wr_full			(tx_wr_full),
		.wr_almost_full		(),
	
		.rd_clk				(tx_clk),
		.rd_reset			(rst),
		.rd_data			( {M_AXIS_TDATA, M_AXIS_TSTRB }),
		.rd_metadata		(M_AXIS_TUSER),
		.rd_last			(M_AXIS_TLAST),
		.rd					(M_AXIS_TVALID & M_AXIS_TREADY),
		.rd_empty			(tx_rd_empty),
	
		.error				()
	);

	assign m_axis_tready_pcie_clk = ~tx_wr_full;
	assign M_AXIS_TVALID = ~tx_rd_empty;

	simple_packet_fifo #(

		.data_width							($bits(S_AXIS_TDATA) + $bits(S_AXIS_TSTRB)),
		.metadata_width						($bits(S_AXIS_TUSER)),
		.store_and_forward					(0),
		.almost_full_offset					(9'h080),
		.almost_full_offset_metadata_fifo	(9'h080)

	) rx_fifo (

		.wr_clk				(rx_clk),
		.wr_reset			(rst),
		.wr_data			({ S_AXIS_TDATA, S_AXIS_TSTRB }),
		.wr_metadata		(S_AXIS_TUSER),
		.wr_last			(S_AXIS_TLAST),
		.wr_dirty			(1'b0),
		.wr					(S_AXIS_TVALID & S_AXIS_TREADY),
		.wr_full			(rx_wr_full),
		.wr_almost_full		(),
	
		.rd_clk				(pcie_clk),
		.rd_reset			(rst),
		.rd_data			({ s_axis_tdata_pcie_clk, s_axis_tstrb_pcie_clk }),
		.rd_metadata		(s_axis_tuser_pcie_clk),
		.rd_last			(s_axis_tlast_pcie_clk),
		.rd					(s_axis_tvalid_pcie_clk & s_axis_tready_pcie_clk),
		.rd_empty			(rx_rd_empty),
	
		.error				()
	);

	assign S_AXIS_TREADY = ~rx_wr_full;
	assign s_axis_tvalid_pcie_clk = ~rx_rd_empty;


	// dma channel
	iface #(
		.IFACE_ID(0)
	) u_iface (

		.tx_clk(pcie_clk),
		.M_AXIS_TDATA(m_axis_tdata_pcie_clk),
		.M_AXIS_TSTRB(m_axis_tstrb_pcie_clk),
		.M_AXIS_TVALID(m_axis_tvalid_pcie_clk),
		.M_AXIS_TREADY(m_axis_tready_pcie_clk),
		.M_AXIS_TLAST(m_axis_tlast_pcie_clk),
		.M_AXIS_TUSER(m_axis_tuser_pcie_clk),

		.rx_clk(pcie_clk),
		.S_AXIS_TDATA(s_axis_tdata_pcie_clk),
		.S_AXIS_TSTRB(s_axis_tstrb_pcie_clk),
		.S_AXIS_TVALID(s_axis_tvalid_pcie_clk),
		.S_AXIS_TREADY(s_axis_tready_pcie_clk),
		.S_AXIS_TLAST(s_axis_tlast_pcie_clk),
		.S_AXIS_TUSER(s_axis_tuser_pcie_clk),

		.chipscope_control_0(chipscope_control_1),

		.*
	);

	// instantiate pcie engines
	pcie_rx u_pcie_rx(.*);

	pcie_tx u_pcie_tx (
		.tx_clk(pcie_clk),
		.rx_clk(pcie_clk),
		.*
	);

   // instantiate AXI lite Master interface
   axi u_axi(.*);

   // pcie signal tie-off
   assign trn_tsrc_dsc_n = 1;
   assign trn_terrfwd_n = 1;
   assign cfg_di = 0;
   assign cfg_byte_en_n = 4'hf;
   assign cfg_dwaddr = 0;
   assign cfg_wr_en_n = 1;
   assign cfg_rd_en_n = 1;
   assign cfg_err_cor_n = 1;
   assign cfg_err_ur_n = 1;
   assign cfg_err_ecrc_n = 1;
   assign cfg_err_cpl_timeout_n = 1;
   assign cfg_err_cpl_abort_n = 1;
   assign cfg_err_cpl_unexpect_n = 1;
   assign cfg_err_posted_n = 1;
   assign cfg_err_tlp_cpl_header = 0;
   assign cfg_err_locked_n = 1;
   assign cfg_interrupt_assert_n = 1;
   assign cfg_pm_wake_n = 1;


`ifdef DEBUG_CHIPSCOPE

wire [127:0] data = {

	axi_rd_go,
	axi_rd_done,
	axi_rd_data, // 32 bit

	axi_wr_go,
	axi_wr_data, // 32 bit
	axi_wr_done,

	axi_rdwr_addr, // 32 bit

	axi_error,
	mem_cfg_rd_valid
};

wire [15:0] trig = {

	axi_rd_go,
	axi_rd_done,
	axi_wr_go,
	axi_wr_done,
	axi_error,
	mem_cfg_rd_valid
};

reg [127:0] data_reg;
reg [15:0] trig_reg;

always @(posedge axi_clk) begin
	data_reg <= data;
	trig_reg <= trig;
end

chipscope_ila_128 axi_lite_ila_inst (
	.CONTROL(chipscope_control_0),
	.CLK(axi_clk),
	.DATA(data_reg),
	.TRIG0(trig_reg)
);

`else

assign chipscope_control_0 = 0;

`endif
   
endmodule

`ifdef DEBUG_CHIPSCOPE

module chipscope_ila_128 (
	inout [35:0] CONTROL,
	input CLK,
	input [127:0] DATA,
	input [15:0] TRIG0
);
endmodule

`endif

`undef DEBUG_CHIPSCOPE

