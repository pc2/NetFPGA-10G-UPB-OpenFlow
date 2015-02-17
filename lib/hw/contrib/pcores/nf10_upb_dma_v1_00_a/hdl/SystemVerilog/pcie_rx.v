/*******************************************************************************
 *
 *  NetFPGA-10G http://www.netfpga.org
 *
 *  File:
 *        pcie_rx.v
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
 *  Description:
 *        Top level PCIe receive module that wraps read, write and completion
 *        receive submodules.
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

`include "dma_defs.vh"

module pcie_rx
  (
   // PCIe Transaction RX
   input logic [63:0]                trn_rd,
   input logic [7:0]                 trn_rrem_n,
   input logic                       trn_rsof_n,
   input logic                       trn_reof_n,
   input logic                       trn_rsrc_rdy_n,
   input logic                       trn_rerrfwd_n,
   input logic [6:0]                 trn_rbar_hit_n,
   output logic                      trn_rdst_rdy_n,
   output logic                      trn_rnp_ok_n,
   output logic                      trn_rcpl_streaming_n,
   output logic                      cfg_trn_pending_n,

   input logic                       iface_rdy,
   
   // memory write interface
   output logic [1:0]                wr_if_select,
   output logic [3:0]                wr_mem_select,
   output logic [`MEM_ADDR_BITS-1:0] wr_addr_hi,
   output logic [31:0]               wr_data_hi,
   output logic [3:0]                wr_mask_hi,
   output logic                      wr_en_hi,
   output logic [`MEM_ADDR_BITS-1:0] wr_addr_lo,
   output logic [31:0]               wr_data_lo,
   output logic [3:0]                wr_mask_lo,
   output logic                      wr_en_lo,

   // completion fifo interface
   output logic                      cm_q_wr_en,
   output logic [`CM_Q_WIDTH-1:0]    cm_q_data,
   input logic [`NUM_PORTS-1:0]      cm_q_almost_full,

   // ORT request interface
   input logic                       ort_req_v,
   input logic [3:0]                 ort_req_tag,
   input logic [1:0]                 ort_req_iface,
   input logic [3:0]                 ort_req_mem,
   input logic [`MEM_ADDR_BITS-1:0]  ort_req_addr,
   output logic                      ort_next_tag_v,
   output logic [3:0]                ort_next_tag,

`ifdef DEBUG_PCIE
   // debug
   output logic [63:0]               debug_r_h1[7:0],
   output logic [63:0]               debug_r_h2[7:0],
   output logic [63:0]               debug_r_ts[7:0],
`endif
   
   // stats
   output logic [63:0]               stat_pcie_rx_ts,
   output logic [31:0]               stat_pcie_rx_word_cnt,
   output logic [31:0]               stat_pcie_rx_wr_cnt,
   output logic [31:0]               stat_pcie_rx_rd_cnt,
   output logic [31:0]               stat_pcie_rx_cm_cnt,
   output logic [31:0]               stat_pcie_rx_err_cnt,
   
   // misc
   input logic                       pcie_clk,
   input logic                       rst
   );

   // flop the reset to help timing
   logic                             rst_reg, rst_d1, rst_d2;
   always_ff @(posedge pcie_clk) begin
      rst_d1  <= rst;
      rst_d2  <= rst_d1;
      rst_reg <= rst_d1 | rst_d2;
   end
   
   // assign some pcie signals
   assign trn_rdst_rdy_n               = ~iface_rdy;
   always_ff @(posedge pcie_clk) trn_rnp_ok_n <= |cm_q_almost_full;
   assign trn_rcpl_streaming_n         = 0;
   
   
   logic [63:0]    time_stamp;

`ifdef DEBUG_PCIE
   // debug
   logic [63:0]    debug_h1_nxt[7:0];
   logic [63:0]    debug_h2_nxt[7:0];
   logic [63:0]    debug_ts_nxt[7:0];
   logic [2:0]     debug_ptr, debug_ptr_nxt;
`endif
   
   // stats
   logic           stat_pcie_rx_word_cnt_inc;
   logic           stat_pcie_rx_wr_cnt_inc;
   logic           stat_pcie_rx_rd_cnt_inc;
   logic           stat_pcie_rx_cm_cnt_inc;
   logic           stat_pcie_rx_err_cnt_inc;

   // write interfaces
   logic [1:0]                wr_if_select_wr,  wr_if_select_cm;
   logic [3:0]                wr_mem_select_wr, wr_mem_select_cm;
   logic [`MEM_ADDR_BITS-1:0] wr_addr_hi_wr,    wr_addr_hi_cm;
   logic [31:0]               wr_data_hi_wr,    wr_data_hi_cm;
   logic [3:0]                wr_mask_hi_wr,    wr_mask_hi_cm;
   logic                      wr_en_hi_wr,      wr_en_hi_cm;
   logic [`MEM_ADDR_BITS-1:0] wr_addr_lo_wr,    wr_addr_lo_cm;
   logic [31:0]               wr_data_lo_wr,    wr_data_lo_cm;
   logic [3:0]                wr_mask_lo_wr,    wr_mask_lo_cm;
   logic                      wr_en_lo_wr,      wr_en_lo_cm;

   // stats
   always_ff @(posedge pcie_clk) stat_pcie_rx_word_cnt_inc <= ~trn_rsrc_rdy_n & trn_rerrfwd_n;
   

   pcie_rx_wr u_pcie_rx_wr
     (.wr_if_select(wr_if_select_wr),
      .wr_mem_select(wr_mem_select_wr),
      .wr_addr_hi(wr_addr_hi_wr),
      .wr_data_hi(wr_data_hi_wr),
      .wr_mask_hi(wr_mask_hi_wr),
      .wr_en_hi(wr_en_hi_wr),
      .wr_addr_lo(wr_addr_lo_wr),
      .wr_data_lo(wr_data_lo_wr),
      .wr_mask_lo(wr_mask_lo_wr),
      .wr_en_lo(wr_en_lo_wr),
      .rst(rst_reg),
      .*);

   pcie_rx_rd u_pcie_rx_rd(.rst(rst_reg),.*);

   pcie_rx_cm u_pcie_rx_cm
     (.wr_if_select(wr_if_select_cm),
      .wr_mem_select(wr_mem_select_cm),
      .wr_addr_hi(wr_addr_hi_cm),
      .wr_data_hi(wr_data_hi_cm),
      .wr_mask_hi(wr_mask_hi_cm),
      .wr_en_hi(wr_en_hi_cm),
      .wr_addr_lo(wr_addr_lo_cm),
      .wr_data_lo(wr_data_lo_cm),
      .wr_mask_lo(wr_mask_lo_cm),
      .wr_en_lo(wr_en_lo_cm),
      .rst(rst_reg),
      .*);

   // mux write signals
   always_comb begin      
      if(wr_en_hi_wr | wr_en_lo_wr) begin
         wr_if_select  = wr_if_select_wr;
         wr_mem_select = wr_mem_select_wr;      
         wr_addr_lo = wr_addr_lo_wr;
         wr_data_lo = wr_data_lo_wr;
         wr_mask_lo = wr_mask_lo_wr;
         wr_addr_hi = wr_addr_hi_wr;
         wr_data_hi = wr_data_hi_wr;
         wr_mask_hi = wr_mask_hi_wr;
         wr_en_hi = wr_en_hi_wr;
         wr_en_lo = wr_en_lo_wr;
      end
      else begin
         wr_if_select  = wr_if_select_cm;
         wr_mem_select = wr_mem_select_cm;      
         wr_addr_lo = wr_addr_lo_cm;
         wr_data_lo = wr_data_lo_cm;
         wr_mask_lo = wr_mask_lo_cm;
         wr_addr_hi = wr_addr_hi_cm;
         wr_data_hi = wr_data_hi_cm;
         wr_mask_hi = wr_mask_hi_cm;
         wr_en_hi = wr_en_hi_cm;
         wr_en_lo = wr_en_lo_cm;
      end
   end

`ifdef DEBUG_PCIE
   logic [63:0] trn_rd_debug_d1,         trn_rd_debug_d2;
   logic        trn_rsof_n_debug_d1,     trn_rsof_n_debug_d2;
   logic        trn_rsrc_rdy_n_debug_d1, trn_rsrc_rdy_n_debug_d2;
   logic        trn_rbar_debug_d1,       trn_rbar_debug_d2;
   
   always_ff @(posedge pcie_clk) begin
      trn_rd_debug_d1 <= trn_rd;
      trn_rd_debug_d2 <= trn_rd_debug_d1;

      trn_rsof_n_debug_d1 <= trn_rsof_n;
      trn_rsof_n_debug_d2 <= trn_rsof_n_debug_d1;

      trn_rsrc_rdy_n_debug_d1 <= trn_rsrc_rdy_n;
      trn_rsrc_rdy_n_debug_d2 <= trn_rsrc_rdy_n_debug_d1;

      trn_rbar_debug_d1 <= trn_rbar_hit_n[2];
      trn_rbar_debug_d2 <= trn_rbar_debug_d1;
   end
   
   always_comb begin
      debug_h1_nxt  = debug_r_h1;
      debug_h2_nxt  = debug_r_h2;
      debug_ts_nxt  = debug_r_ts;
      debug_ptr_nxt = debug_ptr;

      if(!trn_rsrc_rdy_n_debug_d2 && !trn_rsof_n_debug_d2 && !trn_rbar_debug_d2) begin
         debug_h1_nxt[debug_ptr] = trn_rd_debug_d2;
         debug_h2_nxt[debug_ptr] = trn_rd_debug_d1;
         debug_ts_nxt[debug_ptr] = time_stamp;
         debug_ptr_nxt = debug_ptr + 1;
      end
   end
`endif
   
   
   always_ff @(posedge pcie_clk) begin
      integer i;
      if(rst_reg) begin

`ifdef DEBUG_PCIE
         // debug
         for(i=0; i<16; i++) begin
            debug_r_h1[i] <= 0;
            debug_r_h2[i] <= 0;
            debug_r_ts[i] <= 0;
         end
         debug_ptr   <= 0;
`endif
         
         // stats
         stat_pcie_rx_ts       <= 0;
         stat_pcie_rx_word_cnt <= 0;
         stat_pcie_rx_wr_cnt   <= 0;
         stat_pcie_rx_rd_cnt   <= 0;
         stat_pcie_rx_cm_cnt   <= 0;
         stat_pcie_rx_err_cnt  <= 0;

         time_stamp <= 0;

      end
      else begin

`ifdef DEBUG_PCIE
         // debug
         debug_r_h1 <= debug_h1_nxt;
         debug_r_h2 <= debug_h2_nxt;
         debug_r_ts <= debug_ts_nxt;
         debug_ptr  <= debug_ptr_nxt;
`endif         
         // stats
         if(stat_pcie_rx_word_cnt_inc) stat_pcie_rx_word_cnt <= stat_pcie_rx_word_cnt + 1;
         if(stat_pcie_rx_wr_cnt_inc)   stat_pcie_rx_wr_cnt   <= stat_pcie_rx_wr_cnt + 1;
         if(stat_pcie_rx_rd_cnt_inc)   stat_pcie_rx_rd_cnt   <= stat_pcie_rx_rd_cnt + 1;
         if(stat_pcie_rx_cm_cnt_inc)   stat_pcie_rx_cm_cnt   <= stat_pcie_rx_cm_cnt + 1;
         if(stat_pcie_rx_err_cnt_inc)  stat_pcie_rx_err_cnt  <= stat_pcie_rx_err_cnt + 1;
         if(stat_pcie_rx_word_cnt_inc | stat_pcie_rx_wr_cnt_inc | stat_pcie_rx_rd_cnt_inc |
            stat_pcie_rx_cm_cnt_inc   | stat_pcie_rx_err_cnt_inc) stat_pcie_rx_ts <= time_stamp;
         
         time_stamp  <= time_stamp + 1;
      end
      
   end
endmodule
