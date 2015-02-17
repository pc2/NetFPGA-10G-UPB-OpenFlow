/*******************************************************************************
 *
 *  NetFPGA-10G http://www.netfpga.org
 *
 *  File:
 *        pcie_tx_rd.v
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
 *        PCIe read transmit module. Once this module is granted access
 *        to the PCIe core interface it drives the next read pakcet onto
 *        that interface.
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

module pcie_tx_rd
  (
   // PCIe Transaction TX
   output logic [63:0]                trn_td,
   output logic [7:0]                 trn_trem_n,
   output logic                       trn_tsof_n,
   output logic                       trn_teof_n,
   output logic                       trn_tsrc_rdy_n,

   // PCIe misc
   input logic [15:0]                 pcie_id,

   // control
   output logic                       pcie_req_v,
   input logic                        pcie_req_grant,
   input logic                        pcie_req_stall,
   output logic                       pcie_req_done,

   // read queue interfaces
   input logic [`NUM_PORTS-1:0]       rd_q_req_v,
   input logic [`PCIE_RD_Q_WIDTH-1:0] rd_q_req_data[`NUM_PORTS-1:0],
   output logic [`NUM_PORTS-1:0]      rd_q_req_grant,

   // ORT request interface
   output logic                       ort_req_v,
   output logic [3:0]                 ort_req_tag,
   output logic [1:0]                 ort_req_iface,
   output logic [3:0]                 ort_req_mem,
   output logic [`MEM_ADDR_BITS-1:0]  ort_req_addr,
   input logic                        ort_next_tag_v,
   input logic [3:0]                  ort_next_tag,

   // misc
   input logic                        tx_clk,
   input logic                        pcie_clk,
   input logic                        rst

   );

   logic                              rd_q_deq_en;
   logic [`PCIE_RD_Q_WIDTH-1:0]       rd_q_deq_data;
   logic                              rd_q_empty;

   logic                              ort_next_tag_v_reg;

   assign pcie_req_v = ~rd_q_empty & ort_next_tag_v_reg;

   // -----------------------------------
   // -- Read Queue
   // -----------------------------------
   // [88+:`MEM_ADDR_BITS] address
   // [87:86] if_select
   // [85:82] mem_select
   // [81:18] host addr
   // [17:8]  dw_length
   // [7:4]   first DW BE
   // [3:0]   last DW BE

   
   pcie_tx_q #(.WIDTH(`PCIE_RD_Q_WIDTH), .DEPTH(`PCIE_RD_Q_DEPTH),
               .NUM_PORTS(`NUM_PORTS), .PORT_BITS(`PORT_BITS))
   u_rd_q(.req_v(rd_q_req_v),
          .req_data(rd_q_req_data),
          .req_grant(rd_q_req_grant),
          .deq_en(rd_q_deq_en),
          .deq_data(rd_q_deq_data),
          .empty(rd_q_empty),
          .enq_clk(tx_clk),
          .deq_clk(pcie_clk),
          .rst(rst));

   
   // -----------------------------------
   // -- PCIe transmit logic
   // -----------------------------------
   localparam STATE_HEADER1 = 0;
   localparam STATE_HEADER2 = 1;

   logic                              state,             state_nxt;
   logic [63:0]                       head2,             head2_nxt;
   logic [10:0]                       dw_count,          dw_count_nxt;
   logic                              double_last,       double_last_nxt;

   logic [3:0]                        ort_next_tag_reg;
   logic                              ort_req_v_nxt;
   logic [3:0]                        ort_req_tag_nxt;
   logic [1:0]                        ort_req_iface_nxt;
   logic [3:0]                        ort_req_mem_nxt;
   logic [`MEM_ADDR_BITS-1:0]         ort_req_addr_nxt;

   logic [63:0]                       trn_td_reg;
   logic [7:0]                        trn_trem_n_reg;
   logic                              trn_tsof_n_reg;
   logic                              trn_teof_n_reg;
   logic                              trn_tsrc_rdy_n_reg;

   // address generation pipeline stage
   always_comb begin
      state_nxt = state;
      head2_nxt = head2;
      dw_count_nxt    = dw_count;
      double_last_nxt = double_last;

      rd_q_deq_en = 0;
      

      if(pcie_req_stall && (state != STATE_HEADER1)) begin      
         trn_td         = trn_td_reg;
         trn_tsof_n     = trn_tsof_n_reg;
         trn_teof_n     = trn_teof_n_reg;
         trn_trem_n     = trn_trem_n_reg;
         trn_tsrc_rdy_n = trn_tsrc_rdy_n_reg;
      end
      else begin      
         trn_td         = 0;
         trn_tsof_n     = 1;
         trn_teof_n     = 1;
         trn_trem_n     = 'hff;
         trn_tsrc_rdy_n = 1;
      end

      ort_req_v_nxt     = 0;
      ort_req_tag_nxt   = ort_req_tag;
      ort_req_iface_nxt = ort_req_iface;
      ort_req_mem_nxt   = ort_req_mem;
      ort_req_addr_nxt  = ort_req_addr;

      pcie_req_done = 0;
      
      if(pcie_req_grant & ~pcie_req_stall) begin
         case(state)
           STATE_HEADER1: begin     
              // ------------ 
              // *** READ *** 
              // ------------ 
              
              dw_count_nxt = 0;

              if(rd_q_deq_data[81-:32] == 32'b0) begin // 3DW
                 trn_td[63:56] = {8'b00000000};
                 head2_nxt[63:0] = {rd_q_deq_data[49:20], 2'b0, 32'b0};
                 double_last_nxt = 0;
              end
              else begin // 4DW
                 trn_td[63:56] = {8'b00100000};
                 head2_nxt[63:0] = {rd_q_deq_data[81:20], 2'b0};
                 double_last_nxt = 1;
              end
              
              trn_td[55:32] = {8'b0, 6'b0, rd_q_deq_data[17:8]};
              trn_td[31:0]  = {pcie_id, 3'b0, 1'b0, ort_next_tag_reg, rd_q_deq_data[3:0], rd_q_deq_data[7:4]};
              
              rd_q_deq_en = 1;
              
              trn_tsof_n = 0;
              trn_teof_n = 1;
              trn_trem_n = 'h00;
              trn_tsrc_rdy_n = 0;

              ort_req_v_nxt     = 1;
              ort_req_tag_nxt   = ort_next_tag_reg;
              ort_req_iface_nxt = rd_q_deq_data[87:86];
              ort_req_mem_nxt   = rd_q_deq_data[85:82];
              ort_req_addr_nxt  = rd_q_deq_data[88+:`MEM_ADDR_BITS];
              
              state_nxt = STATE_HEADER2;

           end

           STATE_HEADER2: begin

              trn_td = head2;              
              trn_tsof_n = 1;
              trn_tsrc_rdy_n = 0;
              
              if(double_last)
                trn_trem_n = 'h00;
              else
                trn_trem_n = 'h0f;
              
              trn_teof_n = 0;
              state_nxt = STATE_HEADER1;
              pcie_req_done = 1;
           end
         endcase

      end
   end

   always_ff @(posedge pcie_clk) begin
      integer i;
      if(rst) begin
         state     <= STATE_HEADER1;
         ort_req_v <= 0;
      end
      else begin
         state     <= state_nxt;
         ort_req_v <= ort_req_v_nxt;
      end
      
      dw_count <= dw_count_nxt;

      head2             <= head2_nxt;
      double_last       <= double_last_nxt;
      
      ort_next_tag_v_reg <= ort_next_tag_v;
      ort_next_tag_reg   <= ort_next_tag;

      ort_req_tag   <= ort_req_tag_nxt;
      ort_req_iface <= ort_req_iface_nxt;
      ort_req_mem   <= ort_req_mem_nxt;
      ort_req_addr  <= ort_req_addr_nxt;

      if(pcie_req_grant & ~pcie_req_stall) begin
         trn_td_reg         <= trn_td;
         trn_tsof_n_reg     <= trn_tsof_n;
         trn_teof_n_reg     <= trn_teof_n;
         trn_trem_n_reg     <= trn_trem_n;
         trn_tsrc_rdy_n_reg <= trn_tsrc_rdy_n;
      end

   end
   
endmodule
