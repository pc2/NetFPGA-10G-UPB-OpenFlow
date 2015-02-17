/*******************************************************************************
 *
 *  NetFPGA-10G http://www.netfpga.org
 *
 *  File:
 *        pcie_tx_cm.v
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
 *        PCIe completion transmit module. Once this module is granted access
 *        to the PCIe core interface it drives the next completion pakcet onto
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

module pcie_tx_cm
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
   output logic [2:0]                 ppl_ctrl,

   // memory reads
   output logic [1:0]                 rd_if_select,
   output logic [3:0]                 rd_mem_select,
   output logic [`MEM_ADDR_BITS-1:0]  rd_addr_hi,
   output logic                       rd_en_hi,
   output logic [`MEM_ADDR_BITS-1:0]  rd_addr_lo,
   output logic                       rd_en_lo,

   // completion queue interfaces
   input logic [`NUM_PORTS-1:0]       cm_q_req_v,
   input logic [`PCIE_CM_Q_WIDTH-1:0] cm_q_req_data[`NUM_PORTS-1:0],
   output logic [`NUM_PORTS-1:0]      cm_q_req_grant,
   
   // AXI rd ready qualifier
   input logic                        mem_cfg_rd_valid,

   // misc
   input logic                        rx_clk,
   input logic                        pcie_clk,
   input logic                        rst
   );

   logic                        cm_q_deq_en;
   logic [`PCIE_CM_Q_WIDTH-1:0] cm_q_deq_data;
   logic                        cm_q_empty;

   assign pcie_req_v = ~cm_q_empty & mem_cfg_rd_valid;
   
   // -----------------------------------
   // -- Completion Queue
   // -----------------------------------
   // [60+:`MEM_ADDR_BITS] address
   // [59:59] timeout
   // [58:57] if_select
   // [56:53] mem_select
   // [52:43] dw_len
   // [42:31] byte_len
   // [30:15] requester_id
   // [14:7]  tag
   // [6:0]   low_addr

   pcie_tx_q #(.WIDTH(`PCIE_CM_Q_WIDTH), .DEPTH(`PCIE_CM_Q_DEPTH),
               .NUM_PORTS(`NUM_PORTS), .PORT_BITS(`PORT_BITS))
   u_cm_q(.req_v(cm_q_req_v),
          .req_data(cm_q_req_data),
          .req_grant(cm_q_req_grant),
          .deq_en(cm_q_deq_en),
          .deq_data(cm_q_deq_data),
          .empty(cm_q_empty),
          .enq_clk(rx_clk),
          .deq_clk(pcie_clk),
          .rst(rst));

   
   // -----------------------------------
   // -- PCIe transmit logic
   // -----------------------------------
   localparam STATE_HEADER1 = 0;
   localparam STATE_HEADER2 = 1;
   localparam STATE_BODY    = 2;

   localparam OP_RD_3DW_ODD  = 0;
   localparam OP_RD_3DW_EVEN = 1;

   logic [1:0]                state,             state_nxt;
   logic                      op,                op_nxt;
   logic [63:0]               head2,             head2_nxt;
   logic [`MEM_ADDR_BITS-1:0] addr,              addr_nxt_1, addr_nxt_2, addr_nxt_3;
   logic [10:0]               dw_count,          dw_count_nxt_1, dw_count_nxt_2, dw_count_nxt_3;
   logic                      double_last,       double_last_nxt;
   logic [1:0]                rd_if_select_reg,  rd_if_select_reg_nxt;
   logic [3:0]                rd_mem_select_reg, rd_mem_select_reg_nxt;
   logic [`MEM_ADDR_BITS-1:0] rd_addr_hi_reg, rd_addr_lo_reg;

   logic [63:0]               trn_td_reg;
   logic [7:0]                trn_trem_n_reg;
   logic                      trn_tsof_n_reg;
   logic                      trn_teof_n_reg;
   logic                      trn_tsrc_rdy_n_reg;
   logic [2:0]                ppl_ctrl_reg;

   logic [1:0]                ppl_mux_select; // 1 - head1-completion; 2 - head2; 3 - body;
   
   // address generation pipeline stage
   always_comb begin
      state_nxt = state;
      op_nxt    = op;
      head2_nxt = head2;
      addr_nxt_1  = addr;
      addr_nxt_2  = addr;
      addr_nxt_3  = addr;
      rd_if_select_reg_nxt   = rd_if_select_reg;
      rd_mem_select_reg_nxt  = rd_mem_select_reg;
      dw_count_nxt_1    = dw_count;
      dw_count_nxt_2    = dw_count;
      dw_count_nxt_3    = dw_count;
      double_last_nxt = double_last;

      rd_if_select  = rd_if_select_reg;
      rd_mem_select = rd_mem_select_reg;
      
      cm_q_deq_en = 0;

      rd_addr_hi = rd_addr_hi_reg;
      rd_addr_lo = rd_addr_lo_reg;
      rd_en_hi   = 1;
      rd_en_lo   = 1;
      
      if(pcie_req_stall && (state != STATE_HEADER1)) begin      
         trn_td         = trn_td_reg;
         trn_tsof_n     = trn_tsof_n_reg;
         trn_teof_n     = trn_teof_n_reg;
         trn_trem_n     = trn_trem_n_reg;
         trn_tsrc_rdy_n = trn_tsrc_rdy_n_reg;
         ppl_ctrl       = ppl_ctrl_reg;
      end
      else begin      
         trn_td         = 0;
         trn_tsof_n     = 1;
         trn_teof_n     = 1;
         trn_trem_n     = 'hff;
         trn_tsrc_rdy_n = 1;
         ppl_ctrl       = 0;
      end

      ppl_ctrl = 0;
      ppl_mux_select = 0;

      pcie_req_done = 0;
      
      if(pcie_req_grant & ~pcie_req_stall) begin
         case(state)
           STATE_HEADER1: begin                   
              // ------------------ 
              // *** COMPLETION *** 
              // ------------------

              ppl_mux_select = 1;

              trn_td[63:32] = {8'b01001010, 8'b0, 6'b0, cm_q_deq_data[52:43]};
              trn_td[31:0] = {pcie_id, 4'b0, cm_q_deq_data[42:31]};
              head2_nxt[63:32] = {cm_q_deq_data[30:15], cm_q_deq_data[14:7], 1'b0, cm_q_deq_data[6:0]};
              head2_nxt[31:0] = 0;
              
              addr_nxt_1 = cm_q_deq_data[59+:`MEM_ADDR_BITS] + 4;
              dw_count_nxt_1 = {1'b0, cm_q_deq_data[52:43] - 10'h1};
              rd_if_select_reg_nxt  = cm_q_deq_data[58:57];
              rd_if_select   = cm_q_deq_data[58:57];
              rd_mem_select_reg_nxt = cm_q_deq_data[56:53];
              rd_mem_select  = cm_q_deq_data[56:53];
              
              rd_addr_hi = cm_q_deq_data[59+:`MEM_ADDR_BITS];
              rd_addr_lo = cm_q_deq_data[59+:`MEM_ADDR_BITS];
              
              if(cm_q_deq_data[61]) begin // odd address (body is even)
                 rd_en_hi = 1;
                 op_nxt = OP_RD_3DW_EVEN;
              end
              else begin // even address (body is odd)
                 rd_en_lo = 1;
                 op_nxt = OP_RD_3DW_ODD;
              end
              
              cm_q_deq_en = 1;
              
              trn_tsof_n = 0;
              trn_teof_n = 1;
              trn_trem_n = 'h00;
              trn_tsrc_rdy_n = 0;
              
              state_nxt = STATE_HEADER2;
              double_last_nxt = 1;

           end
           STATE_HEADER2: begin

              ppl_mux_select = 2;
              
              // write trn_td
              unique case(op)
                OP_RD_3DW_ODD: begin
                   trn_td[63:32] = head2[63:32];
                   ppl_ctrl = 1;
                end
                OP_RD_3DW_EVEN: begin
                   trn_td[63:32] = head2[63:32];
                   ppl_ctrl = 2;
                end
                default: begin
                   trn_td = head2;
                end
              endcase
              
              // update addr and dw_count
              if(op == OP_RD_3DW_ODD) begin
                 if(dw_count == 1) begin
                    rd_addr_hi = addr;
                    rd_en_hi = 1;
                    dw_count_nxt_2 = 0;
                    double_last_nxt = 0;
                 end
                 else begin
                    rd_addr_hi = addr;
                    rd_addr_lo = addr+4;
                    rd_en_hi = 1;
                    rd_en_lo = 1;
                    dw_count_nxt_2 = dw_count - 2;
                    double_last_nxt = 1;
                    addr_nxt_2 = addr + 8;
                 end                
              end
              else if(op == OP_RD_3DW_EVEN) begin
                 if(dw_count == 1) begin
                    rd_addr_lo = addr;
                    rd_en_lo = 1;
                    dw_count_nxt_2 = 0;
                    double_last_nxt = 0;
                 end
                 else begin
                    rd_addr_hi = addr+4;
                    rd_addr_lo = addr;
                    rd_en_hi = 1;
                    rd_en_lo = 1;
                    dw_count_nxt_2 = dw_count - 2;
                    double_last_nxt = 1;
                    addr_nxt_2 = addr + 8;
                 end
              end
              
              trn_tsof_n = 1;
              trn_tsrc_rdy_n = 0;
              
              // is this the last frame?
              if(dw_count == 0) begin
                 
                 if(double_last)
                   trn_trem_n = 'h00;
                 else
                   trn_trem_n = 'h0f;
                 
                 trn_teof_n = 0;
                 state_nxt = STATE_HEADER1;
                 pcie_req_done = 1;
              end
              else begin
                 trn_trem_n = 'h00;
                 trn_teof_n = 1;
                 state_nxt = STATE_BODY;
              end
           end
           STATE_BODY: begin
              
              ppl_mux_select = 3;

              // write trn_td, update addr and dw_count
              if(op == OP_RD_3DW_ODD) begin
                 ppl_ctrl = 3;
                 if(dw_count == 1) begin
                    rd_addr_hi = addr;
                    rd_en_hi = 1;
                    dw_count_nxt_3 = 0;
                    double_last_nxt = 0;
                 end
                 else begin
                    rd_addr_hi = addr;
                    rd_addr_lo = addr+4;
                    rd_en_hi = 1;
                    rd_en_lo = 1;
                    dw_count_nxt_3 = dw_count - 2;
                    double_last_nxt = 1;
                    addr_nxt_3 = addr + 8;
                 end                
              end
              else if(op == OP_RD_3DW_EVEN) begin
                 ppl_ctrl = 4;
                 if(dw_count == 1) begin
                    rd_addr_lo = addr;
                    rd_en_lo = 1;
                    dw_count_nxt_3 = 0;
                    double_last_nxt = 0;
                 end
                 else begin
                    rd_addr_hi = addr+4;
                    rd_addr_lo = addr;
                    rd_en_hi = 1;
                    rd_en_lo = 1;
                    dw_count_nxt_3 = dw_count - 2;
                    double_last_nxt = 1;
                    addr_nxt_3 = addr + 8;
                 end
              end
              
              trn_tsof_n = 1;
              trn_tsrc_rdy_n = 0;

              // is this the last frame?
              if(dw_count == 0) begin
                 
                 if(double_last)
                   trn_trem_n = 'h00;
                 else
                   trn_trem_n = 'h0f;
                 
                 trn_teof_n = 0;
                 state_nxt = STATE_HEADER1;
                 pcie_req_done = 1;
              end
              else begin
                 trn_trem_n = 'h00;
                 trn_teof_n = 1;
                 state_nxt = STATE_BODY;
              end           
           end
         endcase

      end
   end

   always_ff @(posedge pcie_clk) begin
      if(rst) begin
         state <= STATE_HEADER1;
      end
      else begin
         state <= state_nxt;
      end
      
      case(ppl_mux_select)
        2'h1: begin addr <= addr_nxt_1; dw_count <= dw_count_nxt_1; end
        2'h2: begin addr <= addr_nxt_2; dw_count <= dw_count_nxt_2; end
        2'h3: begin addr <= addr_nxt_3; dw_count <= dw_count_nxt_3; end
        default: begin addr <= addr; dw_count <= dw_count; end
      endcase
      
      op                <= op_nxt;
      head2             <= head2_nxt;
      rd_if_select_reg  <= rd_if_select_reg_nxt;
      rd_mem_select_reg <= rd_mem_select_reg_nxt;
      double_last       <= double_last_nxt;
      
      if(pcie_req_grant & ~pcie_req_stall) begin
         rd_addr_lo_reg <= rd_addr_lo;
         rd_addr_hi_reg <= rd_addr_hi;

         trn_td_reg         <= trn_td;
         trn_tsof_n_reg     <= trn_tsof_n;
         trn_teof_n_reg     <= trn_teof_n;
         trn_trem_n_reg     <= trn_trem_n;
         trn_tsrc_rdy_n_reg <= trn_tsrc_rdy_n;
         ppl_ctrl_reg       <= ppl_ctrl;
      end

   end
endmodule

