/*******************************************************************************
 *
 *  NetFPGA-10G http://www.netfpga.org
 *
 *  File:
 *        pcie_cm_q.v
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
 *        PCIe completion queue and packet splitter. PCIe completions (read
 *        replies) are put in the queue in a custom format. They are then
 *        split according to PCIe rules (maximum payload size) and each new
 *        packet is driven to the pcie_tx_cm module.
 * 
 * Top level include file for the DMA engine. It contains constant
 *        definitions that are used throughout the design.
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

module pcie_cm_q
  (
   // completion queue write interface
   input logic                         cm_q_wr_en,
   input logic [`CM_Q_WIDTH-1:0]       cm_q_data,
   output logic                        cm_q_almost_full,

   // pcie_tx queue interface
   output logic                        cm_q_req_v,
   output logic [`PCIE_CM_Q_WIDTH-1:0] cm_q_req_data,
   input logic                         cm_q_req_grant,

   // pcie misc
   input logic [9:0]                   max_payload_decoded,
   input logic                         read_completion_bundary,

   // misc
   input logic                         pcie_clk,
   input logic                         rx_clk,
   input logic                         rst
   );


   // -----------------------------------
   // -- Completion Queue
   // -----------------------------------
   // [75+:`MEM_ADDR_BITS] address
   // [74:59] usec_cnt
   // [58:57] if_select
   // [56:53] mem_select
   // [52:43] dw_len
   // [42:31] byte_len
   // [30:15] requester_id
   // [14:7]  tag
   // [6:0]   low_addr

   logic                               cm_q_deq_en;
   logic [`CM_Q_WIDTH-1:0]             cm_q_deq_data;
   logic                               cm_q_empty;
   logic                               cm_q_full;
  
   fifo #(.WIDTH(`CM_Q_WIDTH), .DEPTH(`CM_Q_DEPTH)) 
   u_cm_q (
           .enq_en(cm_q_wr_en),
           .enq_data(cm_q_data),
           .deq_en(cm_q_deq_en),
           .deq_data(cm_q_deq_data),
           .empty(cm_q_empty),
           .almost_full(cm_q_almost_full),
           .full(cm_q_full),
           .enq_clk(pcie_clk),
           .deq_clk(rx_clk),
           .rst(rst));
 
   localparam CM_STATE_IDLE      = 0;
   localparam CM_STATE_WAIT      = 1;
   localparam CM_STATE_START     = 2;
   localparam CM_STATE_BODY_PREP = 3;
   localparam CM_STATE_BODY      = 4;

   logic [2:0]                  cm_state, cm_state_nxt;
   
   logic [9:0]                  cm_local_dw_size, cm_local_dw_size_nxt;
   
   logic [`MEM_ADDR_BITS-1:0]   cm_q_addr_reg,      cm_q_addr_reg_nxt;
   logic [9:0]                  cm_q_dw_len_reg,    cm_q_dw_len_reg_nxt;
   logic [11:0]                 cm_q_byte_len_reg,  cm_q_byte_len_reg_nxt;
   logic [6:0]                  cm_q_low_addr_reg,  cm_q_low_addr_reg_nxt;

   logic                        cm_q_req_v_nxt;
   logic [`PCIE_CM_Q_WIDTH-1:0] cm_q_req_data_nxt;

   // meet the size and aligment requests for completions
   always_comb begin
      cm_state_nxt = cm_state;
      
      cm_q_req_data_nxt = cm_q_req_data;

      if(cm_q_req_grant)
        cm_q_req_v_nxt = 0;
      else
        cm_q_req_v_nxt = cm_q_req_v;

      cm_q_addr_reg_nxt      = cm_q_addr_reg;
      cm_q_dw_len_reg_nxt    = cm_q_dw_len_reg;
      cm_q_byte_len_reg_nxt  = cm_q_byte_len_reg;
      cm_q_low_addr_reg_nxt  = cm_q_low_addr_reg;
      
      cm_q_deq_en = 0;
      cm_local_dw_size_nxt   = cm_local_dw_size;

      case(cm_state)
        CM_STATE_IDLE: begin
           if(~cm_q_empty & (~cm_q_req_v | cm_q_req_grant)) begin
              
              if(cm_q_deq_data[52:43] > max_payload_decoded) begin // completion bigger than max_payload
                 if(read_completion_bundary)
                   cm_local_dw_size_nxt = max_payload_decoded - {5'b0, cm_q_deq_data[81:77]};
                 else
                   cm_local_dw_size_nxt = max_payload_decoded - {6'b0, cm_q_deq_data[80:77]};
              end
              else begin // smaller (or equal) than max_payload
                 cm_local_dw_size_nxt = cm_q_deq_data[52:43];
              end

              cm_state_nxt = CM_STATE_WAIT;
              
           end
        end
        CM_STATE_WAIT: begin
           cm_state_nxt = CM_STATE_START;                
        end
        CM_STATE_START: begin
           if(cm_q_deq_data[52:43] > max_payload_decoded) begin // completion bigger than max_payload
              cm_state_nxt = CM_STATE_BODY_PREP;
           end
           else begin // smaller than max_payload
              cm_state_nxt = CM_STATE_IDLE;
           end

           cm_q_req_data_nxt[59+:`MEM_ADDR_BITS] = cm_q_deq_data[75+:`MEM_ADDR_BITS];
           cm_q_req_data_nxt[58:57] = cm_q_deq_data[58:57];
           cm_q_req_data_nxt[56:53] = cm_q_deq_data[56:53];
           cm_q_req_data_nxt[42:31] = cm_q_deq_data[42:31];
           cm_q_req_data_nxt[30:15] = cm_q_deq_data[30:15];
           cm_q_req_data_nxt[14:7]  = cm_q_deq_data[14:7];
           cm_q_req_data_nxt[6:0]   = cm_q_deq_data[6:0];
           cm_q_req_data_nxt[52:43] = cm_local_dw_size;
           
           cm_q_addr_reg_nxt        = cm_q_deq_data[75+:`MEM_ADDR_BITS] + {{(`MEM_ADDR_BITS-12){1'b0}}, cm_local_dw_size, 2'b0};
           cm_q_dw_len_reg_nxt      = cm_q_deq_data[52:43] - cm_local_dw_size;
           cm_q_byte_len_reg_nxt    = cm_q_deq_data[42:31] - {cm_local_dw_size, 2'b0} + {10'b0, cm_q_deq_data[1:0]};
           cm_q_low_addr_reg_nxt    = {cm_q_deq_data[6:2], 2'b0} + {cm_local_dw_size[4:0], 2'b0};
           
           cm_q_deq_en = 1;
           cm_q_req_v_nxt = 1;
        end
        CM_STATE_BODY_PREP: begin
           // compute next dw_size              
           if(cm_q_dw_len_reg > max_payload_decoded) begin // next completion bigger than max_payload
              if(read_completion_bundary)
                cm_local_dw_size_nxt = max_payload_decoded - {5'b0, cm_q_addr_reg[6:2]};
              else
                cm_local_dw_size_nxt = max_payload_decoded - {6'b0, cm_q_addr_reg[5:2]};
           end
           else begin // next completion smaller than max_payload
              cm_local_dw_size_nxt = cm_q_dw_len_reg;
           end
           cm_state_nxt = CM_STATE_BODY;
        end
        CM_STATE_BODY: begin
           if(~cm_q_req_v | cm_q_req_grant) begin

              if(cm_q_dw_len_reg > max_payload_decoded) begin // completion bigger than max_payload
                 cm_state_nxt = CM_STATE_BODY_PREP;
              end
              else begin // smaller than max_payload
                 cm_state_nxt = CM_STATE_IDLE;              
              end
              
              cm_q_req_data_nxt[59+:`MEM_ADDR_BITS] = cm_q_addr_reg;
              cm_q_req_data_nxt[42:31] = cm_q_byte_len_reg;
              cm_q_req_data_nxt[6:0]   = cm_q_low_addr_reg;
              cm_q_req_data_nxt[52:43] = cm_local_dw_size;
              
              cm_q_addr_reg_nxt        = cm_q_addr_reg + {{(`MEM_ADDR_BITS-12){1'b0}}, cm_local_dw_size, 2'b0};
              cm_q_dw_len_reg_nxt      = cm_q_dw_len_reg - cm_local_dw_size;
              cm_q_byte_len_reg_nxt    = cm_q_byte_len_reg - {cm_local_dw_size, cm_q_low_addr_reg[1:0]};
              cm_q_low_addr_reg_nxt    = {cm_q_low_addr_reg[6:2], 2'b0} + {cm_local_dw_size[4:0], 2'b0};
              
              cm_q_req_v_nxt = 1;

           end
        end
        default: begin
           cm_state_nxt = CM_STATE_IDLE;
        end          
      endcase
   end
   always_ff @(posedge rx_clk) begin
      if(rst) begin
         cm_state   <= CM_STATE_IDLE;
         cm_q_req_v <= 0;

         cm_local_dw_size <= 0;
      end
      else begin
         cm_state <= cm_state_nxt;
         cm_q_req_v    <= cm_q_req_v_nxt;

         cm_local_dw_size   <= cm_local_dw_size_nxt;
      end

      cm_q_req_data <= cm_q_req_data_nxt;
      cm_q_addr_reg      <= cm_q_addr_reg_nxt;
      cm_q_dw_len_reg    <= cm_q_dw_len_reg_nxt;
      cm_q_byte_len_reg  <= cm_q_byte_len_reg_nxt;
      cm_q_low_addr_reg  <= cm_q_low_addr_reg_nxt;
   end
   
endmodule
