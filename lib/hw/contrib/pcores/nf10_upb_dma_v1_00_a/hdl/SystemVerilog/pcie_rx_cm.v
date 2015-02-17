/*******************************************************************************
 *
 *  NetFPGA-10G http://www.netfpga.org
 *
 *  File:
 *        pcie_rx_cm.v
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
 *        PCIe completion receive module. Processes incoming PCIe packets and
 *        writes the data into correct memory location accorting to the ORT
 *        (Outstanding Request Table) entry. This module also manages the ORT
 *        entry creation requests.
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

module pcie_rx_cm
  (
   // PCIe Transaction RX
   input logic [63:0]                trn_rd,
   input logic [7:0]                 trn_rrem_n,
   input logic                       trn_rsof_n,
   input logic                       trn_reof_n,
   input logic                       trn_rsrc_rdy_n,
   input logic                       trn_rerrfwd_n,
   input logic [6:0]                 trn_rbar_hit_n,
   output logic                      cfg_trn_pending_n,
   
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

   // ORT request interface
   input logic                       ort_req_v,
   input logic [3:0]                 ort_req_tag,
   input logic [1:0]                 ort_req_iface,
   input logic [3:0]                 ort_req_mem,
   input logic [`MEM_ADDR_BITS-1:0]  ort_req_addr,
   output logic                      ort_next_tag_v,
   output logic [3:0]                ort_next_tag,

   // stats
   output logic                      stat_pcie_rx_cm_cnt_inc,
   output logic                      stat_pcie_rx_err_cnt_inc,

   // misc
   input logic                       pcie_clk,
   input logic                       rst
   );

   // -----------------------------------
   // -- flop input PCIe signals
   // -----------------------------------
   logic [63:0]                      trn_rd_reg;
   logic [7:0]                       trn_rrem_n_reg;
   logic                             trn_rsof_n_reg;
   logic                             trn_reof_n_reg;
   logic                             trn_rsrc_rdy_n_reg;
   logic                             trn_rerrfwd_n_reg;
   logic [6:0]                       trn_rbar_hit_n_reg;
   always_ff @(posedge pcie_clk) begin
      trn_rd_reg <= trn_rd;
      trn_rrem_n_reg <= trn_rrem_n;
      trn_rsof_n_reg <= trn_rsof_n;
      trn_reof_n_reg <= trn_reof_n;
      trn_rsrc_rdy_n_reg <= trn_rsrc_rdy_n;
      trn_rerrfwd_n_reg  <= trn_rerrfwd_n;
      trn_rbar_hit_n_reg <= trn_rbar_hit_n;
   end
   
   // -----------------------------------
   // -- Outstanding Request Table entries
   // -----------------------------------
   logic [15:0]               ort_valid,       ort_valid_nxt;
   logic [1:0]                ort_iface[15:0], ort_iface_nxt[15:0];
   logic [3:0]                ort_mem  [15:0], ort_mem_nxt  [15:0];
   logic [`MEM_ADDR_BITS-1:0] ort_addr [15:0], ort_addr_nxt [15:0];

   always_ff @(posedge pcie_clk) cfg_trn_pending_n <= ~|ort_valid;

   // -----------------------------------
   // -- PCIe request processing logic
   // -----------------------------------
   localparam STATE_HEADER1 = 0;
   localparam STATE_HEADER2 = 1;
   localparam STATE_BODY    = 2;

   localparam OP_WR_ODD  = 0;
   localparam OP_WR_EVEN = 1;
   localparam OP_DROP    = 2;
   
   logic [1:0]                state,  state_nxt;
   logic [1:0]                op,     op_nxt;
   logic [63:0]               head1,  head1_nxt;
   logic [`MEM_ADDR_BITS-1:0] addr,   addr_nxt_1,   addr_nxt_2,   addr_nxt_3;
   logic [3:0]                lastBE, lastBE_nxt;

   logic [`MEM_ADDR_BITS-1:0] ort_update_addr,  ort_update_addr_nxt;
   logic                      ort_update_valid, ort_update_valid_nxt;
   logic [3:0]                ort_update_tag,   ort_update_tag_nxt;

   logic [1:0]                wr_if_select_nxt_1,  wr_if_select_nxt_2,  wr_if_select_nxt_3;
   logic [3:0]                wr_mem_select_nxt_1, wr_mem_select_nxt_2, wr_mem_select_nxt_3;
   logic [`MEM_ADDR_BITS-1:0] wr_addr_hi_nxt_1,    wr_addr_hi_nxt_2,    wr_addr_hi_nxt_3;
   logic [31:0]               wr_data_hi_nxt_1,    wr_data_hi_nxt_2,    wr_data_hi_nxt_3;
   logic [3:0]                wr_mask_hi_nxt_1,    wr_mask_hi_nxt_2,    wr_mask_hi_nxt_3;
   logic                      wr_en_hi_nxt_1,      wr_en_hi_nxt_2,      wr_en_hi_nxt_3;
   logic [`MEM_ADDR_BITS-1:0] wr_addr_lo_nxt_1,    wr_addr_lo_nxt_2,    wr_addr_lo_nxt_3;
   logic [31:0]               wr_data_lo_nxt_1,    wr_data_lo_nxt_2,    wr_data_lo_nxt_3;
   logic [3:0]                wr_mask_lo_nxt_1,    wr_mask_lo_nxt_2,    wr_mask_lo_nxt_3;
   logic                      wr_en_lo_nxt_1,      wr_en_lo_nxt_2,      wr_en_lo_nxt_3;
   
   logic [1:0]                wr_mux_select; // 0 - head2-write; 1 - head2-completion; 2-body-odd; 3-body-even    

   always_comb begin
      integer i;
      
      state_nxt   = state;
      op_nxt      = op;
      head1_nxt   = head1;
      addr_nxt_1  = addr;
      addr_nxt_2  = addr;
      addr_nxt_3  = addr;
      lastBE_nxt  = lastBE;

      wr_if_select_nxt_1  = wr_if_select;
      wr_mem_select_nxt_1 = wr_mem_select;      
      wr_addr_hi_nxt_1 = 0;
      wr_data_hi_nxt_1 = 0;
      wr_mask_hi_nxt_1 = 0;
      wr_en_hi_nxt_1   = 0;
      wr_addr_lo_nxt_1 = 0;
      wr_data_lo_nxt_1 = 0;
      wr_mask_lo_nxt_1 = 0;
      wr_en_lo_nxt_1   = 0;

      wr_if_select_nxt_2  = wr_if_select;
      wr_mem_select_nxt_2 = wr_mem_select;      
      wr_addr_hi_nxt_2 = 0;
      wr_data_hi_nxt_2 = 0;
      wr_mask_hi_nxt_2 = 0;
      wr_en_hi_nxt_2   = 0;
      wr_addr_lo_nxt_2 = 0;
      wr_data_lo_nxt_2 = 0;
      wr_mask_lo_nxt_2 = 0;
      wr_en_lo_nxt_2   = 0;

      wr_if_select_nxt_3  = wr_if_select;
      wr_mem_select_nxt_3 = wr_mem_select;      
      wr_addr_hi_nxt_3 = 0;
      wr_data_hi_nxt_3 = 0;
      wr_mask_hi_nxt_3 = 0;
      wr_en_hi_nxt_3   = 0;
      wr_addr_lo_nxt_3 = 0;
      wr_data_lo_nxt_3 = 0;
      wr_mask_lo_nxt_3 = 0;
      wr_en_lo_nxt_3   = 0;

      wr_mux_select = 1;
      
      ort_valid_nxt = ort_valid;
      ort_iface_nxt = ort_iface;
      ort_mem_nxt   = ort_mem;
      ort_addr_nxt  = ort_addr;

      ort_next_tag_v = 0;
      ort_next_tag   = 0;

      ort_update_tag_nxt   = ort_update_tag;
      ort_update_addr_nxt  = ort_update_addr;
      ort_update_valid_nxt = 0;

      // stats
      stat_pcie_rx_cm_cnt_inc  = 0;
      stat_pcie_rx_err_cnt_inc = 0;
      
      // --------------------
      // ORT request handling
      // --------------------
      for(i = 0; i < 16; i++) begin
         if(!ort_valid[i] && !(ort_req_v && ort_req_tag == i[3:0])) begin
            ort_next_tag   = i[3:0];
            ort_next_tag_v = 1;
         end
      end
      if(ort_req_v) begin
         ort_valid_nxt[ort_req_tag] = 1;
         ort_iface_nxt[ort_req_tag] = ort_req_iface;
         ort_mem_nxt[ort_req_tag]   = ort_req_mem;
         ort_addr_nxt[ort_req_tag]  = ort_req_addr;
      end
      if(ort_update_valid) begin
         ort_addr_nxt[ort_update_tag] = ort_update_addr;
      end

      // --------------------
      // Processing TLPs
      // --------------------     
      if(!trn_rsrc_rdy_n_reg && trn_rerrfwd_n_reg) begin
         case(state)

           // wait for start of frame
           STATE_HEADER1: begin
              if(!trn_rsof_n_reg) begin
                 head1_nxt = trn_rd_reg;
                 state_nxt = STATE_HEADER2;
              end
           end

           // process header
           STATE_HEADER2: begin
              
              // -----------------------
              // **** OK COMPLETION ****
              // -----------------------
              if((head1[62:61] == 2'b10) && (head1[60:56] == 5'b01010) && (head1[15:13] == 3'b000)) begin
                 wr_mux_select = 1;
                 // read state from ORT
                 wr_if_select_nxt_1  = ort_iface[trn_rd_reg[43:40]];
                 wr_mem_select_nxt_1 = ort_mem[trn_rd_reg[43:40]];
                 addr_nxt_1          = ort_addr[trn_rd_reg[43:40]] + 4;
                 wr_addr_hi_nxt_1    = ort_addr[trn_rd_reg[43:40]];
                 wr_addr_lo_nxt_1    = ort_addr[trn_rd_reg[43:40]];

                 // setup data and mask
                 wr_data_hi_nxt_1 = {trn_rd_reg[7:0], trn_rd_reg[15:8], trn_rd_reg[23:16], trn_rd_reg[31:24]};
                 wr_data_lo_nxt_1 = {trn_rd_reg[7:0], trn_rd_reg[15:8], trn_rd_reg[23:16], trn_rd_reg[31:24]};
                 case(trn_rd_reg[33:32])
                   2'b00: begin wr_mask_hi_nxt_1 = 4'b1111; wr_mask_lo_nxt_1 = 4'b1111; end
                   2'b01: begin wr_mask_hi_nxt_1 = 4'b1110; wr_mask_lo_nxt_1 = 4'b1110; end
                   2'b10: begin wr_mask_hi_nxt_1 = 4'b1100; wr_mask_lo_nxt_1 = 4'b1100; end
                   2'b11: begin wr_mask_hi_nxt_1 = 4'b1000; wr_mask_lo_nxt_1 = 4'b1000; end
                 endcase
                 
                 // store operation state
                 if(ort_addr[trn_rd_reg[43:40]][2]) begin
                    wr_en_hi_nxt_1 = 1;
                    op_nxt = OP_WR_EVEN;
                 end
                 else begin
                    wr_en_lo_nxt_1 = 1;
                    op_nxt = OP_WR_ODD;
                 end

                 // is this the last reply
                 if( {2'b0, head1[41:32]} == ((head1[11:0] + {10'b0, trn_rd_reg[33:32]} + 12'd3) >> 2) ) begin
                    case(trn_rd_reg[33:32] + head1[1:0])
                      2'b00: lastBE_nxt = 4'b1111;
                      2'b01: lastBE_nxt = 4'b0001;
                      2'b10: lastBE_nxt = 4'b0011;
                      2'b11: lastBE_nxt = 4'b0111;
                    endcase
                    ort_valid_nxt[trn_rd_reg[43:40]] = 0;
                 end
                 else begin
                    lastBE_nxt = 4'b1111;
                 end                 

                 if(head1[41:32] == 10'b0) begin
                    ort_update_addr_nxt = ort_addr[trn_rd_reg[43:40]] + 'h1000;
                    ort_update_tag_nxt = trn_rd_reg[43:40];
                    ort_update_valid_nxt = 1;
                 end
                 else begin
                    ort_update_addr_nxt = ort_addr[trn_rd_reg[43:40]] + {{(`MEM_ADDR_BITS-12){1'b0}}, head1[41:32], 2'b0};
                    ort_update_tag_nxt = trn_rd_reg[43:40];
                    ort_update_valid_nxt = 1;
                 end

                 // stats
                 stat_pcie_rx_cm_cnt_inc = 1;
              end

              // --------------------------
              // **** ERROR COMPLETION ****
              // --------------------------
              else if((head1[62:61] == 2'b10) && (head1[60:56] == 5'b01010) && (head1[15:13] != 3'b000)) begin
                 stat_pcie_rx_err_cnt_inc = 1;
                 ort_valid_nxt[trn_rd_reg[43:40]] = 0;
                 op_nxt = OP_DROP;
              end

              // --------------------------
              // **** UNKNOWN ****
              // --------------------------
              else begin
                 op_nxt = OP_DROP;
              end

              // advance state machine
              if(!trn_reof_n_reg) begin
                 state_nxt = STATE_HEADER1;
              end
              else begin
                 state_nxt = STATE_BODY;
              end

           end
           
           // receiving body of the packet
           STATE_BODY: begin
              case(op)
                OP_WR_ODD: begin
                   wr_mux_select = 2;
                   
                   addr_nxt_2 = addr + 8;

                   wr_addr_lo_nxt_2 = addr + 4;
                   wr_data_lo_nxt_2 = {trn_rd_reg[7:0],   trn_rd_reg[15:8],  trn_rd_reg[23:16], trn_rd_reg[31:24]};
                   
                   wr_addr_hi_nxt_2 = addr;
                   wr_data_hi_nxt_2 = {trn_rd_reg[39:32], trn_rd_reg[47:40], trn_rd_reg[55:48], trn_rd_reg[63:56]};

                   if(!trn_reof_n_reg) begin // writing last word
                      if(trn_rrem_n_reg[3:0] == 4'b0000) begin
                         wr_mask_lo_nxt_2 = lastBE;
                         wr_en_lo_nxt_2   = 1;
                         
                         wr_mask_hi_nxt_2 = 4'b1111;
                         wr_en_hi_nxt_2   = 1;
                      end
                      else begin
                         wr_mask_hi_nxt_2 = lastBE;
                         wr_en_hi_nxt_2   = 1;
                      end
                   end
                   else begin // writing non-last word
                      wr_mask_lo_nxt_2 = 4'b1111;
                      wr_en_lo_nxt_2   = 1;
                      
                      wr_mask_hi_nxt_2 = 4'b1111;
                      wr_en_hi_nxt_2   = 1;
                   end                                         
                end
                
                OP_WR_EVEN: begin
                   wr_mux_select = 3;
                   
                   addr_nxt_3 = addr + 8;

                   wr_addr_lo_nxt_3 = addr;
                   wr_data_lo_nxt_3 = {trn_rd_reg[39:32], trn_rd_reg[47:40], trn_rd_reg[55:48], trn_rd_reg[63:56]};

                   wr_addr_hi_nxt_3 = addr + 4;
                   wr_data_hi_nxt_3 = {trn_rd_reg[7:0],   trn_rd_reg[15:8],  trn_rd_reg[23:16], trn_rd_reg[31:24]};                   

                   if(!trn_reof_n_reg) begin // writing last word
                      if(trn_rrem_n_reg[3:0] == 4'b0000) begin
                         wr_mask_lo_nxt_3 = 4'b1111;
                         wr_en_lo_nxt_3   = 1;

                         wr_mask_hi_nxt_3 = lastBE;
                         wr_en_hi_nxt_3   = 1;                         
                      end
                      else begin
                         wr_mask_lo_nxt_3 = lastBE;
                         wr_en_lo_nxt_3   = 1;
                      end
                   end
                   else begin // writing non-last word
                      wr_mask_lo_nxt_3 = 4'b1111;
                      wr_en_lo_nxt_3   = 1;
                      
                      wr_mask_hi_nxt_3 = 4'b1111;
                      wr_en_hi_nxt_3   = 1;
                   end                                         
                end
                OP_DROP: begin
                end
                default: begin
                end
              endcase

              // advance state machine
              if(!trn_reof_n_reg) begin
                 state_nxt = STATE_HEADER1;
              end                   
              
           end
         endcase
      end
   end

   always_ff @(posedge pcie_clk) begin
      if(rst) begin
         state      <= STATE_HEADER1;
         ort_valid  <= 16'h0;
         ort_update_valid <= 0;

         wr_en_lo   <= 0;
         wr_en_hi   <= 0;
      end
      else begin
         state  <= state_nxt;

         case(wr_mux_select)
           2'h1: begin
              wr_en_lo <= wr_en_lo_nxt_1;
              wr_en_hi <= wr_en_hi_nxt_1;
           end
           2'h2: begin
              wr_en_lo <= wr_en_lo_nxt_2;
              wr_en_hi <= wr_en_hi_nxt_2;
           end
           2'h3: begin
              wr_en_lo <= wr_en_lo_nxt_3;
              wr_en_hi <= wr_en_hi_nxt_3;
           end
           default: begin
              wr_en_lo <= 0;
              wr_en_hi <= 0;
           end
         endcase

         ort_valid  <= ort_valid_nxt;
         ort_update_valid <= ort_update_valid_nxt;
      end

      op     <= op_nxt;
      head1  <= head1_nxt;
      lastBE <= lastBE_nxt;

      case(wr_mux_select)
        2'h1: begin
           addr <= addr_nxt_1;
           wr_if_select  <= wr_if_select_nxt_1;
           wr_mem_select <= wr_mem_select_nxt_1;      
           wr_addr_lo <= wr_addr_lo_nxt_1;
           wr_data_lo <= wr_data_lo_nxt_1;
           wr_mask_lo <= wr_mask_lo_nxt_1;
           wr_addr_hi <= wr_addr_hi_nxt_1;
           wr_data_hi <= wr_data_hi_nxt_1;
           wr_mask_hi <= wr_mask_hi_nxt_1;
        end
        2'h2: begin
           addr <= addr_nxt_2;
           wr_if_select  <= wr_if_select_nxt_2;
           wr_mem_select <= wr_mem_select_nxt_2;      
           wr_addr_lo <= wr_addr_lo_nxt_2;
           wr_data_lo <= wr_data_lo_nxt_2;
           wr_mask_lo <= wr_mask_lo_nxt_2;
           wr_addr_hi <= wr_addr_hi_nxt_2;
           wr_data_hi <= wr_data_hi_nxt_2;
           wr_mask_hi <= wr_mask_hi_nxt_2;
        end
        2'h3: begin
           addr <= addr_nxt_3;
           wr_if_select  <= wr_if_select_nxt_3;
           wr_mem_select <= wr_mem_select_nxt_3;      
           wr_addr_lo <= wr_addr_lo_nxt_3;
           wr_data_lo <= wr_data_lo_nxt_3;
           wr_mask_lo <= wr_mask_lo_nxt_3;
           wr_addr_hi <= wr_addr_hi_nxt_3;
           wr_data_hi <= wr_data_hi_nxt_3;
           wr_mask_hi <= wr_mask_hi_nxt_3;
        end
        default: begin
        end
      endcase

      ort_update_tag   <= ort_update_tag_nxt;
      ort_update_addr  <= ort_update_addr_nxt;
      
      ort_iface  <= ort_iface_nxt;
      ort_mem    <= ort_mem_nxt;
      ort_addr   <= ort_addr_nxt;

   end
endmodule
