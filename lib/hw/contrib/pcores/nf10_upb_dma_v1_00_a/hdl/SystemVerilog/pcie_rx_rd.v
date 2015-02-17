/*******************************************************************************
 *
 *  NetFPGA-10G http://www.netfpga.org
 *
 *  File:
 *        pcie_rx_rd.v
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
 *        PCIe read receive module. Incoming read requests are processed by
 *        creating a completion (reply) request and putting it into the pcie_cm_q.
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

`include "dma_defs.vh"

module pcie_rx_rd
  (
   // PCIe Transaction RX
   input logic [63:0]             trn_rd,
   input logic [7:0]              trn_rrem_n,
   input logic                    trn_rsof_n,
   input logic                    trn_reof_n,
   input logic                    trn_rsrc_rdy_n,
   input logic                    trn_rerrfwd_n,
   input logic [6:0]              trn_rbar_hit_n,

   // completion fifo interface
   output logic                   cm_q_wr_en,
   output logic [`CM_Q_WIDTH-1:0] cm_q_data,
   input logic [`NUM_PORTS-1:0]   cm_q_almost_full,

   // stats
   output logic                   stat_pcie_rx_rd_cnt_inc,

   // misc
   input logic                    pcie_clk,
   input logic                    rst
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
   // -- PCIe request processing logic
   // -----------------------------------
   localparam STATE_HEADER1 = 0;
   localparam STATE_HEADER2 = 1;

   logic                      state,  state_nxt;
   logic [63:0]               head1,  head1_nxt;

   logic                      cm_q_wr_en_nxt;
   logic [`CM_Q_WIDTH-1:0]    cm_q_data_nxt;

   always_comb begin

      state_nxt   = state;
      head1_nxt   = head1;
      
      cm_q_wr_en_nxt = 0;
      cm_q_data_nxt  = 0;

      // stats
      stat_pcie_rx_rd_cnt_inc = 0;

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
              
              // --------------------
              // **** 3/4DW READ ****
              // --------------------
              if((head1[62] == 1'b0) && (head1[60:56] == 5'b00000)) begin
                 // process address
                 if(head1[61] == 1'b0) begin // 3dw read
                    if(!trn_rbar_hit_n_reg[0]) begin
                       cm_q_data_nxt[58:57] = 2'h0;
                       cm_q_data_nxt[56:53] = {3'b0,  trn_rd_reg[32+`BAR0_MEM_SELECT_HI    : 32+`BAR0_MEM_SELECT_LO]};
                       cm_q_data_nxt[6:2]   =         trn_rd_reg[32+`BAR0_ADDR_SELECT_LO+6 : 32+`BAR0_ADDR_SELECT_LO+2];
                       cm_q_data_nxt[75+:`MEM_ADDR_BITS] = {6'b0, trn_rd_reg[32+`BAR0_ADDR_SELECT_HI : 32+`BAR0_ADDR_SELECT_LO]};
                    end
                    else if(!trn_rbar_hit_n_reg[2]) begin
                       cm_q_data_nxt[58:57] = 2'h0;
                       cm_q_data_nxt[56:53] = {3'b0, trn_rd_reg[32+`BAR2_MEM_SELECT_HI    : 32+`BAR2_MEM_SELECT_LO]} + 2;
                       cm_q_data_nxt[6:2]   = trn_rd_reg[32+`BAR2_ADDR_SELECT_LO+6 : 32+`BAR2_ADDR_SELECT_LO+2];
                       cm_q_data_nxt[75+:`MEM_ADDR_BITS] = trn_rd_reg[32+`BAR2_ADDR_SELECT_HI : 32+`BAR2_ADDR_SELECT_LO];
                    end
                 end
                 else if(head1[61] == 1'b1) begin // 4dw read
                    if(!trn_rbar_hit_n_reg[0]) begin
                       cm_q_data_nxt[58:57] = 2'h0;
                       cm_q_data_nxt[56:53] = {3'b0, trn_rd_reg[`BAR0_MEM_SELECT_HI    : `BAR0_MEM_SELECT_LO]};
                       cm_q_data_nxt[6:2]   =        trn_rd_reg[`BAR0_ADDR_SELECT_LO+6 : `BAR0_ADDR_SELECT_LO+2];
                       cm_q_data_nxt[75+:`MEM_ADDR_BITS] = {6'b0, trn_rd_reg[`BAR0_ADDR_SELECT_HI : `BAR0_ADDR_SELECT_LO]};
                    end
                    else if(!trn_rbar_hit_n_reg[2]) begin
                       cm_q_data_nxt[58:57] = 2'h0;
                       cm_q_data_nxt[56:53] = {3'b0, trn_rd_reg[`BAR2_MEM_SELECT_HI    : `BAR2_MEM_SELECT_LO]} + 2;
                       cm_q_data_nxt[6:2]   = trn_rd_reg[`BAR2_ADDR_SELECT_LO+6 : `BAR2_ADDR_SELECT_LO+2];
                       cm_q_data_nxt[75+:`MEM_ADDR_BITS] = trn_rd_reg[`BAR2_ADDR_SELECT_HI : `BAR2_ADDR_SELECT_LO];
                    end
                 end

                 // prepare data for completion queue:
                 //   59+:`MEM_ADDR_BITS -- address, 
                 //   58:57 -- if_select, 
                 //   56:53 -- mem_select, 
                 //   52:43 -- dw_len, 
                 //   42:31 -- byte_len, 
                 //   30:15 -- requester_id, 
                 //   14:7  -- tag, 
                 //   6:0   -- low_addr
                 cm_q_data_nxt[74:59] = 0;
                 cm_q_data_nxt[52:43] = head1[41:32];
                 cm_q_data_nxt[30:15] = head1[31:16];
                 cm_q_data_nxt[14:7]  = head1[15:8];
                 casez({head1[3:0], head1[7:4]})
                   8'b1??1_0000: cm_q_data_nxt[42:31] = 4;
                   8'b01?1_0000: cm_q_data_nxt[42:31] = 3;
                   8'b1?10_0000: cm_q_data_nxt[42:31] = 3;
                   8'b0011_0000: cm_q_data_nxt[42:31] = 2;
                   8'b0110_0000: cm_q_data_nxt[42:31] = 2;
                   8'b1100_0000: cm_q_data_nxt[42:31] = 2;
                   8'b0001_0000: cm_q_data_nxt[42:31] = 1;
                   8'b0010_0000: cm_q_data_nxt[42:31] = 1;
                   8'b0100_0000: cm_q_data_nxt[42:31] = 1;
                   8'b1000_0000: cm_q_data_nxt[42:31] = 1;
                   8'b0000_0000: cm_q_data_nxt[42:31] = 1;
                   8'b???1_1???: cm_q_data_nxt[42:31] = {head1[41:32], 2'b0};
                   8'b???1_01??: cm_q_data_nxt[42:31] = {head1[41:32], 2'b0} - 1;
                   8'b???1_001?: cm_q_data_nxt[42:31] = {head1[41:32], 2'b0} - 2;
                   8'b???1_0001: cm_q_data_nxt[42:31] = {head1[41:32], 2'b0} - 3;
                   8'b??10_1???: cm_q_data_nxt[42:31] = {head1[41:32], 2'b0} - 1;
                   8'b??10_01??: cm_q_data_nxt[42:31] = {head1[41:32], 2'b0} - 2;
                   8'b??10_001?: cm_q_data_nxt[42:31] = {head1[41:32], 2'b0} - 3;
                   8'b??10_0001: cm_q_data_nxt[42:31] = {head1[41:32], 2'b0} - 4;
                   8'b?100_1???: cm_q_data_nxt[42:31] = {head1[41:32], 2'b0} - 2;
                   8'b?100_01??: cm_q_data_nxt[42:31] = {head1[41:32], 2'b0} - 3;
                   8'b?100_001?: cm_q_data_nxt[42:31] = {head1[41:32], 2'b0} - 4;
                   8'b?100_0001: cm_q_data_nxt[42:31] = {head1[41:32], 2'b0} - 5;
                   8'b1000_1???: cm_q_data_nxt[42:31] = {head1[41:32], 2'b0} - 3;
                   8'b1000_01??: cm_q_data_nxt[42:31] = {head1[41:32], 2'b0} - 4;
                   8'b1000_001?: cm_q_data_nxt[42:31] = {head1[41:32], 2'b0} - 5;
                   8'b1000_0001: cm_q_data_nxt[42:31] = {head1[41:32], 2'b0} - 6;
                   default: cm_q_data_nxt[42:31] = 0;
                 endcase
                 casez(head1[3:0])
                   4'b0000: cm_q_data_nxt[1:0] = 2'b00; 
                   4'b???1: cm_q_data_nxt[1:0] = 2'b00; 
                   4'b??10: cm_q_data_nxt[1:0] = 2'b01; 
                   4'b?100: cm_q_data_nxt[1:0] = 2'b10; 
                   4'b1000: cm_q_data_nxt[1:0] = 2'b11; 
                 endcase

                 // write to completion queue
                 cm_q_wr_en_nxt = 1;

                 // stats
                 stat_pcie_rx_rd_cnt_inc = 1;
              end
              
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
         state <= STATE_HEADER1;         
      end
      else begin
         state <= state_nxt;
      end

      head1 <= head1_nxt;
    
      cm_q_wr_en <= cm_q_wr_en_nxt;
      cm_q_data  <= cm_q_data_nxt;
   end
endmodule
