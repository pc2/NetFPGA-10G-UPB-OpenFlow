/*******************************************************************************
 *
 *  NetFPGA-10G http://www.netfpga.org
 *
 *  File:
 *        tx_ctrl.v
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
 *        This module controls the transmision of packets on the AXIS interface.
 *        It also manages TX descriptors on the card, sending DMA reads for
 *        fetching new packets, as well as realignment of data on the
 *        AXIS interface, if neccessary.
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

module tx_ctrl
  (
   // memory read interfaces
   output logic [`MEM_ADDR_BITS-1:0]  mem_tx_dsc_rd_addr,
   input logic [63:0]                 mem_tx_dsc_rd_data,
   output logic                       mem_tx_dsc_rd_en,

   output logic [`MEM_ADDR_BITS-1:0]  mem_tx_pkt_rd_addr,
   input logic [63:0]                 mem_tx_pkt_rd_data,
   output logic                       mem_tx_pkt_rd_en,

   // memory write interfaces
   output logic [`MEM_ADDR_BITS-1:0]  mem_tx_dne_wr_addr,
   output logic [63:0]                mem_tx_dne_wr_data,
   output logic [7:0]                 mem_tx_dne_wr_mask,
   output logic                       mem_tx_dne_wr_en,

   // memory valid interfaces
   output logic [`MEM_ADDR_BITS-12:0] mem_vld_tx_dsc_wr_addr,
   output logic [31:0]                mem_vld_tx_dsc_wr_mask,
   output logic                       mem_vld_tx_dsc_wr_clear,
   input logic                        mem_vld_tx_dsc_wr_stall,
   input logic                        mem_vld_tx_dsc_rd_bit,

   output logic [`MEM_ADDR_BITS-12:0] mem_vld_tx_pkt_wr_addr,
   output logic [31:0]                mem_vld_tx_pkt_wr_mask,
   output logic                       mem_vld_tx_pkt_wr_clear,
   input logic                        mem_vld_tx_pkt_wr_stall,
   input logic                        mem_vld_tx_pkt_rd_bit,
   
   output logic [`MEM_ADDR_BITS-12:0] mem_vld_tx_dne_wr_addr,
   output logic [31:0]                mem_vld_tx_dne_wr_mask,
   output logic                       mem_vld_tx_dne_wr_clear,
   input logic                        mem_vld_tx_dne_wr_stall,

   // config registers
   input logic [63:0]                 tx_dsc_mask,
   input logic [63:0]                 tx_pkt_mask,
   input logic [63:0]                 tx_dne_mask,
   input logic [63:0]                 rx_dsc_mask,

   // pcie read queue interface
   output logic                       rd_q_enq_en,
   output logic [`RD_Q_WIDTH-1:0]     rd_q_enq_data,
   input logic                        rd_q_full,
   
   // MAC interface
   output logic [63:0]                M_AXIS_TDATA,
   output logic [7:0]                 M_AXIS_TSTRB,
   output logic                       M_AXIS_TVALID,
   input logic                        M_AXIS_TREADY,
   output logic                       M_AXIS_TLAST,
   output logic [127:0]               M_AXIS_TUSER,

   // stats
   output logic [63:0]                stat_mac_tx_ts,
   output logic [31:0]                stat_mac_tx_word_cnt,
   output logic [31:0]                stat_mac_tx_pkt_cnt,

   // misc
   input logic                        clk,
   input logic                        rst
   );   
   
   // ----------------------------------
   // -- mem pointers
   // ----------------------------------
   logic [`MEM_ADDR_BITS-1:0]        mem_tx_dsc_head, mem_tx_dsc_head_nxt;

   logic [`MEM_ADDR_BITS-1:0]        mem_tx_pkt_head, mem_tx_pkt_head_nxt;
   logic [`MEM_ADDR_BITS-1:0]        mem_tx_pkt_end,  mem_tx_pkt_end_nxt;

   logic [`MEM_ADDR_BITS-1:0]        mem_tx_dne_tail, mem_tx_dne_tail_nxt;
   logic [`MEM_ADDR_BITS-1:0]        mem_tx_dne_clear;

   // ----------------------------------
   // -- stats
   // ----------------------------------
   logic [63:0]                      time_stamp;
   logic [63:0]                      stat_mac_tx_ts_nxt;
   logic [31:0]                      stat_mac_tx_word_cnt_nxt;
   logic [31:0]                      stat_mac_tx_pkt_cnt_nxt;

   always_comb begin
      stat_mac_tx_ts_nxt       = stat_mac_tx_ts;
      stat_mac_tx_word_cnt_nxt = stat_mac_tx_word_cnt;
      stat_mac_tx_pkt_cnt_nxt  = stat_mac_tx_pkt_cnt;

      if(M_AXIS_TVALID & M_AXIS_TREADY) begin
         stat_mac_tx_ts_nxt        = time_stamp;
         stat_mac_tx_word_cnt_nxt  = stat_mac_tx_word_cnt + 1;
         if(M_AXIS_TLAST) 
           stat_mac_tx_pkt_cnt_nxt = stat_mac_tx_pkt_cnt_nxt + 1;
      end
   end
   
   always_ff @(posedge clk) begin
      if(rst) begin
         time_stamp           <= 0;
         stat_mac_tx_ts       <= 0;
         stat_mac_tx_word_cnt <= 0;
         stat_mac_tx_pkt_cnt  <= 0;
      end
      else begin
         time_stamp           <= time_stamp + 1;
         stat_mac_tx_ts       <= stat_mac_tx_ts_nxt;
         stat_mac_tx_word_cnt <= stat_mac_tx_word_cnt_nxt;
         stat_mac_tx_pkt_cnt  <= stat_mac_tx_pkt_cnt_nxt;
      end
   end
   
   // ----------------------------------
   // -- local signals
   // ----------------------------------
   /* verilator lint_off UNOPTFLAT */
   logic                             dma_rd_go;
   logic                             dma_rd_done;
   logic [63:0]                      dma_rd_host_addr;
   /* verilator lint_on UNOPTFLAT */
   logic [15:0]                      dma_rd_len;
   logic [`MEM_ADDR_BITS-1:0]        dma_rd_local_addr;
   
   logic                             rd_q_enq_en_nxt;
   logic [`RD_Q_WIDTH-1:0]           rd_q_enq_data_nxt;

   // ----------------------------------
   // -- Send a DMA read
   // ----------------------------------
   always_comb begin

      if(~rd_q_full) begin
         rd_q_enq_en_nxt = 0;
         rd_q_enq_data_nxt = 0; 
      end
      else begin
         rd_q_enq_en_nxt = rd_q_enq_en;
         rd_q_enq_data_nxt = rd_q_enq_data;
      end
      
      dma_rd_done        = 0;
      
      if(~rd_q_full) begin
         if(dma_rd_go) begin
            rd_q_enq_en_nxt = 1;
            // read the last cache line (because that will set off the valid bit)
            if(dma_rd_local_addr[5:0] + dma_rd_len[5:0] == 6'b0)
              rd_q_enq_data_nxt[15:0] = dma_rd_len[15:0];
            else
              rd_q_enq_data_nxt[15:0] = dma_rd_len[15:0] + {10'b0, 6'd0 - (dma_rd_local_addr[5:0] + dma_rd_len[5:0])};
            rd_q_enq_data_nxt[19:16] = `ID_MEM_TX_PKT; // mem_select
            rd_q_enq_data_nxt[83:20] = dma_rd_host_addr; // host addr
            rd_q_enq_data_nxt[84+:`MEM_ADDR_BITS] = dma_rd_local_addr; // addr    
            dma_rd_done = 1;
         end
      end
   end
   always_ff @(posedge clk) begin
      if(rst) begin
         rd_q_enq_en <= 0;
      end
      else begin
         rd_q_enq_en <= rd_q_enq_en_nxt;
      end
      rd_q_enq_data <= rd_q_enq_data_nxt;
   end

   // ----------------------------------
   // -- tx pending queue
   // ----------------------------------
   logic                           tx_pend_q_enq_en;
   logic [2*`MEM_ADDR_BITS+16-1:0] tx_pend_q_enq_data;
   logic                           tx_pend_q_deq_en;
   logic [2*`MEM_ADDR_BITS+16-1:0] tx_pend_q_deq_data;
   logic                           tx_pend_q_empty;                        
   logic                           tx_pend_q_full;
   
   fifo #(.WIDTH(`MEM_ADDR_BITS*2+16), .DEPTH(`TX_PENDING_DEPTH))
   u_tx_pending_q(.enq_en(tx_pend_q_enq_en),
                  .enq_data(tx_pend_q_enq_data),
                  .deq_en(tx_pend_q_deq_en),
                  .deq_data(tx_pend_q_deq_data),
                  .empty(tx_pend_q_empty),
                  .almost_full(),
                  .full(tx_pend_q_full),
                  .enq_clk(clk),
                  .deq_clk(clk),
                  .*);
   
   // -------------------------------------------
   // -- read new TX descriptor and process it
   // -------------------------------------------
   logic [`MEM_ADDR_BITS-1:0] dma_rd_local_addr_nxt;
   logic [15:0]               dma_rd_len_nxt;
   logic [15:0]               pkt_port, pkt_port_nxt;
   
   localparam READ_TX_DSC_STATE_IDLE      = 0;
   localparam READ_TX_DSC_STATE_L1        = 1;
   localparam READ_TX_DSC_STATE_L2        = 2;
   localparam READ_TX_DSC_STATE_WAIT      = 3;   

   logic [1:0]                 read_tx_dsc_state, read_tx_dsc_state_nxt;

   logic [`MEM_ADDR_BITS-1:0]  mem_tx_dsc_head_reg, mem_tx_dsc_head_reg_nxt;
   
   always_comb begin
      read_tx_dsc_state_nxt = read_tx_dsc_state;      
      mem_tx_dsc_head_nxt = mem_tx_dsc_head;

      mem_tx_dsc_rd_en = 1;
      mem_tx_dsc_rd_addr = mem_tx_dsc_head;

      mem_vld_tx_dsc_wr_addr  = mem_tx_dsc_head_reg[`MEM_ADDR_BITS-1:11];
      mem_vld_tx_dsc_wr_mask  = 1 << mem_tx_dsc_head_reg[10:6];
      mem_vld_tx_dsc_wr_clear = 0;
      
      dma_rd_len_nxt        = dma_rd_len;
      dma_rd_local_addr_nxt = dma_rd_local_addr;

      dma_rd_host_addr = 0;
      dma_rd_go        = 0;
      tx_pend_q_enq_data = 0;
      tx_pend_q_enq_en   = 0;

      pkt_port_nxt = pkt_port;

      mem_tx_dsc_head_reg_nxt = mem_tx_dsc_head_reg;

      case(read_tx_dsc_state)
        READ_TX_DSC_STATE_IDLE: begin
           if(mem_vld_tx_dsc_rd_bit) begin
              // move head pointer
              mem_tx_dsc_head_nxt = (mem_tx_dsc_head + 8) & tx_dsc_mask[`MEM_ADDR_BITS-1:0];
              mem_tx_dsc_head_reg_nxt = mem_tx_dsc_head;
              // advance state
              read_tx_dsc_state_nxt = READ_TX_DSC_STATE_L1;
           end
        end
        READ_TX_DSC_STATE_L1: begin
           // store read line
           dma_rd_len_nxt = mem_tx_dsc_rd_data[63:48];
           pkt_port_nxt   = mem_tx_dsc_rd_data[47:32];
           dma_rd_local_addr_nxt = mem_tx_dsc_rd_data[`MEM_ADDR_BITS-1:0];
           // advance state
           read_tx_dsc_state_nxt = READ_TX_DSC_STATE_L2;
        end
        READ_TX_DSC_STATE_L2: begin
           
           if(!tx_pend_q_full && (mem_tx_dsc_rd_data != 0)) begin // data not already in mem_tx_pkt, dma_read
              dma_rd_host_addr = mem_tx_dsc_rd_data;
              dma_rd_go = 1;
           end

           if(!tx_pend_q_full && ((mem_tx_dsc_rd_data == 0) || (dma_rd_done == 1)) ) begin // got DMA grant
              tx_pend_q_enq_data[2*`MEM_ADDR_BITS+:16] = pkt_port;
              tx_pend_q_enq_data[`MEM_ADDR_BITS+:`MEM_ADDR_BITS] = dma_rd_local_addr;
              tx_pend_q_enq_data[0+:`MEM_ADDR_BITS] = dma_rd_local_addr + 
                                                      {{(`MEM_ADDR_BITS-$bits(dma_rd_len)){1'b0}}, dma_rd_len};
              tx_pend_q_enq_en = 1;

              // clear valid bit              
              read_tx_dsc_state_nxt = READ_TX_DSC_STATE_WAIT;
              
              // move head pointer
              mem_tx_dsc_head_nxt = (mem_tx_dsc_head + 56) & tx_dsc_mask[`MEM_ADDR_BITS-1:0];
           end
        end

        READ_TX_DSC_STATE_WAIT: begin
           if(~mem_vld_tx_dsc_wr_stall) begin
              mem_vld_tx_dsc_wr_clear = 1;
              read_tx_dsc_state_nxt = READ_TX_DSC_STATE_IDLE;
           end
        end
      endcase
      
   end
   always_ff @(posedge clk) begin
      if(rst) begin
         read_tx_dsc_state <= READ_TX_DSC_STATE_IDLE;
         mem_tx_dsc_head  <= 0; 
      end
      else begin
         read_tx_dsc_state <= read_tx_dsc_state_nxt;
         mem_tx_dsc_head   <= mem_tx_dsc_head_nxt;
      end

      dma_rd_len        <= dma_rd_len_nxt;
      dma_rd_local_addr <= dma_rd_local_addr_nxt;

      mem_tx_dsc_head_reg <= mem_tx_dsc_head_reg_nxt;
      pkt_port <= pkt_port_nxt;
   end

   // ----------------------------------
   // -- Shift ouput S_AXIS data
   // ----------------------------------
   logic [63:0]                M_AXIS_TDATA_L;
   logic                       M_AXIS_TREADY_L;
   logic [7:0]                 M_AXIS_TSTRB_L;
   logic                       M_AXIS_TVALID_L;
   logic                       M_AXIS_TLAST_L;
   logic [127:0]               M_AXIS_TUSER_L;

   tx_pkt_shift u_shift(.in_tdata(M_AXIS_TDATA_L),
                        .in_tstrb(M_AXIS_TSTRB_L),
                        .in_tvalid(M_AXIS_TVALID_L),
                        .in_tready(M_AXIS_TREADY_L),
                        .in_tlast(M_AXIS_TLAST_L),
                        .in_tuser(M_AXIS_TUSER_L),
                        .out_tdata(M_AXIS_TDATA),
                        .out_tstrb(M_AXIS_TSTRB),
                        .out_tvalid(M_AXIS_TVALID),
                        .out_tready(M_AXIS_TREADY),
                        .out_tlast(M_AXIS_TLAST),
                        .out_tuser(M_AXIS_TUSER),
                        .clk(clk),
                        .rst(rst)
                        );
   
   // ----------------------------------
   // -- send the packet out
   // ----------------------------------
   localparam PKT_SEND_STATE_IDLE       = 0;
   localparam PKT_SEND_STATE_PKT_START  = 1;
   localparam PKT_SEND_STATE_LINE_DATA  = 2;

   logic [1:0]                 pkt_send_state, pkt_send_state_nxt;
   
   logic                       mem_tx_pkt_mark_end, mem_tx_pkt_mark_end_nxt;

   logic [`MEM_ADDR_BITS-12:0] mem_vld_tx_pkt_wr_addr_nxt;
   logic [31:0]                mem_vld_tx_pkt_wr_mask_nxt;
   logic                       mem_vld_tx_pkt_wr_clear_nxt;
                                         
   logic [`MEM_ADDR_BITS-1:0]  mem_tx_pkt_head_l, mem_tx_pkt_head_reg;
   logic                       mem_vld_tx_pkt_wr_clear_l;

   logic [63:0]                M_AXIS_TDATA_L_nxt;
   logic [7:0]                 M_AXIS_TSTRB_L_nxt;
   logic                       M_AXIS_TVALID_L_nxt;
   logic                       M_AXIS_TLAST_L_nxt;
   logic [127:0]               M_AXIS_TUSER_L_nxt;

   logic [15:0]                mem_tx_pkt_port, mem_tx_pkt_port_nxt;

   assign mem_tx_pkt_rd_addr = mem_tx_pkt_head;
   
   always_comb begin
      pkt_send_state_nxt = pkt_send_state;
      
      mem_tx_pkt_head     = mem_tx_pkt_head_l;
      mem_tx_pkt_head_nxt = mem_tx_pkt_head_l;
      mem_tx_pkt_end_nxt  = mem_tx_pkt_end;
      mem_tx_pkt_rd_en    = 1;
      mem_tx_pkt_mark_end_nxt = mem_tx_pkt_mark_end;
      
      mem_vld_tx_pkt_wr_addr_nxt  = mem_vld_tx_pkt_wr_addr;
      mem_vld_tx_pkt_wr_mask_nxt  = mem_vld_tx_pkt_wr_mask;
      mem_vld_tx_pkt_wr_clear     = mem_vld_tx_pkt_wr_clear_l;
      mem_vld_tx_pkt_wr_clear_nxt = 0;

      // clear 7 entries ahead, make sure buffer is at least 16 lines deep
      mem_tx_dne_clear        = (mem_tx_dne_tail + 64*7) & tx_dne_mask[`MEM_ADDR_BITS-1:0];
      mem_vld_tx_dne_wr_addr  = mem_tx_dne_clear[`MEM_ADDR_BITS-1:11];
      mem_vld_tx_dne_wr_mask  = 1 << mem_tx_dne_clear[10:6];
      mem_vld_tx_dne_wr_clear = 0;
      
      M_AXIS_TDATA_L_nxt  = M_AXIS_TDATA_L;
      M_AXIS_TSTRB_L_nxt  = M_AXIS_TSTRB_L;
      M_AXIS_TLAST_L_nxt  = M_AXIS_TLAST_L;
      M_AXIS_TUSER_L_nxt  = M_AXIS_TUSER_L;
      
      if(M_AXIS_TREADY_L)
        M_AXIS_TVALID_L_nxt = 0;
      else
        M_AXIS_TVALID_L_nxt = M_AXIS_TVALID_L;
      
      tx_pend_q_deq_en = 0;

      mem_tx_pkt_port_nxt = mem_tx_pkt_port;
      
      mem_tx_dne_wr_en     = 0;
      mem_tx_dne_wr_mask   = 8'hff;
      mem_tx_dne_wr_data   = 0;
      mem_tx_dne_wr_addr   = 0;
      mem_tx_dne_tail_nxt  = mem_tx_dne_tail;
      
      case(pkt_send_state)
        PKT_SEND_STATE_IDLE: begin
           if(~tx_pend_q_empty) begin
              mem_tx_pkt_port_nxt = tx_pend_q_deq_data[2*`MEM_ADDR_BITS+:16];
              mem_tx_pkt_head_nxt = tx_pend_q_deq_data[`MEM_ADDR_BITS+:`MEM_ADDR_BITS] & tx_pkt_mask[`MEM_ADDR_BITS-1:0];
              mem_tx_pkt_end_nxt  = tx_pend_q_deq_data[0+:`MEM_ADDR_BITS] & tx_pkt_mask[`MEM_ADDR_BITS-1:0];;
              mem_tx_pkt_head     = mem_tx_pkt_head_nxt; // save a cycle by doing this
              pkt_send_state_nxt  = PKT_SEND_STATE_PKT_START;
           end
        end

        PKT_SEND_STATE_PKT_START: begin
           if(mem_vld_tx_pkt_rd_bit) begin
              // dequeue pending transmit
              tx_pend_q_deq_en = 1;
              // move head pointer
              mem_tx_pkt_head_nxt = (mem_tx_pkt_head + 8) & tx_pkt_mask[`MEM_ADDR_BITS-1:0];
              // advance state
              pkt_send_state_nxt = PKT_SEND_STATE_LINE_DATA;
              // prepare vld_tx_pkt clear mask and address
              mem_vld_tx_pkt_wr_addr_nxt = mem_tx_pkt_head[`MEM_ADDR_BITS-1:11];
              mem_vld_tx_pkt_wr_mask_nxt = 32'hffffffff << mem_tx_pkt_head[10:6];
              // prepare M_AXIS_TUSER
              M_AXIS_TUSER_L_nxt[31:16] = mem_tx_pkt_port;
              M_AXIS_TUSER_L_nxt[15:0]  = (mem_tx_pkt_end[15:0] - mem_tx_pkt_head[15:0]) & tx_pkt_mask[15:0];
           end
           else begin
              pkt_send_state_nxt = PKT_SEND_STATE_IDLE;
           end
        end

        PKT_SEND_STATE_LINE_DATA: begin
           if(!mem_vld_tx_pkt_rd_bit || !M_AXIS_TREADY_L || mem_vld_tx_pkt_wr_stall || mem_vld_tx_dne_wr_stall) begin
              mem_tx_pkt_head = mem_tx_pkt_head_reg;
              mem_vld_tx_pkt_wr_clear = 0;
              mem_vld_tx_pkt_wr_clear_nxt = mem_vld_tx_pkt_wr_clear_l;
           end
           else begin
              M_AXIS_TVALID_L_nxt = 1;
              if(~mem_tx_pkt_mark_end) begin
                 case(mem_tx_pkt_head[2:0])
                   3'h0: begin
                      M_AXIS_TSTRB_L_nxt = 8'hff;
                      M_AXIS_TDATA_L_nxt = mem_tx_pkt_rd_data;
                   end
                   3'h1: begin
                      M_AXIS_TSTRB_L_nxt = 8'h7f;
                      M_AXIS_TDATA_L_nxt = {8'b0, mem_tx_pkt_rd_data[63:8]};
                   end
                   3'h2: begin
                      M_AXIS_TSTRB_L_nxt = 8'h3f;
                      M_AXIS_TDATA_L_nxt = {16'b0, mem_tx_pkt_rd_data[63:16]};
                   end
                   3'h3: begin 
                      M_AXIS_TSTRB_L_nxt = 8'h1f;
                      M_AXIS_TDATA_L_nxt = {24'b0, mem_tx_pkt_rd_data[63:24]};
                   end
                   3'h4: begin 
                      M_AXIS_TSTRB_L_nxt = 8'h0f;
                      M_AXIS_TDATA_L_nxt = {32'b0, mem_tx_pkt_rd_data[63:32]};
                   end
                   3'h5: begin 
                      M_AXIS_TSTRB_L_nxt = 8'h07;
                      M_AXIS_TDATA_L_nxt = {40'b0, mem_tx_pkt_rd_data[63:40]};
                   end
                   3'h6: begin 
                      M_AXIS_TSTRB_L_nxt = 8'h03;
                      M_AXIS_TDATA_L_nxt = {48'b0, mem_tx_pkt_rd_data[63:48]};
                   end
                   3'h7: begin 
                      M_AXIS_TSTRB_L_nxt = 8'h01;
                      M_AXIS_TDATA_L_nxt = {56'b0, mem_tx_pkt_rd_data[63:56]};
                   end
                 endcase
                 M_AXIS_TLAST_L_nxt = 0;

                 // read next line
                 if(((mem_tx_pkt_end - mem_tx_pkt_head) & tx_pkt_mask[`MEM_ADDR_BITS-1:0]) > 8) begin
                    mem_tx_pkt_head_nxt = ({mem_tx_pkt_head[`MEM_ADDR_BITS-1:3],3'b0} + 8) & tx_pkt_mask[`MEM_ADDR_BITS-1:0];
                    // done with this batch of lines, clear valid bits
                    if((mem_tx_pkt_head[10:6] == 5'b11111) && (mem_tx_pkt_head_nxt[10:6] == 5'b00000)) begin
                       mem_vld_tx_pkt_wr_clear_nxt = 1;
                    end
                 end
                 else begin
                    mem_tx_pkt_mark_end_nxt = 1;   
                 end                  

                 // reinitialize vld_tx_pkt write
                 if(mem_tx_pkt_head[10:6] == 5'b00000) begin
                    mem_vld_tx_pkt_wr_addr_nxt = mem_tx_pkt_head[`MEM_ADDR_BITS-1:11];
                    mem_vld_tx_pkt_wr_mask_nxt = 32'hffffffff;
                 end  

              end
              else begin
                 case(mem_tx_pkt_end[2:0])
                   3'd0: begin
                      M_AXIS_TSTRB_L_nxt = 8'hff;
                      M_AXIS_TDATA_L_nxt = mem_tx_pkt_rd_data;
                   end
                   3'd1: begin
                      M_AXIS_TSTRB_L_nxt = 8'h01;
                      M_AXIS_TDATA_L_nxt = {56'b0, mem_tx_pkt_rd_data[7:0]};
                   end
                   3'd2: begin 
                      M_AXIS_TSTRB_L_nxt = 8'h03;
                      M_AXIS_TDATA_L_nxt = {48'b0, mem_tx_pkt_rd_data[15:0]};
                   end
                   3'd3: begin
                      M_AXIS_TSTRB_L_nxt = 8'h07;
                      M_AXIS_TDATA_L_nxt = {40'b0, mem_tx_pkt_rd_data[23:0]};
                   end
                   3'd4: begin
                      M_AXIS_TSTRB_L_nxt = 8'h0f;
                      M_AXIS_TDATA_L_nxt = {32'b0, mem_tx_pkt_rd_data[31:0]};
                   end
                   3'd5: begin
                      M_AXIS_TSTRB_L_nxt = 8'h1f;
                      M_AXIS_TDATA_L_nxt = {24'b0, mem_tx_pkt_rd_data[39:0]};
                   end
                   3'd6: begin
                      M_AXIS_TSTRB_L_nxt = 8'h3f;
                      M_AXIS_TDATA_L_nxt = {16'b0, mem_tx_pkt_rd_data[47:0]};
                   end
                   3'd7: begin
                      M_AXIS_TSTRB_L_nxt = 8'h7f;
                      M_AXIS_TDATA_L_nxt = {8'b0, mem_tx_pkt_rd_data[55:0]};
                   end
                   default: begin
                      M_AXIS_TSTRB_L_nxt = 8'hff;
                      M_AXIS_TDATA_L_nxt = mem_tx_pkt_rd_data;
                   end
                 endcase
                 M_AXIS_TLAST_L_nxt = 1;
                 
                 // clear the tx_pkt buffer
                 if(mem_tx_pkt_end[5:0] == 6'b0) begin
                    mem_vld_tx_pkt_wr_mask_nxt = mem_vld_tx_pkt_wr_mask & (32'hffffffff >> (5'd32 - mem_tx_pkt_end[10:6]));
                 end
                 else begin 
                    mem_vld_tx_pkt_wr_mask_nxt = mem_vld_tx_pkt_wr_mask & (32'hffffffff >> (5'd31 - mem_tx_pkt_end[10:6]));
                 end
                 mem_vld_tx_pkt_wr_clear_nxt = 1;

                 // clear the end mark
                 mem_tx_pkt_mark_end_nxt = 0;
                 
                 // advance state
                 pkt_send_state_nxt = PKT_SEND_STATE_IDLE;
                 // generate interrupt
                 mem_tx_dne_wr_en = 1;
                 mem_tx_dne_wr_mask = 8'hff;
                 mem_tx_dne_wr_data[15:0] = 'd1;
                 mem_tx_dne_wr_addr = mem_tx_dne_tail;
                 mem_tx_dne_tail_nxt = (mem_tx_dne_tail + 64) & tx_dne_mask[`MEM_ADDR_BITS-1:0];
                 mem_vld_tx_dne_wr_clear = 1;

              end             
           end
        end
                
      endcase
      
   end
   always_ff @(posedge clk) begin
      if(rst) begin
         pkt_send_state <= PKT_SEND_STATE_IDLE;

         mem_tx_pkt_head_l <= 0;         
         mem_tx_pkt_end  <= 0;
         mem_tx_pkt_mark_end <= 0;
         
         mem_vld_tx_pkt_wr_clear_l <= 0;

         mem_tx_dne_tail <= 0;

         M_AXIS_TVALID_L <= 0;
      end
      else begin
         pkt_send_state <= pkt_send_state_nxt;       

         mem_tx_pkt_head_l <= mem_tx_pkt_head_nxt;         

         mem_tx_pkt_end  <= mem_tx_pkt_end_nxt;
         mem_tx_pkt_mark_end <= mem_tx_pkt_mark_end_nxt;
         
         mem_vld_tx_pkt_wr_clear_l <= mem_vld_tx_pkt_wr_clear_nxt;

         mem_tx_dne_tail <= mem_tx_dne_tail_nxt;

         M_AXIS_TVALID_L <= M_AXIS_TVALID_L_nxt;
      end

      mem_vld_tx_pkt_wr_addr <= mem_vld_tx_pkt_wr_addr_nxt;
      mem_vld_tx_pkt_wr_mask <= mem_vld_tx_pkt_wr_mask_nxt;

      mem_tx_pkt_head_reg <= mem_tx_pkt_head;
      mem_tx_pkt_port     <= mem_tx_pkt_port_nxt;

      M_AXIS_TDATA_L <= M_AXIS_TDATA_L_nxt;
      M_AXIS_TSTRB_L <= M_AXIS_TSTRB_L_nxt;
      M_AXIS_TLAST_L <= M_AXIS_TLAST_L_nxt;
      M_AXIS_TUSER_L <= M_AXIS_TUSER_L_nxt;
      
   end
   
endmodule

module tx_pkt_shift
  (
   input logic [63:0]   in_tdata,
   input logic [7:0]    in_tstrb,
   input logic          in_tvalid,
   output logic         in_tready,
   input logic          in_tlast,
   input logic [127:0]  in_tuser,
   
   output logic [63:0]  out_tdata,
   output logic [7:0]   out_tstrb,
   output logic         out_tvalid,
   input logic          out_tready,
   output logic         out_tlast, 
   output logic [127:0] out_tuser, 
   
   input logic          clk,
   input logic          rst
   );

   localparam STATE_IDLE = 0;
   localparam STATE_TX   = 1;
   localparam STATE_LAST = 2;

   logic [1:0]          state, state_nxt;
   
   logic [63:0]         out_tdata_nxt,  out_tdata_reg,  out_tdata_reg_d1;
   logic [7:0]          out_tstrb_nxt,  out_tstrb_reg,  out_tstrb_reg_d1;
   logic                out_tvalid_nxt, out_tvalid_reg, out_tvalid_reg_d1;
   logic                out_tlast_nxt,  out_tlast_reg,  out_tlast_reg_d1;
   logic [127:0]        out_tuser_nxt,  out_tuser_reg,  out_tuser_reg_d1;

   logic [63:0]         tdata, tdata_nxt;
   logic [7:0]          tstrb, tstrb_nxt;

   logic [2:0]          offset, offset_nxt;

   logic                in_tready_nxt;
   logic                out_tready_d1;
   
   always_comb begin
      state_nxt = state;

      offset_nxt = offset;

      tdata_nxt = tdata;
      tstrb_nxt = tstrb;

      out_tvalid_nxt = 0;
      out_tlast_nxt  = 0;
      out_tuser_nxt  = 0;
      out_tdata_nxt  = out_tdata_reg;
      out_tstrb_nxt  = out_tstrb_reg;

      out_tvalid = out_tvalid_reg;
      out_tlast  = out_tlast_reg;
      out_tuser  = out_tuser_reg;
      out_tdata  = out_tdata_reg;
      out_tstrb  = out_tstrb_reg;

      in_tready_nxt = 1;
      
      if(~out_tready) begin
         in_tready_nxt = 0;
      end
      else if(out_tready & ~out_tready_d1) begin
         out_tvalid = out_tvalid_reg_d1;
         out_tlast  = out_tlast_reg_d1;
         out_tuser  = out_tuser_reg_d1;
         out_tdata  = out_tdata_reg_d1;
         out_tstrb  = out_tstrb_reg_d1; 
         if(state == STATE_LAST) in_tready_nxt = 0;
      end
            
      if(out_tready_d1) begin
         case(state)
           STATE_IDLE: begin
              if(in_tvalid) begin
                 case(in_tstrb)
                   8'hff: begin 
                      offset_nxt = 0;
                      out_tdata_nxt = in_tdata;
                      out_tstrb_nxt = in_tstrb;
                      out_tvalid_nxt = 1;
                      out_tuser_nxt = in_tuser;
                   end
                   8'h7f: begin
                      tdata_nxt = {in_tdata[55:0], 8'b0};
                      tstrb_nxt = {in_tstrb[6:0], 1'b0};
                      offset_nxt = 1;
                   end
                   8'h3f: begin
                      tdata_nxt = {in_tdata[47:0], 16'b0};
                      tstrb_nxt = {in_tstrb[5:0], 2'b0};
                      offset_nxt = 2;
                   end
                   8'h1f: begin
                      tdata_nxt = {in_tdata[39:0], 24'b0};
                      tstrb_nxt = {in_tstrb[4:0], 3'b0};
                      offset_nxt = 3;
                   end
                   8'h0f: begin
                      tdata_nxt = {in_tdata[31:0], 32'b0};
                      tstrb_nxt = {in_tstrb[3:0], 4'b0};
                      offset_nxt = 4;
                   end
                   8'h07: begin
                      tdata_nxt = {in_tdata[23:0], 40'b0};
                      tstrb_nxt = {in_tstrb[2:0], 5'b0};
                      offset_nxt = 5;
                   end
                   8'h03: begin
                      tdata_nxt = {in_tdata[15:0], 48'b0};
                      tstrb_nxt = {in_tstrb[1:0], 6'b0};
                      offset_nxt = 6;
                   end
                   8'h01: begin
                      tdata_nxt = {in_tdata[7:0], 56'b0};
                      tstrb_nxt = {in_tstrb[0], 7'b0};
                      offset_nxt = 7;
                   end
                   default: begin
                   end
                 endcase
                 state_nxt = STATE_TX;
              end
           end
           STATE_TX: begin
              if(in_tvalid) begin
                 tdata_nxt = in_tdata;
                 tstrb_nxt = in_tstrb;
                 case(offset)
                   3'h0: begin
                      out_tdata_nxt = in_tdata;
                      out_tstrb_nxt = in_tstrb;
                      out_tvalid_nxt = 1;
                      out_tuser_nxt = in_tuser;
                      if(in_tlast) begin
                         out_tlast_nxt = 1;
                         state_nxt = STATE_IDLE;
                      end
                   end
                   3'h1: begin
                      out_tdata_nxt = {in_tdata[7:0], tdata[63:8]};
                      out_tstrb_nxt = {in_tstrb[0], tstrb[7:1]};
                      out_tvalid_nxt = 1;
                      out_tuser_nxt = in_tuser;
                      if(in_tlast & ~in_tstrb[1]) begin
                         out_tlast_nxt = 1;
                         state_nxt = STATE_IDLE;
                      end
                      else if(in_tlast & in_tstrb[1]) begin
                         state_nxt = STATE_LAST;
                         in_tready_nxt = 0;
                      end
                   end
                   3'h2: begin
                      out_tdata_nxt = {in_tdata[15:0], tdata[63:16]};
                      out_tstrb_nxt = {in_tstrb[1:0], tstrb[7:2]};
                      out_tvalid_nxt = 1;
                      out_tuser_nxt = in_tuser;
                      if(in_tlast & ~in_tstrb[2]) begin
                         out_tlast_nxt = 1;
                         state_nxt = STATE_IDLE;
                      end
                      else if(in_tlast & in_tstrb[2]) begin
                         state_nxt = STATE_LAST;
                         in_tready_nxt = 0;
                      end
                   end
                   3'h3: begin
                      out_tdata_nxt = {in_tdata[23:0], tdata[63:24]};
                      out_tstrb_nxt = {in_tstrb[2:0], tstrb[7:3]};
                      out_tvalid_nxt = 1;
                      out_tuser_nxt = in_tuser;
                      if(in_tlast & ~in_tstrb[3]) begin
                         out_tlast_nxt = 1;
                         state_nxt = STATE_IDLE;
                      end
                      else if(in_tlast & in_tstrb[3]) begin
                         state_nxt = STATE_LAST;
                         in_tready_nxt = 0;
                      end
                   end
                   3'h4: begin
                      out_tdata_nxt = {in_tdata[31:0], tdata[63:32]};
                      out_tstrb_nxt = {in_tstrb[3:0], tstrb[7:4]};
                      out_tvalid_nxt = 1;
                      out_tuser_nxt = in_tuser;
                      if(in_tlast & ~in_tstrb[4]) begin
                         out_tlast_nxt = 1;
                         state_nxt = STATE_IDLE;
                      end
                      else if(in_tlast & in_tstrb[4]) begin
                         state_nxt = STATE_LAST;
                         in_tready_nxt = 0;
                      end
                   end
                   3'h5: begin
                      out_tdata_nxt = {in_tdata[39:0], tdata[63:40]};
                      out_tstrb_nxt = {in_tstrb[4:0], tstrb[7:5]};
                      out_tvalid_nxt = 1;
                      out_tuser_nxt = in_tuser;
                      if(in_tlast & ~in_tstrb[5]) begin
                         out_tlast_nxt = 1;
                         state_nxt = STATE_IDLE;
                      end
                      else if(in_tlast & in_tstrb[5]) begin
                         state_nxt = STATE_LAST;
                         in_tready_nxt = 0;
                      end
                   end
                   3'h6: begin
                      out_tdata_nxt = {in_tdata[47:0], tdata[63:48]};
                      out_tstrb_nxt = {in_tstrb[5:0], tstrb[7:6]};
                      out_tvalid_nxt = 1;
                      out_tuser_nxt = in_tuser;
                      if(in_tlast & ~in_tstrb[6]) begin
                         out_tlast_nxt = 1;
                         state_nxt = STATE_IDLE;
                      end
                      else if(in_tlast & in_tstrb[6]) begin
                         state_nxt = STATE_LAST;
                         in_tready_nxt = 0;
                      end
                   end
                   3'h7: begin
                      out_tdata_nxt = {in_tdata[55:0], tdata[63:56]};
                      out_tstrb_nxt = {in_tstrb[6:0], tstrb[7]};
                      out_tvalid_nxt = 1;
                      out_tuser_nxt = in_tuser;
                      if(in_tlast & ~in_tstrb[7]) begin
                         out_tlast_nxt = 1;
                         state_nxt = STATE_IDLE;
                      end
                      else if(in_tlast & in_tstrb[7]) begin
                         state_nxt = STATE_LAST;
                         in_tready_nxt = 0;
                      end
                   end
                 endcase
              end
           end
           STATE_LAST: begin
              out_tlast_nxt = 1;
              out_tvalid_nxt = 1;
              out_tuser_nxt = in_tuser;
              case(offset)
                3'h0: begin
                end
                3'h1: begin
                   out_tdata_nxt = {8'b0, tdata[63:8]};
                   out_tstrb_nxt = {1'b0, tstrb[7:1]};
                end
                3'h2: begin
                   out_tdata_nxt = {16'b0, tdata[63:16]};
                   out_tstrb_nxt = {2'b0, tstrb[7:2]};
                end
                3'h3: begin
                   out_tdata_nxt = {24'b0, tdata[63:24]};
                   out_tstrb_nxt = {3'b0, tstrb[7:3]};
                end
                3'h4: begin
                   out_tdata_nxt = {32'b0, tdata[63:32]};
                   out_tstrb_nxt = {4'b0, tstrb[7:4]};
                end
                3'h5: begin
                   out_tdata_nxt = {40'b0, tdata[63:40]};
                   out_tstrb_nxt = {5'b0, tstrb[7:5]};
                end
                3'h6: begin
                   out_tdata_nxt = {48'b0, tdata[63:48]};
                   out_tstrb_nxt = {6'b0, tstrb[7:6]};
                end
                3'h7: begin
                   out_tdata_nxt = {56'b0, tdata[63:56]};
                   out_tstrb_nxt = {7'b0, tstrb[7]};
                end
              endcase
              state_nxt = STATE_IDLE;
           end
         endcase
      end
   end
   always_ff @(posedge clk) begin
      if(rst) begin
         state      <= STATE_IDLE;
         in_tready  <= 0;
         out_tvalid_reg    <= 0;
         out_tvalid_reg_d1 <= 0;
      end
      else begin
         state      <= state_nxt;
         in_tready  <= in_tready_nxt;
         
         if(out_tready_d1) begin
            out_tvalid_reg    <= out_tvalid_nxt;
            out_tvalid_reg_d1 <= out_tvalid_reg;
         end
         
      end
      offset <= offset_nxt;

      out_tready_d1 <= out_tready;
      tdata <= tdata_nxt;
      tstrb <= tstrb_nxt;

      if(out_tready_d1) begin
         out_tdata_reg    <= out_tdata_nxt;
         out_tstrb_reg    <= out_tstrb_nxt;
         out_tlast_reg    <= out_tlast_nxt;
         out_tuser_reg    <= out_tuser_nxt;
         out_tdata_reg_d1 <= out_tdata_reg;
         out_tstrb_reg_d1 <= out_tstrb_reg;
         out_tlast_reg_d1 <= out_tlast_reg;
         out_tuser_reg_d1 <= out_tuser_reg;
      end
         
   end
   
endmodule
