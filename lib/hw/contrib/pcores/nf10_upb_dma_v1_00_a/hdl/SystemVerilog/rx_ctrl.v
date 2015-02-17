/*******************************************************************************
 *
 *  NetFPGA-10G http://www.netfpga.org
 *
 *  File:
 *        rx_ctrl.v
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
 *        This module controls the reception of packets from the AXIS interface.
 *        It also manages RX descriptors on the card, sending completion
 *        notifications and interrupts, as well as realignment of data on the
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

//`define DEBUG_CHIPSCOPE

`include "dma_defs.vh"

module rx_ctrl
  (
   // memory read interfaces
   output logic [`MEM_ADDR_BITS-1:0]  mem_rx_dsc_rd_addr,
   input logic [63:0]                 mem_rx_dsc_rd_data,
   output logic                       mem_rx_dsc_rd_en,

   // memory write interfaces
   output logic [`MEM_ADDR_BITS-1:0]  mem_rx_pkt_wr_addr,
   output logic [63:0]                mem_rx_pkt_wr_data,
   output logic [7:0]                 mem_rx_pkt_wr_mask,
   output logic                       mem_rx_pkt_wr_en,

   output logic [`MEM_ADDR_BITS-1:0]  mem_rx_dne_wr_addr,
   output logic [63:0]                mem_rx_dne_wr_data,
   output logic [7:0]                 mem_rx_dne_wr_mask,
   output logic                       mem_rx_dne_wr_en,

   // memory valid interfaces
   output logic [`MEM_ADDR_BITS-12:0] mem_vld_rx_dsc_wr_addr,
   output logic [31:0]                mem_vld_rx_dsc_wr_mask,
   output logic                       mem_vld_rx_dsc_wr_clear,
   input logic                        mem_vld_rx_dsc_wr_stall,
   input logic                        mem_vld_rx_dsc_rd_bit,

   output logic [`MEM_ADDR_BITS-12:0] mem_vld_rx_dne_wr_addr,
   output logic [31:0]                mem_vld_rx_dne_wr_mask,
   output logic                       mem_vld_rx_dne_wr_clear,
   input logic                        mem_vld_rx_dne_wr_stall,
   output logic [`MEM_ADDR_BITS-12:0] mem_vld_rx_dne_rd_addr,
   input logic [31:0]                 mem_vld_rx_dne_rd_bits,

   output logic [`MEM_ADDR_BITS-12:0] mem_vld_tx_dne_rd_addr,
   input logic [31:0]                 mem_vld_tx_dne_rd_bits,

   // Config registers
   input logic [63:0]                 rx_dsc_mask,
   input logic [63:0]                 rx_pkt_mask,
   input logic [63:0]                 tx_dne_mask,
   input logic [63:0]                 rx_dne_mask,
   input logic                        tx_int_enable,
   input logic                        rx_int_enable,
   input logic [15:0]                 rx_byte_wait,
   input logic [63:0]                 host_tx_dne_offset,
   input logic [63:0]                 host_tx_dne_mask,
   input logic [63:0]                 host_rx_dne_offset,
   input logic [63:0]                 host_rx_dne_mask,

   // pcie write queue interface
   output logic                       wr_q_enq_en,
   output logic [`WR_Q_WIDTH-1:0]     wr_q_enq_data,
   input logic                        wr_q_almost_full,
   input logic                        wr_q_full,
   
   // MAC interface
   input logic [63:0]                 S_AXIS_TDATA,
   input logic [7:0]                  S_AXIS_TSTRB,
   input logic                        S_AXIS_TVALID,
   output logic                       S_AXIS_TREADY,
   input logic                        S_AXIS_TLAST,
   input logic [127:0]                S_AXIS_TUSER,

   // stats
   output logic [63:0]                stat_mac_rx_ts,
   output logic [31:0]                stat_mac_rx_word_cnt,
   output logic [31:0]                stat_mac_rx_pkt_cnt,
   output logic [31:0]                stat_mac_rx_err_cnt,

   // misc
   input logic                        clk,
   input logic                        rst,

   inout wire [35:0]                  chipscope_control_0
   );
   
   // ----------------------------------
   // -- mem pointers
   // ----------------------------------
   logic [`MEM_ADDR_BITS-1:0]        mem_rx_dsc_head, mem_rx_dsc_head_nxt;

   logic [`MEM_ADDR_BITS-1:0]        mem_rx_pkt_tail, mem_rx_pkt_tail_nxt;

   logic [`MEM_ADDR_BITS-1:0]        mem_rx_dne_head, mem_rx_dne_head_nxt;
   logic [`MEM_ADDR_BITS-1:0]        mem_rx_dne_tail, mem_rx_dne_tail_nxt;
   logic [`MEM_ADDR_BITS-1:0]        mem_rx_dne_clear;

   logic [`MEM_ADDR_BITS-1:0]        mem_tx_dne_head, mem_tx_dne_head_nxt;

   // ----------------------------------
   // -- stats
   // ----------------------------------
   logic [63:0]                      time_stamp;
   logic [63:0]                      stat_mac_rx_ts_nxt;
   logic [31:0]                      stat_mac_rx_word_cnt_nxt;
   logic [31:0]                      stat_mac_rx_pkt_cnt_nxt;
   logic [31:0]                      stat_mac_rx_err_cnt_nxt;

   always_comb begin
      stat_mac_rx_ts_nxt       = stat_mac_rx_ts;
      stat_mac_rx_word_cnt_nxt = stat_mac_rx_word_cnt;
      stat_mac_rx_pkt_cnt_nxt  = stat_mac_rx_pkt_cnt;

      if(S_AXIS_TVALID & S_AXIS_TREADY) begin
         stat_mac_rx_ts_nxt        = time_stamp;
         stat_mac_rx_word_cnt_nxt  = stat_mac_rx_word_cnt + 1;
         if(S_AXIS_TLAST) 
           stat_mac_rx_pkt_cnt_nxt = stat_mac_rx_pkt_cnt_nxt + 1;
      end
   end
   
   always_ff @(posedge clk) begin
      if(rst) begin
         time_stamp           <= 0;
         stat_mac_rx_ts       <= 0;
         stat_mac_rx_word_cnt <= 0;
         stat_mac_rx_pkt_cnt  <= 0;
      end
      else begin
         time_stamp           <= time_stamp + 1;
         stat_mac_rx_ts       <= stat_mac_rx_ts_nxt;
         stat_mac_rx_word_cnt <= stat_mac_rx_word_cnt_nxt;
         stat_mac_rx_pkt_cnt  <= stat_mac_rx_pkt_cnt_nxt;
      end
   end
   
   // ----------------------------------
   // -- Send a DMA write/interrupt
   // ----------------------------------
   localparam DMA_WR_STATE_IDLE = 0;
   localparam DMA_WR_STATE_INTR = 1;

   logic                             dma_wr_state, dma_wr_state_nxt;
   logic                             dma_wr_intr_data, dma_wr_intr_data_nxt;        

   logic                             dma_wr_go, dma_wr_go_nxt;
   logic                             dma_wr_rdy;
   logic [63:0]                      dma_wr_host_addr, dma_wr_host_addr_nxt;
   logic [`MEM_ADDR_BITS-1:0]        dma_wr_local_addr, dma_wr_local_addr_nxt;            
   logic [15:0]                      dma_wr_len, dma_wr_len_nxt;
   logic [15:0]                      dma_rx_byte, dma_rx_byte_nxt;

   logic                             wr_q_enq_en_nxt;
   logic [`WR_Q_WIDTH-1:0]           wr_q_enq_data_nxt;
   
   always_comb begin
      dma_wr_state_nxt = dma_wr_state;
      dma_wr_intr_data_nxt = dma_wr_intr_data;
      
      mem_rx_dne_head_nxt = mem_rx_dne_head;
      mem_tx_dne_head_nxt = mem_tx_dne_head;
      
      if(~wr_q_full) begin
         wr_q_enq_en_nxt = 0;
         wr_q_enq_data_nxt = 0;
      end
      else begin
         wr_q_enq_en_nxt = wr_q_enq_en;
         wr_q_enq_data_nxt = wr_q_enq_data;
      end
      
      dma_wr_rdy = ~wr_q_almost_full;

      mem_vld_rx_dne_rd_addr = mem_rx_dne_head[`MEM_ADDR_BITS-1:11];
      mem_vld_tx_dne_rd_addr = mem_tx_dne_head[`MEM_ADDR_BITS-1:11];
      
      case(dma_wr_state)
        DMA_WR_STATE_IDLE: begin      
           if(~wr_q_full) begin
              if(dma_wr_go) begin
                 wr_q_enq_data_nxt[21:6] = dma_wr_len; // byte len
                 wr_q_enq_data_nxt[25:22] = `ID_MEM_RX_PKT; // mem select
                 wr_q_enq_data_nxt[89:26] = dma_wr_host_addr; // host address
                 wr_q_enq_data_nxt[90+:`MEM_ADDR_BITS] = dma_wr_local_addr; // address                                    
                 wr_q_enq_en_nxt = 1;
              end
              else if(mem_vld_rx_dne_rd_bits[mem_rx_dne_head[10:6]]) begin
                 mem_rx_dne_head_nxt    = (mem_rx_dne_head + 64) & rx_dne_mask[`MEM_ADDR_BITS-1:0];
                 mem_vld_rx_dne_rd_addr = mem_rx_dne_head_nxt[`MEM_ADDR_BITS-1:11];
                 
                 wr_q_enq_data_nxt[25:22] = `ID_MEM_RX_DNE; // mem select
                 wr_q_enq_data_nxt[21:6] = 8; // byte len
                 wr_q_enq_data_nxt[90+:`MEM_ADDR_BITS] = mem_rx_dne_head + 56;
                 wr_q_enq_data_nxt[89:26] = (host_rx_dne_offset & ~host_rx_dne_mask) |
                                            ({{($bits(host_rx_dne_mask)-`MEM_ADDR_BITS){1'b0}}, mem_rx_dne_head} & host_rx_dne_mask) + 56; // host address
                 wr_q_enq_en_nxt = 1;
                 if(rx_int_enable) begin
                    dma_wr_intr_data_nxt = 0;
                    dma_wr_state_nxt = DMA_WR_STATE_INTR;                    
                 end
              end
              else if(mem_vld_tx_dne_rd_bits[mem_tx_dne_head[10:6]]) begin
                 mem_tx_dne_head_nxt    = (mem_tx_dne_head + 64) & tx_dne_mask[`MEM_ADDR_BITS-1:0];
                 mem_vld_tx_dne_rd_addr = mem_tx_dne_head_nxt[`MEM_ADDR_BITS-1:11];
                 
                 wr_q_enq_data_nxt[21:6] = 4; // byte len
                 wr_q_enq_data_nxt[25:22] = `ID_MEM_TX_DNE; // mem select
                 wr_q_enq_data_nxt[89:26] = (host_tx_dne_offset & ~host_tx_dne_mask) | 
                                            ({{($bits(host_tx_dne_mask)-`MEM_ADDR_BITS){1'b0}}, mem_tx_dne_head} & host_tx_dne_mask); // host address
                 wr_q_enq_data_nxt[90+:`MEM_ADDR_BITS] = mem_tx_dne_head; // address                   
                 wr_q_enq_en_nxt = 1;
                 if(tx_int_enable) begin
                    dma_wr_intr_data_nxt = 1;
                    dma_wr_state_nxt = DMA_WR_STATE_INTR;
                 end
              end
           end
        end
        DMA_WR_STATE_INTR: begin
           if(~wr_q_full) begin
              if(dma_wr_go) begin
                 wr_q_enq_data_nxt[21:6] = dma_wr_len; // byte len
                 wr_q_enq_data_nxt[25:22] = `ID_MEM_RX_PKT; // mem select
                 wr_q_enq_data_nxt[89:26] = dma_wr_host_addr; // host address
                 wr_q_enq_data_nxt[90+:`MEM_ADDR_BITS] = dma_wr_local_addr; // address                                    
                 wr_q_enq_en_nxt = 1;
              end
              else begin
                 wr_q_enq_data_nxt[0] = 1; // interrupt
                 wr_q_enq_data_nxt[1] = dma_wr_intr_data;
                 wr_q_enq_en_nxt = 1;              
                 dma_wr_state_nxt = DMA_WR_STATE_IDLE;
              end
           end
        end
      endcase         
   end

   always_ff @(posedge clk) begin
      if(rst) begin
         dma_wr_state    <= DMA_WR_STATE_IDLE;
         mem_rx_dne_head <= 0;
         mem_tx_dne_head <= 0;
         wr_q_enq_en     <= 0;
      end
      else begin
         dma_wr_state    <= dma_wr_state_nxt;
         mem_tx_dne_head <= mem_tx_dne_head_nxt;
         mem_rx_dne_head <= mem_rx_dne_head_nxt;
         wr_q_enq_en     <= wr_q_enq_en_nxt;
      end

      dma_wr_intr_data <= dma_wr_intr_data_nxt;
      wr_q_enq_data <= wr_q_enq_data_nxt;

   end

   // ----------------------------------------
   // -- Get next RX descriptor
   // ----------------------------------------
   localparam DSC_RX_STATE_IDLE = 0;
   localparam DSC_RX_STATE_L1   = 1;
   localparam DSC_RX_STATE_L2   = 2;
   localparam DSC_RX_STATE_WAIT = 3;

   logic [`MEM_ADDR_BITS-1:0]        mem_rx_dsc_head_reg, mem_rx_dsc_head_reg_nxt;

   logic [63:0]                      dsc_rx_host_ptr, dsc_rx_host_ptr_nxt;
   logic [15:0]                      dsc_rx_buf_len,  dsc_rx_buf_len_nxt;
   logic [15:0]                      dsc_rx_id,  dsc_rx_id_nxt;
   logic [`MEM_ADDR_BITS-1:0]        dsc_rx_addr,     dsc_rx_addr_nxt;
   logic                             dsc_rx_vld,      dsc_rx_vld_nxt;

   logic [63:0]                      dsc_rx_host_ptr_c, dsc_rx_host_ptr_c_nxt;
   logic [15:0]                      dsc_rx_buf_len_c,  dsc_rx_buf_len_c_nxt;
   logic [15:0]                      dsc_rx_id_c,  dsc_rx_id_c_nxt;
   logic [`MEM_ADDR_BITS-1:0]        dsc_rx_addr_c,     dsc_rx_addr_c_nxt;
   logic                             dsc_rx_vld_c,      dsc_rx_vld_c_nxt;

   logic                             dsc_rx_next;   
   logic [1:0]                       dsc_rx_state,    dsc_rx_state_nxt;

   always_comb begin
      dsc_rx_state_nxt = dsc_rx_state;

      dsc_rx_host_ptr_nxt = dsc_rx_host_ptr;
      dsc_rx_buf_len_nxt  = dsc_rx_buf_len;
      dsc_rx_id_nxt  = dsc_rx_id;
      dsc_rx_addr_nxt     = dsc_rx_addr;
      dsc_rx_vld_nxt      = dsc_rx_vld;

      dsc_rx_host_ptr_c_nxt = dsc_rx_host_ptr_c;
      dsc_rx_buf_len_c_nxt  = dsc_rx_buf_len_c;
      dsc_rx_id_c_nxt  = dsc_rx_id_c;
      dsc_rx_addr_c_nxt     = dsc_rx_addr_c;
      dsc_rx_vld_c_nxt      = dsc_rx_vld_c;
  
      mem_rx_dsc_head_nxt = mem_rx_dsc_head;    
      mem_rx_dsc_rd_addr  = mem_rx_dsc_head;
      mem_rx_dsc_rd_en    = 1;
      
      mem_vld_rx_dsc_wr_clear = 0;
      mem_vld_rx_dsc_wr_mask  = 1 << mem_rx_dsc_head_reg[10:6];
      mem_vld_rx_dsc_wr_addr  = mem_rx_dsc_head_reg[`MEM_ADDR_BITS-1:11];
      
      mem_rx_dsc_head_reg_nxt = mem_rx_dsc_head_reg;
      
      if(~dsc_rx_vld || dsc_rx_next) begin
         dsc_rx_vld_nxt = 0;
         if(dsc_rx_vld_c) begin
            dsc_rx_host_ptr_nxt = dsc_rx_host_ptr_c;
            dsc_rx_buf_len_nxt  = dsc_rx_buf_len_c;
            dsc_rx_id_nxt  = dsc_rx_id_c;
            dsc_rx_addr_nxt     = dsc_rx_addr_c;
            dsc_rx_vld_nxt      = 1;
            dsc_rx_vld_c_nxt    = 0;
         end
      end
      
      case(dsc_rx_state)
        DSC_RX_STATE_IDLE: begin
           if(~dsc_rx_vld_c) begin
              if(mem_vld_rx_dsc_rd_bit) begin
                 dsc_rx_state_nxt    = DSC_RX_STATE_L1;
                 mem_rx_dsc_head_nxt = (mem_rx_dsc_head + 8) & rx_dsc_mask[`MEM_ADDR_BITS-1:0];
                 mem_rx_dsc_head_reg_nxt = mem_rx_dsc_head;
              end
           end
        end
        DSC_RX_STATE_L1: begin
           dsc_rx_buf_len_c_nxt  = mem_rx_dsc_rd_data[63:48];
           dsc_rx_id_c_nxt = mem_rx_dsc_rd_data[47:32];
           dsc_rx_addr_c_nxt     = mem_rx_dsc_rd_data[0+:`MEM_ADDR_BITS];
           mem_rx_dsc_head_nxt   = (mem_rx_dsc_head + 56) & rx_dsc_mask[`MEM_ADDR_BITS-1:0];
           dsc_rx_state_nxt      = DSC_RX_STATE_L2;
        end
        DSC_RX_STATE_L2: begin
           dsc_rx_host_ptr_c_nxt   = mem_rx_dsc_rd_data;
           dsc_rx_vld_c_nxt        = 1;
           // invalidate this descriptor line
           if(~mem_vld_rx_dsc_wr_stall) begin
              mem_vld_rx_dsc_wr_clear = 1;
              dsc_rx_state_nxt        = DSC_RX_STATE_IDLE;
           end
           else begin
              dsc_rx_state_nxt        = DSC_RX_STATE_WAIT;
           end
        end
        DSC_RX_STATE_WAIT: begin
           if(~mem_vld_rx_dsc_wr_stall) begin
              mem_vld_rx_dsc_wr_clear  = 1;
              dsc_rx_state_nxt = DSC_RX_STATE_IDLE;
           end
        end
      endcase
   end

   always_ff @(posedge clk) begin
      if(rst) begin
         dsc_rx_state        <= DSC_RX_STATE_IDLE; 
         dsc_rx_vld          <= 0;
         dsc_rx_vld_c        <= 0;
         mem_rx_dsc_head     <= 0;
     end
      else begin
         dsc_rx_state        <= dsc_rx_state_nxt;
         dsc_rx_vld          <= dsc_rx_vld_nxt;
         dsc_rx_vld_c        <= dsc_rx_vld_c_nxt;
         mem_rx_dsc_head     <= mem_rx_dsc_head_nxt;
      end

      dsc_rx_host_ptr     <= dsc_rx_host_ptr_nxt;
      dsc_rx_buf_len      <= dsc_rx_buf_len_nxt;
      dsc_rx_id           <= dsc_rx_id_nxt;
      dsc_rx_addr         <= dsc_rx_addr_nxt;
      dsc_rx_host_ptr_c   <= dsc_rx_host_ptr_c_nxt;
      dsc_rx_buf_len_c    <= dsc_rx_buf_len_c_nxt;
      dsc_rx_id_c         <= dsc_rx_id_c_nxt;
      dsc_rx_addr_c       <= dsc_rx_addr_c_nxt;

      mem_rx_dsc_head_reg <= mem_rx_dsc_head_reg_nxt;
   end


   // ----------------------------------
   // -- Shift input S_AXIS data
   // ----------------------------------
   logic [63:0]                 S_AXIS_TDATA_L0; // just flopped S_AXIS_TDATA
   logic [7:0]                  S_AXIS_TSTRB_L0;
   logic                        S_AXIS_TVALID_L0;
   logic                        S_AXIS_TREADY_L0;
   logic                        S_AXIS_TLAST_L0;
   logic [127:0]                S_AXIS_TUSER_L0;

   logic [63:0]                 S_AXIS_TDATA_L1; // flopped and shifted S_AXIS_TDATA
   logic [7:0]                  S_AXIS_TSTRB_L1;
   logic                        S_AXIS_TVALID_L1;
   logic                        S_AXIS_TREADY_L1;
   logic                        S_AXIS_TLAST_L1;
   logic [127:0]                S_AXIS_TUSER_L1;

   logic [1:0]                  s_axis_shift_by, s_axis_shift_by_reg;

   rx_pkt_shift u_rx_shift(.in_tdata(S_AXIS_TDATA),
                           .in_tstrb(S_AXIS_TSTRB),
                           .in_tvalid(S_AXIS_TVALID),
                           .in_tready(S_AXIS_TREADY),
                           .in_tlast(S_AXIS_TLAST),
                           .in_tuser(S_AXIS_TUSER),
                           .out_tdata(S_AXIS_TDATA_L0),
                           .out_tstrb(S_AXIS_TSTRB_L0),
                           .out_tvalid(S_AXIS_TVALID_L0),
                           .out_tready(S_AXIS_TREADY_L0),
                           .out_tlast(S_AXIS_TLAST_L0),
                           .out_tuser(S_AXIS_TUSER_L0),
                           .shift_tdata(S_AXIS_TDATA_L1),
                           .shift_tstrb(S_AXIS_TSTRB_L1),
                           .shift_tvalid(S_AXIS_TVALID_L1),
                           .shift_tready(S_AXIS_TREADY_L1),
                           .shift_tlast(S_AXIS_TLAST_L1),
                           .shift_tuser(S_AXIS_TUSER_L1),
                           .shift_by(s_axis_shift_by),
                           .clk(clk),
                           .rst(rst)
                           );
   
   // ----------------------------------------
   // -- Process packet coming from the MAC
   // ----------------------------------------
   localparam MAC_RX_STATE_PREP  = 0;
   localparam MAC_RX_STATE_IDLE  = 1;
   localparam MAC_RX_STATE_DATA  = 2;

   logic [1:0]                 mac_rx_state, mac_rx_state_nxt;
   logic [15:0]                pkt_size_cnt, pkt_size_cnt_nxt; // increased width to 16 bits to support Jumbo Frames
   logic [15:0]                pkt_dma_rem_cnt, pkt_dma_rem_cnt_nxt;
   logic [12:0]                pkt_line_cnt, pkt_line_cnt_nxt;  // should also work for packets <= 65535
   
   logic [63:0]                dsc_rx_host_ptr_reg, dsc_rx_host_ptr_reg_nxt;
   logic [15:0]                dsc_rx_buf_len_reg, dsc_rx_buf_len_reg_nxt;
   logic [15:0]                dsc_rx_id_reg, dsc_rx_id_reg_nxt;
   logic [`MEM_ADDR_BITS-1:0]  dsc_rx_addr_reg, dsc_rx_addr_reg_nxt;

   logic                       pkt_buffer_full, pkt_buffer_full_nxt;
   logic [15:0]                pkt_port, pkt_port_nxt;

   always_comb begin
      mac_rx_state_nxt = mac_rx_state;
      pkt_size_cnt_nxt = pkt_size_cnt;
      pkt_dma_rem_cnt_nxt  = pkt_dma_rem_cnt;
      pkt_line_cnt_nxt = pkt_line_cnt;
      s_axis_shift_by  = s_axis_shift_by_reg;
      
      dsc_rx_next  = 0;

      mem_rx_pkt_wr_addr  = mem_rx_pkt_tail;
      mem_rx_pkt_wr_mask  = 8'hff;
      mem_rx_pkt_wr_data  = 0;
      mem_rx_pkt_wr_en    = 0;
      mem_rx_pkt_tail_nxt = mem_rx_pkt_tail;      

      mem_rx_dne_wr_en     = 0;
      mem_rx_dne_wr_mask   = 8'hff;
      mem_rx_dne_wr_data   = 0;
      mem_rx_dne_wr_addr   = mem_rx_dne_tail;
      mem_rx_dne_tail_nxt  = mem_rx_dne_tail;

      // clear 7 entries ahead, make sure buffer is at least 16 lines deep
      mem_rx_dne_clear        = (mem_rx_dne_tail + 64*7) & rx_dne_mask[`MEM_ADDR_BITS-1:0];
      mem_vld_rx_dne_wr_addr  = mem_rx_dne_clear[`MEM_ADDR_BITS-1:11];
      mem_vld_rx_dne_wr_mask  = 1 << mem_rx_dne_clear[10:6];
      mem_vld_rx_dne_wr_clear = 0;

      dma_wr_go_nxt         = 0;
      dma_wr_local_addr_nxt = dsc_rx_addr_reg;
      dma_wr_host_addr_nxt  = dsc_rx_host_ptr_reg;
      dma_wr_len_nxt        = dma_wr_len;
      dma_rx_byte_nxt       = dma_rx_byte;
  
      dsc_rx_host_ptr_reg_nxt = dsc_rx_host_ptr_reg;
      dsc_rx_buf_len_reg_nxt = dsc_rx_buf_len_reg;
      dsc_rx_id_reg_nxt = dsc_rx_id_reg;
      dsc_rx_addr_reg_nxt = dsc_rx_addr_reg;
      
      S_AXIS_TREADY_L0 = 1;
      S_AXIS_TREADY_L1 = 1;

      pkt_buffer_full_nxt = 0;
      pkt_port_nxt = pkt_port;
      
      stat_mac_rx_err_cnt_nxt  = stat_mac_rx_err_cnt;

      case(mac_rx_state)
        MAC_RX_STATE_PREP: begin
           S_AXIS_TREADY_L0 = 0;
           S_AXIS_TREADY_L1 = 0;
           if(dsc_rx_vld) begin
              s_axis_shift_by = dsc_rx_addr[1:0];
              mac_rx_state_nxt = MAC_RX_STATE_IDLE;
           end
        end
        MAC_RX_STATE_IDLE: begin
           if(S_AXIS_TVALID_L0 && S_AXIS_TVALID_L1 && (S_AXIS_TSTRB_L0 == 8'hff)) begin
              mac_rx_state_nxt = MAC_RX_STATE_DATA;                 
              pkt_port_nxt     = S_AXIS_TUSER_L1[31:16];

              pkt_line_cnt_nxt = 'd1;

              case(S_AXIS_TSTRB_L1[7:4])
                4'hf: begin
                   pkt_size_cnt_nxt = 'd8;
                   mem_rx_pkt_wr_data = S_AXIS_TDATA_L1;
                end
                4'h7: begin
                   pkt_size_cnt_nxt = 'd7;
                   mem_rx_pkt_wr_data = {S_AXIS_TDATA_L1[55:0], 8'b0};
                end
                4'h3: begin
                   pkt_size_cnt_nxt = 'd6;
                   mem_rx_pkt_wr_data = {S_AXIS_TDATA_L1[47:0], 16'b0};
                end
                4'h1: begin
                   pkt_size_cnt_nxt = 'd5;
                   mem_rx_pkt_wr_data = {S_AXIS_TDATA_L1[39:0], 24'b0};
                end
                default: begin
                   pkt_size_cnt_nxt = 'd0;
                   mem_rx_pkt_wr_data = 64'b0;
                end
              endcase
              pkt_dma_rem_cnt_nxt = pkt_size_cnt_nxt;

              mem_rx_pkt_wr_mask = 8'hff;
              mem_rx_pkt_wr_en    = 1;
              mem_rx_pkt_wr_addr  = {dsc_rx_addr[`MEM_ADDR_BITS-1:2], 2'b0};
              mem_rx_pkt_tail_nxt = ({dsc_rx_addr[`MEM_ADDR_BITS-1:2], 2'b0} + 8) & rx_pkt_mask[`MEM_ADDR_BITS-1:0];

              // store dma data
              dsc_rx_addr_reg_nxt = dsc_rx_addr;
              dsc_rx_host_ptr_reg_nxt = dsc_rx_host_ptr;
              dsc_rx_buf_len_reg_nxt = dsc_rx_buf_len  - {12'b0, 2'b10, dsc_rx_addr[1:0]}; // make sure there is space for the last two lines and offset bits
              dsc_rx_id_reg_nxt = dsc_rx_id;
              dsc_rx_next = 1;
              dma_rx_byte_nxt = rx_byte_wait - {14'b0, dsc_rx_addr[1:0]};
              
           end              
        end
        MAC_RX_STATE_DATA: begin

           // Check for buffer overflow
           if(pkt_size_cnt >= dsc_rx_buf_len_reg)
             pkt_buffer_full_nxt = 1;

           if( dma_wr_rdy & ~mem_vld_rx_dne_wr_stall ) begin

              // 64bit received
              if(S_AXIS_TVALID_L0) begin
                 pkt_line_cnt_nxt = pkt_line_cnt + 'd1;                 
              end

              // Descriptor buffer receive
              if(S_AXIS_TVALID_L1) begin
                 if(~pkt_buffer_full) begin
                    if(S_AXIS_TLAST_L1) begin
                       mem_rx_pkt_wr_mask  = 8'hff;
                       mem_rx_pkt_wr_en    = 1;
                       mem_rx_pkt_tail_nxt = (mem_rx_pkt_tail + 8) & rx_pkt_mask[`MEM_ADDR_BITS-1:0];

                       // next state
                       if(dsc_rx_vld && (|pkt_line_cnt[12:3])) begin // (pkt_line_cnt > 7)
                          s_axis_shift_by = dsc_rx_addr[1:0];
                          mac_rx_state_nxt = MAC_RX_STATE_IDLE;
                       end
                       else begin
                          mac_rx_state_nxt = MAC_RX_STATE_PREP;
                       end

                       // generate RX interrupt
                       mem_rx_dne_wr_en = 1;
                       mem_rx_dne_wr_mask = 8'hff;
                       mem_rx_dne_tail_nxt = (mem_rx_dne_tail + 64) & rx_dne_mask[`MEM_ADDR_BITS-1:0];
                       mem_vld_rx_dne_wr_clear = 1;
                       mem_rx_dne_wr_data[31:16] = pkt_port;
                       mem_rx_dne_wr_data[47:32] = dsc_rx_id_reg;
                       
                       case(S_AXIS_TSTRB_L1)
                         8'h01: begin
                            mem_rx_dne_wr_data[15:0] = pkt_size_cnt + 16'd1;
                            mem_rx_pkt_wr_data = {56'b0, S_AXIS_TDATA_L1[7:0]};
                            dma_wr_len_nxt     = pkt_dma_rem_cnt + 16'd1;
                         end
                         8'h03: begin
                            mem_rx_dne_wr_data[15:0] = pkt_size_cnt + 16'd2;
                            mem_rx_pkt_wr_data = {48'b0, S_AXIS_TDATA_L1[15:0]};
                            dma_wr_len_nxt     = pkt_dma_rem_cnt + 16'd2;
                         end
                         8'h07: begin
                            mem_rx_dne_wr_data[15:0] = pkt_size_cnt + 16'd3;
                            mem_rx_pkt_wr_data = {40'b0, S_AXIS_TDATA_L1[23:0]};
                            dma_wr_len_nxt     = pkt_dma_rem_cnt + 16'd3;
                         end
                         8'h0f: begin
                            mem_rx_dne_wr_data[15:0] = pkt_size_cnt + 16'd4;
                            mem_rx_pkt_wr_data = {32'b0, S_AXIS_TDATA_L1[31:0]};
                            dma_wr_len_nxt     = pkt_dma_rem_cnt + 16'd4;
                         end
                         8'h1f: begin
                            mem_rx_dne_wr_data[15:0] = pkt_size_cnt + 16'd5;
                            mem_rx_pkt_wr_data = {24'b0, S_AXIS_TDATA_L1[39:0]};
                            dma_wr_len_nxt     = pkt_dma_rem_cnt + 16'd5;
                         end
                         8'h3f: begin
                            mem_rx_dne_wr_data[15:0] = pkt_size_cnt + 16'd6;
                            mem_rx_pkt_wr_data = {16'b0, S_AXIS_TDATA_L1[47:0]};
                            dma_wr_len_nxt     = pkt_dma_rem_cnt + 16'd6;
                         end
                         8'h7f: begin
                            mem_rx_dne_wr_data[15:0] = pkt_size_cnt + 16'd7;
                            mem_rx_pkt_wr_data = {8'b0, S_AXIS_TDATA_L1[55:0]};
                            dma_wr_len_nxt     = pkt_dma_rem_cnt + 16'd7;
                         end
                         8'hff: begin
                            mem_rx_dne_wr_data[15:0] = pkt_size_cnt + 16'd8;
                            mem_rx_pkt_wr_data = S_AXIS_TDATA_L1;
                            dma_wr_len_nxt     = pkt_dma_rem_cnt + 16'd8;
                         end
                         default: begin
                            mem_rx_dne_wr_data[15:0] = pkt_size_cnt;
                            mem_rx_pkt_wr_data = S_AXIS_TDATA_L1;
                            dma_wr_len_nxt     = pkt_dma_rem_cnt;
                         end
                       endcase
                                              
                       // DMA write the packet
                       if(|dsc_rx_host_ptr_reg) begin
                          dma_wr_go_nxt = 1;
                       end

                    end
					// removed code here that made the hw and sw descriptors get out of sync if the packet size was below 64 bytes); this is now handled in software
                    else begin // tlast not active (assume that tstrb = 0xff)
                       pkt_size_cnt_nxt = pkt_size_cnt + 'd8;
                       pkt_dma_rem_cnt_nxt = pkt_dma_rem_cnt + 'd8;
                       
                       mem_rx_pkt_wr_mask  = 8'hff;
                       mem_rx_pkt_wr_data  = S_AXIS_TDATA_L1;
                       mem_rx_pkt_wr_en    = 1;
                       mem_rx_pkt_tail_nxt = (mem_rx_pkt_tail + 8) & rx_pkt_mask[`MEM_ADDR_BITS-1:0];

                       // DMA write the packet if enough has been received
                       if(pkt_dma_rem_cnt >= rx_byte_wait) begin
                          if(|dsc_rx_host_ptr_reg) begin
                             dma_wr_go_nxt           = 1;
                             dma_wr_len_nxt          = dma_rx_byte;
                             
                             dsc_rx_addr_reg_nxt     = dsc_rx_addr_reg + {{(`MEM_ADDR_BITS-16){1'b0}}, dma_rx_byte};
                             dsc_rx_host_ptr_reg_nxt = dsc_rx_host_ptr_reg + {48'b0, dma_rx_byte};
                             pkt_dma_rem_cnt_nxt     = pkt_dma_rem_cnt + 'd8 - dma_rx_byte;
                             dma_rx_byte_nxt         = rx_byte_wait;
                          end
                       end                       
                    end
                 end
                 else begin // buffer overflow, send max possible
                    if(S_AXIS_TLAST_L1) begin
                       mac_rx_state_nxt = MAC_RX_STATE_PREP;  

                       // generate RX interrupt
                       mem_rx_dne_wr_en = 1;
                       mem_rx_dne_wr_mask = 8'hff;
                       mem_rx_dne_tail_nxt = (mem_rx_dne_tail + 64) & rx_dne_mask[`MEM_ADDR_BITS-1:0];
                       mem_rx_dne_wr_data[15:0] = pkt_size_cnt - 'd8;
                       mem_vld_rx_dne_wr_clear = 1;
                       mem_rx_dne_wr_data[31:16] = pkt_port;
                       mem_rx_dne_wr_data[47:32] = dsc_rx_id_reg;

                       // DMA write the packet
                       if(pkt_dma_rem_cnt > 'd8) begin
                          dma_wr_len_nxt = pkt_dma_rem_cnt - 'd8;
                          if(|dsc_rx_host_ptr_reg) begin
                             dma_wr_go_nxt = 1;
                          end
                       end
                       
                       stat_mac_rx_err_cnt_nxt = stat_mac_rx_err_cnt + 1;                 
                    end
                 end
              end
           end
           else begin
              S_AXIS_TREADY_L0 = 0;
              S_AXIS_TREADY_L1 = 0;
           end
        end
        default: begin
           mac_rx_state_nxt = MAC_RX_STATE_PREP;
        end
      endcase      
   end

   always_ff @(posedge clk) begin
      if(rst) begin
         mac_rx_state     <= MAC_RX_STATE_PREP;
         pkt_size_cnt     <= 0;
         pkt_dma_rem_cnt  <= 0;
         pkt_line_cnt     <= 0;
         mem_rx_pkt_tail  <= 0;
         mem_rx_dne_tail  <= 0;
         dma_wr_go        <= 0;
         stat_mac_rx_err_cnt <= 0;
      end
      else begin
         mac_rx_state     <= mac_rx_state_nxt;
         pkt_size_cnt     <= pkt_size_cnt_nxt;
         pkt_dma_rem_cnt  <= pkt_dma_rem_cnt_nxt;
         pkt_line_cnt     <= pkt_line_cnt_nxt;
         mem_rx_pkt_tail  <= mem_rx_pkt_tail_nxt;
         mem_rx_dne_tail  <= mem_rx_dne_tail_nxt;
         dma_wr_go        <= dma_wr_go_nxt;
         stat_mac_rx_err_cnt <= stat_mac_rx_err_cnt_nxt;
      end
      dma_wr_len          <= dma_wr_len_nxt;
      dma_rx_byte         <= dma_rx_byte_nxt;
      s_axis_shift_by_reg <= s_axis_shift_by;
      dsc_rx_addr_reg     <= dsc_rx_addr_reg_nxt;
      dsc_rx_host_ptr_reg <= dsc_rx_host_ptr_reg_nxt;
      dsc_rx_buf_len_reg  <= dsc_rx_buf_len_reg_nxt;
      dsc_rx_id_reg       <= dsc_rx_id_reg_nxt;
      pkt_buffer_full     <= pkt_buffer_full_nxt;
      dma_wr_local_addr   <= dma_wr_local_addr_nxt;
      dma_wr_host_addr    <= dma_wr_host_addr_nxt;
      pkt_port            <= pkt_port_nxt;
   end


`ifdef DEBUG_CHIPSCOPE

wire axis_error_0;
axis_conform_check_pkg::axi_errors_t axis_errors_0;

axis_conform_check #(

	.axis_data_width(64)

) axis_conform_check_0 (

	.clk(clk),
	.reset(rst),
	.axis_tkeep(S_AXIS_TSTRB),
	.axis_tlast(S_AXIS_TLAST),
	.axis_tvalid(S_AXIS_TVALID),
	.axis_tready(S_AXIS_TREADY),

	.error(axis_error_0), // 1 bit error signal
	.error_vec(axis_errors_0) // 5 bit error code output
);

reg cycle[15:0];

always @(posedge clk)
	if (rst)
		cycle <= 0;
	else
		cycle <= cycle + 1;

wire [127:0] data = {

	axis_errors_0,				// 5 bit
	mem_rx_dne_wr_addr,			// 20 bit
	pkt_buffer_full,
	stat_mac_rx_err_cnt[7:0],	// 8 bit
	mem_rx_dne_wr_en,
	mac_rx_state,				// 2 bit
	dsc_rx_id_reg[7:0],			// 8 bit
	dsc_rx_id[7:0],				// 8 bit
	dsc_rx_id_c[7:0],			// 8 bit
	dsc_rx_next,
	dsc_rx_vld_c,
	dsc_rx_vld,
	mem_rx_dsc_head_reg,		// 20 bit
	mem_vld_rx_dsc_wr_clear,
	mem_vld_rx_dsc_wr_stall,
	dsc_rx_state,				// 2 bit
	mem_vld_rx_dsc_rd_bit
//	cycle						// 16 bit
};

wire changed = data_reg != data_reg_2;

wire [15:0] trig = {

	axis_error_0,
	mem_rx_dne_wr_en,
	mac_rx_state,				// 2 bit
	dsc_rx_next,
	dsc_rx_vld,
	mem_vld_rx_dsc_wr_clear,
	dsc_rx_state,				// 2 bit
	mem_vld_rx_dsc_rd_bit,
	changed
};

reg [127:0] data_reg, data_reg_2;
reg [15:0] trig_reg;

always @(posedge clk) begin
	data_reg <= data;
	data_reg_2 <= data_reg;
	trig_reg <= trig;
end

chipscope_ila_128 axi_lite_ila_inst (
	.CONTROL(chipscope_control_0),
	.CLK(clk),
	.DATA({data_reg, cycle}),
	.TRIG0(trig_reg)
);

`else

assign chipscope_control_0 = 0;

`endif

   
endmodule

// Module that shifts inputs from S_AXIS
// This is needed if the host buffer is not 4-byte aligned :(
module rx_pkt_shift
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

   output logic [63:0]  shift_tdata,
   output logic [7:0]   shift_tstrb,
   output logic         shift_tvalid,
   input logic          shift_tready,
   output logic         shift_tlast, 
   output logic [127:0] shift_tuser,
   input logic [1:0]    shift_by,
   
   input logic          clk,
   input logic          rst
   );

   localparam STATE_IDLE = 0;
   localparam STATE_BODY = 1;
   localparam STATE_LAST = 2;

   logic [1:0]         state, state_nxt, state_d;

   logic [63:0]        shift_tdata_nxt;
   logic [7:0]         shift_tstrb_nxt;
   logic               shift_tvalid_nxt;
   logic               shift_tlast_nxt;
   logic [1:0]         shift_by_reg, shift_by_reg_nxt;

   logic [63:0]        shift_tdata_last, shift_tdata_last_nxt;
   logic [7:0]         shift_tstrb_last, shift_tstrb_last_nxt;

   logic               out_tvalid_nxt;
   logic               out_tlast_nxt;


   always_comb begin
      state_nxt = state;
      shift_by_reg_nxt = shift_by_reg;
      
      shift_tvalid_nxt = 0;
      shift_tdata_nxt  = shift_tdata;
      shift_tstrb_nxt  = shift_tstrb;
      shift_tlast_nxt  = 0;

      shift_tdata_last_nxt = shift_tdata_last;
      shift_tstrb_last_nxt = shift_tstrb_last;;

      out_tvalid_nxt = in_tvalid;
      out_tlast_nxt  = in_tlast;
      
      in_tready = 1;

      if(~shift_tready | ~out_tready) begin
         shift_tvalid_nxt = shift_tvalid;
         shift_tlast_nxt  = shift_tlast;
         out_tvalid_nxt = out_tvalid;
         out_tlast_nxt  = out_tlast;
         in_tready = 0;
         // if we got here straight from IDLE we might need reshifting
         if(shift_tvalid && (state_d == STATE_IDLE)) begin
            shift_by_reg_nxt = shift_by;
            state_nxt = STATE_BODY;
            case(shift_by)
              2'h0: begin
                 shift_tdata_nxt = out_tdata;
                 shift_tstrb_nxt = out_tstrb;
                 shift_tlast_nxt = 0;
              end
              2'h1: begin
                 shift_tdata_nxt = {8'h0, out_tdata[55:0]};
                 shift_tstrb_nxt = {1'h0, out_tstrb[6:0]};
                 shift_tlast_nxt = 0;
              end
              2'h2: begin
                 shift_tdata_nxt = {16'h0, out_tdata[47:0]};
                 shift_tstrb_nxt = {2'h0, out_tstrb[5:0]};
                 shift_tlast_nxt = 0;
              end        
              2'h3: begin
                 shift_tdata_nxt = {24'h0, out_tdata[39:0]};
                 shift_tstrb_nxt = {3'h0, out_tstrb[4:0]};
                 shift_tlast_nxt = 0;
              end
            endcase            
         end
      end
      else begin
         case(state)
           STATE_IDLE: begin
              if(in_tvalid & ~in_tlast) begin
                 shift_tvalid_nxt = 1;
                 shift_by_reg_nxt = shift_by;
                 state_nxt = STATE_BODY;
                 case(shift_by)
                   2'h0: begin
                      shift_tdata_nxt = in_tdata;
                      shift_tstrb_nxt = in_tstrb;
                      shift_tlast_nxt = 0;
                   end
                   2'h1: begin
                      shift_tdata_nxt = {8'h0, in_tdata[55:0]};
                      shift_tstrb_nxt = {1'h0, in_tstrb[6:0]};
                      shift_tlast_nxt = 0;
                   end
                   2'h2: begin
                      shift_tdata_nxt = {16'h0, in_tdata[47:0]};
                      shift_tstrb_nxt = {2'h0, in_tstrb[5:0]};
                      shift_tlast_nxt = 0;
                   end        
                   2'h3: begin
                      shift_tdata_nxt = {24'h0, in_tdata[39:0]};
                      shift_tstrb_nxt = {3'h0, in_tstrb[4:0]};
                      shift_tlast_nxt = 0;
                   end
                 endcase
              end
           end
           STATE_BODY: begin
              if(in_tvalid) begin
                 shift_tvalid_nxt = 1;
                 case(shift_by_reg)
                   2'h0: begin
                      shift_tdata_nxt = in_tdata;
                      shift_tstrb_nxt = in_tstrb;
                      if(in_tlast) begin
                         shift_tlast_nxt = 1;
                         state_nxt = STATE_IDLE;
                      end
                      else begin
                         shift_tlast_nxt = 0;
                         state_nxt = STATE_BODY;
                      end
                   end
                   2'h1: begin
                      shift_tdata_nxt = {in_tdata[55:0], out_tdata[63:56]};
                      shift_tstrb_nxt = {in_tstrb[6:0], out_tstrb[7]};

                      if(in_tlast) begin
                         if(in_tstrb[7]) begin
                            shift_tdata_last_nxt = {56'b0, in_tdata[63:56]};
                            shift_tstrb_last_nxt = {7'b0, in_tstrb[7]};
                            shift_tlast_nxt = 0;
                            state_nxt = STATE_LAST;                              
                         end
                         else begin
                            shift_tlast_nxt = 1;
                            state_nxt = STATE_IDLE;                              
                         end                         
                      end
                      else begin
                         shift_tlast_nxt = 0;
                         state_nxt = STATE_BODY;
                      end
                   end
                   2'h2: begin
                      shift_tdata_nxt = {in_tdata[47:0], out_tdata[63:48]};
                      shift_tstrb_nxt = {in_tstrb[5:0], out_tstrb[7:6]};
                      if(in_tlast) begin
                         if(in_tstrb[6]) begin
                            shift_tdata_last_nxt = {48'b0, in_tdata[63:48]};
                            shift_tstrb_last_nxt = {6'b0, in_tstrb[7:6]};
                            shift_tlast_nxt = 0;
                            state_nxt = STATE_LAST;                              
                         end
                         else begin
                            shift_tlast_nxt = 1;
                            state_nxt = STATE_IDLE;                              
                         end                         
                      end
                      else begin
                         shift_tlast_nxt = 0;
                         state_nxt = STATE_BODY;
                      end
                   end        
                   2'h3: begin
                      shift_tdata_nxt = {in_tdata[39:0], out_tdata[63:40]};
                      shift_tstrb_nxt = {in_tstrb[4:0], out_tstrb[7:5]};
                      if(in_tlast) begin
                         if(in_tstrb[5]) begin
                            shift_tdata_last_nxt = {40'b0, in_tdata[63:40]};
                            shift_tstrb_last_nxt = {5'b0, in_tstrb[7:5]};
                            shift_tlast_nxt = 0;
                            state_nxt = STATE_LAST;                              
                         end
                         else begin
                            shift_tlast_nxt = 1;
                            state_nxt = STATE_IDLE;                              
                         end                         
                      end
                      else begin
                         shift_tlast_nxt = 0;
                         state_nxt = STATE_BODY;
                      end
                   end
                 endcase
              end
           end
           STATE_LAST: begin
              in_tready = 0;
              shift_tdata_nxt = shift_tdata_last;
              shift_tstrb_nxt = shift_tstrb_last;
              shift_tlast_nxt = 1;
              shift_tvalid_nxt = 1;
              
              state_nxt = STATE_IDLE;
           end
           default: state_nxt = STATE_IDLE;
         endcase
      end
   end   

   always_ff @(posedge clk) begin
      if(rst) begin
         state <= STATE_IDLE;
      end
      else begin
         state <= state_nxt;
      end

      if(shift_tready & out_tready & in_tvalid) begin
         state_d <= state;
         out_tdata  <= in_tdata;
         out_tstrb  <= in_tstrb;
      end
      out_tvalid <= out_tvalid_nxt;
      out_tlast  <= out_tlast_nxt;
      out_tuser  <= in_tuser;
      
      shift_tdata  <= shift_tdata_nxt;
      shift_tstrb  <= shift_tstrb_nxt;
      shift_tvalid <= shift_tvalid_nxt;
      shift_tlast  <= shift_tlast_nxt;
      shift_tdata_last  <= shift_tdata_last_nxt;
      shift_tstrb_last  <= shift_tstrb_last_nxt;

      shift_tuser <= in_tuser;

      shift_by_reg <= shift_by_reg_nxt;
   end

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

