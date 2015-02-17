/*******************************************************************************
 *
 *  NetFPGA-10G http://www.netfpga.org
 *
 *  File:
 *        cfg.v
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
 *        Configuration module for the DMA engine. This module contains 
 *        configuration registers that are written through the PCIe interface
 *        by the host. Also, this module resolves all the clock domain crossins
 *        for those registers.
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

module cfg
   (
    // memory interface signals valid
    input logic                      wr_mem_valid,
    input logic                      rd_mem_valid,
    
    // memory write interface
    input logic [`MEM_ADDR_BITS-1:0] wr_addr_hi,
    input logic [31:0]               wr_data_hi,
    input logic [3:0]                wr_mask_hi,
    input logic                      wr_en_hi,
    input logic [`MEM_ADDR_BITS-1:0] wr_addr_lo,
    input logic [31:0]               wr_data_lo,
    input logic [3:0]                wr_mask_lo,
    input logic                      wr_en_lo,

    // memory read interface
    input logic [`MEM_ADDR_BITS-1:0] rd_addr_hi,
    output logic [31:0]              rd_data_hi,
    input logic                      rd_en_hi,
    input logic [`MEM_ADDR_BITS-1:0] rd_addr_lo,
    output logic [31:0]              rd_data_lo,
    input logic                      rd_en_lo,

    // config registers output
    // tx_clk
    output logic [63:0]              tx_dsc_mask,
    output logic [63:0]              tx_pkt_mask,
    output logic [63:0]              tx_dne_mask,
    output logic [63:0]              rx_dsc_mask_t, 

    // rx_clk
    output logic [63:0]              tx_dne_mask_r,
    output logic [63:0]              rx_dsc_mask, 
    output logic [63:0]              rx_pkt_mask,
    output logic [63:0]              rx_dne_mask,

    // rx_clk
    output logic [63:0]              host_tx_dne_offset,
    output logic [63:0]              host_tx_dne_mask,
    output logic [63:0]              host_rx_dne_offset,
    output logic [63:0]              host_rx_dne_mask,

    // rx_clk
    output logic [15:0]              rx_byte_wait,
    output logic                     tx_int_enable,
    output logic                     rx_int_enable,

    // axi_clk
    output logic [31:0]              axi_rdwr_addr,
    input logic [31:0]               axi_rd_data,
    output logic [31:0]              axi_wr_data,
    output logic                     axi_rd_go,
    output logic                     axi_wr_go,
    input logic                      axi_rd_done,
    input logic                      axi_wr_done, 
    input logic                      axi_error,
   
    // pcie_clk
    output logic                     soft_reset,
    output logic                     mem_cfg_rd_valid,

    input logic                      axi_clk,
    input logic                      tx_clk,
    input logic                      rx_clk,
    input logic                      pcie_clk,
    input logic                      rst
    );

   // local config regs (pcie_clk)
   logic [63:0]                      tx_dsc_mask_l;
   logic [63:0]                      tx_pkt_mask_l;
   logic [63:0]                      tx_dne_mask_l;

   logic [63:0]                      rx_dsc_mask_l;
   logic [63:0]                      rx_pkt_mask_l;
   logic [63:0]                      rx_dne_mask_l;

   logic [63:0]                      host_tx_dne_offset_l;
   logic [63:0]                      host_tx_dne_mask_l;
   logic [63:0]                      host_rx_dne_offset_l;
   logic [63:0]                      host_rx_dne_mask_l;

   logic [15:0]                      rx_byte_wait_l;
   
   logic                             tx_int_enable_l;
   logic                             rx_int_enable_l;

   logic [31:0]                      axi_rd_addr_l;
   logic [31:0]                      axi_wr_addr_l;
   logic [31:0]                      axi_rd_data_l;
   logic [31:0]                      axi_wr_data_l;
   logic                             axi_rd_go_l;
   logic                             axi_wr_go_l;

   
   // ---------------------------
   // -- clock domain crossing
   // ---------------------------
   x_signal #(64) x_cfg_0(pcie_clk, tx_dsc_mask_l, tx_clk, tx_dsc_mask);
   x_signal #(64) x_cfg_2(pcie_clk, tx_pkt_mask_l, tx_clk, tx_pkt_mask);
   x_signal #(64) x_cfg_3(pcie_clk, tx_dne_mask_l, tx_clk, tx_dne_mask);
   x_signal #(64) x_cfg_4(pcie_clk, rx_dsc_mask_l, tx_clk, rx_dsc_mask_t);

   x_signal #(64) x_cfg_5(pcie_clk, tx_dne_mask_l, rx_clk, tx_dne_mask_r);
   x_signal #(64) x_cfg_6(pcie_clk, rx_dsc_mask_l, rx_clk, rx_dsc_mask);
   x_signal #(64) x_cfg_8(pcie_clk, rx_pkt_mask_l, rx_clk, rx_pkt_mask);
   x_signal #(64) x_cfg_9(pcie_clk, rx_dne_mask_l, rx_clk, rx_dne_mask);

   x_signal #(64) x_cfg_10(pcie_clk, host_tx_dne_offset_l, rx_clk, host_tx_dne_offset);
   x_signal #(64) x_cfg_11(pcie_clk, host_tx_dne_mask_l,   rx_clk, host_tx_dne_mask);
   x_signal #(64) x_cfg_12(pcie_clk, host_rx_dne_offset_l, rx_clk, host_rx_dne_offset);
   x_signal #(64) x_cfg_13(pcie_clk, host_rx_dne_mask_l,   rx_clk, host_rx_dne_mask);

   x_signal #(16) x_cfg_14(pcie_clk, rx_byte_wait_l, rx_clk, rx_byte_wait);
   x_signal #(1) x_cfg_15(pcie_clk, tx_int_enable_l, rx_clk, tx_int_enable);
   x_signal #(1) x_cfg_16(pcie_clk, rx_int_enable_l, rx_clk, rx_int_enable);
   
   // -----------------------
   // -- write logic
   // -----------------------   
   always_ff @(posedge pcie_clk) begin
      integer i;
      if(rst) begin
         tx_dsc_mask_l <= `MEM_N_TX_DSC*64-1;
         tx_pkt_mask_l <= `MEM_N_TX_PKT*64-1;
         tx_dne_mask_l <= `MEM_N_TX_DNE*64-1;
         
         rx_dsc_mask_l <= `MEM_N_RX_DSC*64-1;
         rx_pkt_mask_l <= `MEM_N_RX_PKT*64-1;
         rx_dne_mask_l <= `MEM_N_RX_DNE*64-1;
         
         host_tx_dne_offset_l <= 0;
         host_tx_dne_mask_l   <= 0;
         host_rx_dne_offset_l <= 0;
         host_rx_dne_mask_l   <= 0;

         rx_byte_wait_l  <= 16'hffff;
         tx_int_enable_l <= 1;
         rx_int_enable_l <= 1;

         soft_reset <= 0;
         
      end
      else if(wr_mem_valid) begin
         if(wr_en_lo) begin
            case(wr_addr_lo[`CFG_ADDR_BITS-1:3])
              1: for(i=0; i<4; i++) if(wr_mask_lo[i]) tx_dsc_mask_l[i*8+:8]   <= wr_data_lo[i*8+:8];
              5: for(i=0; i<4; i++) if(wr_mask_lo[i]) tx_pkt_mask_l[i*8+:8]   <= wr_data_lo[i*8+:8];
              7: for(i=0; i<4; i++) if(wr_mask_lo[i]) tx_dne_mask_l[i*8+:8]   <= wr_data_lo[i*8+:8];
              9: for(i=0; i<4; i++) if(wr_mask_lo[i]) rx_dsc_mask_l[i*8+:8]   <= wr_data_lo[i*8+:8];
              13: for(i=0; i<4; i++) if(wr_mask_lo[i]) rx_pkt_mask_l[i*8+:8]   <= wr_data_lo[i*8+:8];
              15: for(i=0; i<4; i++) if(wr_mask_lo[i]) rx_dne_mask_l[i*8+:8]   <= wr_data_lo[i*8+:8];
              16: for(i=0; i<4; i++) if(wr_mask_lo[i]) host_tx_dne_offset_l[i*8+:8] <= wr_data_lo[i*8+:8];
              17: for(i=0; i<4; i++) if(wr_mask_lo[i]) host_tx_dne_mask_l[i*8+:8]   <= wr_data_lo[i*8+:8];
              18: for(i=0; i<4; i++) if(wr_mask_lo[i]) host_rx_dne_offset_l[i*8+:8] <= wr_data_lo[i*8+:8];
              19: for(i=0; i<4; i++) if(wr_mask_lo[i]) host_rx_dne_mask_l[i*8+:8]   <= wr_data_lo[i*8+:8];

              
              24: for(i=0; i<2; i++) if(wr_mask_lo[i]) rx_byte_wait_l[i*8+:8]  <= wr_data_lo[i*8+:8];
              25: for(i=0; i<1; i++) if(wr_mask_lo[i]) tx_int_enable_l <= wr_data_lo[0];
              26: for(i=0; i<1; i++) if(wr_mask_lo[i]) rx_int_enable_l <= wr_data_lo[0];

              30: for(i=0; i<1; i++) if(wr_mask_lo[i]) soft_reset <= wr_data_lo[0];
              
              1024: for(i=0; i<4; i++) if(wr_mask_lo[i]) axi_wr_data_l[i*8+:8] <= wr_data_lo[i*8+:8];
              default:;
            endcase
         end

         if(wr_en_hi) begin
            case(wr_addr_hi[`CFG_ADDR_BITS-1:3])
              1: for(i=0; i<4; i++) if(wr_mask_hi[i]) tx_dsc_mask_l[32+i*8+:8]   <= wr_data_hi[i*8+:8];
              5: for(i=0; i<4; i++) if(wr_mask_hi[i]) tx_pkt_mask_l[32+i*8+:8]   <= wr_data_hi[i*8+:8];
              7: for(i=0; i<4; i++) if(wr_mask_hi[i]) tx_dne_mask_l[32+i*8+:8]   <= wr_data_hi[i*8+:8];
              9: for(i=0; i<4; i++) if(wr_mask_hi[i]) rx_dsc_mask_l[32+i*8+:8]   <= wr_data_hi[i*8+:8];
              13: for(i=0; i<4; i++) if(wr_mask_hi[i]) rx_pkt_mask_l[32+i*8+:8]   <= wr_data_hi[i*8+:8];
              15: for(i=0; i<4; i++) if(wr_mask_hi[i]) rx_dne_mask_l[32+i*8+:8]   <= wr_data_hi[i*8+:8];
              16: for(i=0; i<4; i++) if(wr_mask_hi[i]) host_tx_dne_offset_l[32+i*8+:8] <= wr_data_hi[i*8+:8];
              17: for(i=0; i<4; i++) if(wr_mask_hi[i]) host_tx_dne_mask_l[32+i*8+:8]   <= wr_data_hi[i*8+:8];
              18: for(i=0; i<4; i++) if(wr_mask_hi[i]) host_rx_dne_offset_l[32+i*8+:8] <= wr_data_hi[i*8+:8];
              19: for(i=0; i<4; i++) if(wr_mask_hi[i]) host_rx_dne_mask_l[32+i*8+:8]   <= wr_data_hi[i*8+:8];

              1024: for(i=0; i<4; i++) if(wr_mask_hi[i]) axi_wr_addr_l[i*8+:8] <= wr_data_hi[i*8+:8];
              1025: for(i=0; i<4; i++) if(wr_mask_hi[i]) axi_rd_addr_l[i*8+:8] <= wr_data_hi[i*8+:8];
              default:;
            endcase
         end
      end
   end

   // -------------------------------------------
   // -- write side effect
   // -------------------------------------------
   logic [31:0] axi_rd_data_ll;
   logic        axi_fifo_empty, axi_fifo_empty_l;
   logic        axi_fifo_almost_full;
   logic        axi_fifo_full;
   logic        axi_select;
   logic        axi_reply_empty;
   logic        axi_error_l;
   
   always_ff @(posedge pcie_clk) begin
      axi_wr_go_l <= wr_en_hi && (wr_addr_hi[`CFG_ADDR_BITS-1:3] == 1024) && wr_mask_hi[3] && wr_mem_valid;
      axi_rd_go_l <= wr_en_hi && (wr_addr_hi[`CFG_ADDR_BITS-1:3] == 1025) && wr_mask_hi[3] && wr_mem_valid;
   end

   fifo #(.WIDTH(64+1), .DEPTH(`CFG_X_AXI_DEPTH), .ALMOST_FULL(16)) 
   u_x_cfg_fifo_0 (.enq_en(axi_wr_go_l | axi_rd_go_l),
                   .enq_data((axi_wr_go_l) ? 
                             ({axi_wr_addr_l[31:0], axi_wr_data_l[31:0], 1'b0}) : 
                             ({axi_rd_addr_l[31:0],               32'b0, 1'b1})),
                   .deq_en(axi_rd_done | axi_wr_done),
                   .deq_data({axi_rdwr_addr[31:0], axi_wr_data[31:0], axi_select}),
                   .empty(axi_fifo_empty),
                   .almost_full(axi_fifo_almost_full),
                   .full(axi_fifo_full),
                   .enq_clk(pcie_clk),
                   .deq_clk(axi_clk),
                   .rst(rst));

   assign axi_wr_go = ~axi_fifo_empty & ~axi_select;
   assign axi_rd_go = ~axi_fifo_empty &  axi_select;

   fifo #(.WIDTH(32), .DEPTH(3)) 
   u_x_cfg_fifo_1 (.enq_en(axi_rd_done),
                   .enq_data(axi_rd_data),
                   .deq_en(~axi_reply_empty),
                   .deq_data(axi_rd_data_ll),
                   .empty(axi_reply_empty),
                   .almost_full(),
                   .full(),
                   .enq_clk(axi_clk),
                   .deq_clk(pcie_clk),
                   .rst(rst));

   x_signal u_x_cfg_empty (axi_clk, axi_fifo_empty, pcie_clk, axi_fifo_empty_l);
   x_flag   u_x_axi_error (axi_clk, axi_error, pcie_clk, axi_error_l);
   
   always_ff @(posedge pcie_clk) begin
      if(rst) begin
         mem_cfg_rd_valid <= 1;
         axi_rd_data_l    <= 32'hcafebabe;
      end
      else begin
         if(axi_rd_go_l) begin
            mem_cfg_rd_valid <= 0;
            axi_rd_data_l    <= 32'hdeadbeef;
         end
         else if(~axi_reply_empty) begin
            mem_cfg_rd_valid <= 1;
            axi_rd_data_l    <= axi_rd_data_ll;
         end
         else if(axi_error_l) begin
            mem_cfg_rd_valid <= 1;
            axi_rd_data_l    <= 32'hdeadbeef;
         end
      end
   end
      
   // -----------------------
   // -- read logic
   // -----------------------
   always_ff @(posedge pcie_clk) begin
      if(rd_mem_valid) begin
         if(rd_en_lo) begin
            case(rd_addr_lo[`CFG_ADDR_BITS-1:3])
              1: rd_data_lo <= tx_dsc_mask_l[0+:32];
              5: rd_data_lo <= tx_pkt_mask_l[0+:32];
              7: rd_data_lo <= tx_dne_mask_l[0+:32];
              9: rd_data_lo <= rx_dsc_mask_l[0+:32];
              13: rd_data_lo <= rx_pkt_mask_l[0+:32];
              15: rd_data_lo <= rx_dne_mask_l[0+:32];
              16: rd_data_lo <= host_tx_dne_offset_l[0+:32];
              17: rd_data_lo <= host_tx_dne_mask_l[0+:32];
              18: rd_data_lo <= host_rx_dne_offset_l[0+:32];
              19: rd_data_lo <= host_rx_dne_mask_l[0+:32];

              24: rd_data_lo <= {16'b0, rx_byte_wait_l};
              25: rd_data_lo <= {31'b0, tx_int_enable_l};
              26: rd_data_lo <= {31'b0, rx_int_enable_l};

              30: rd_data_lo <= {31'b0, soft_reset};

              1024: rd_data_lo <= axi_wr_data_l[0+:32];
              1025: rd_data_lo <= axi_rd_data_l[0+:32];
              1026: rd_data_lo <= {29'b0, axi_fifo_full, axi_fifo_almost_full, axi_fifo_empty_l};
              
              default: rd_data_lo <= 32'h0;
            endcase
         end

         if(rd_en_hi) begin
            case(rd_addr_hi[`CFG_ADDR_BITS-1:3])
              1: rd_data_hi <= tx_dsc_mask_l[32+:32];
              5: rd_data_hi <= tx_pkt_mask_l[32+:32];
              7: rd_data_hi <= tx_dne_mask_l[32+:32];
              9: rd_data_hi <= rx_dsc_mask_l[32+:32];
              13: rd_data_hi <= rx_pkt_mask_l[32+:32];
              15: rd_data_hi <= rx_dne_mask_l[32+:32];
              16: rd_data_hi <= host_tx_dne_offset_l[32+:32];
              17: rd_data_hi <= host_tx_dne_mask_l[32+:32];
              18: rd_data_hi <= host_rx_dne_offset_l[32+:32];
              19: rd_data_hi <= host_rx_dne_mask_l[32+:32];

              1024: rd_data_hi <= axi_wr_addr_l[0+:32];
              1025: rd_data_hi <= axi_rd_addr_l[0+:32];
              
              default: rd_data_hi <= 32'h0;
            endcase
         end
      end
   end

endmodule
