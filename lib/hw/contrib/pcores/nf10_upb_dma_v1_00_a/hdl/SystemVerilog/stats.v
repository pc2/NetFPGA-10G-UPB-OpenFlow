/*******************************************************************************
 *
 *  NetFPGA-10G http://www.netfpga.org
 *
 *  File:
 *        stats.v
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
 *        Statistics module for the DMA engine. This module contains 
 *        statistics registers that are read through the PCIe interface
 *        by the host. Also, this module resolves all the clock domain crossins
 *        for those registers.
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

module stats
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

`ifdef DEBUG_PCIE   
   // debug (pcie_clk)
   input logic [63:0]               debug_t_h1[15:0],
   input logic [63:0]               debug_t_h2[15:0],
   input logic [63:0]               debug_t_ts[15:0],
   input logic [63:0]               debug_r_h1[15:0],
   input logic [63:0]               debug_r_h2[15:0],
   input logic [63:0]               debug_r_ts[15:0],
`endif
   
   // pcie_rx (pcie_clk)
   input logic [63:0]               stat_pcie_rx_ts,
   input logic [31:0]               stat_pcie_rx_word_cnt,
   input logic [31:0]               stat_pcie_rx_wr_cnt,
   input logic [31:0]               stat_pcie_rx_rd_cnt,
   input logic [31:0]               stat_pcie_rx_cm_cnt,
   input logic [31:0]               stat_pcie_rx_err_cnt,

   // pcie_tx (pcie_clk)
   input logic [63:0]               stat_pcie_tx_ts,
   input logic [31:0]               stat_pcie_tx_word_cnt,
   input logic [31:0]               stat_pcie_tx_wr_cnt,
   input logic [31:0]               stat_pcie_tx_rd_cnt,
   input logic [31:0]               stat_pcie_tx_cm_cnt,
   input logic [31:0]               stat_pcie_tx_err_cnt,

   // mac tx (tx_clk)
   input logic [63:0]               stat_mac_tx_ts,
   input logic [31:0]               stat_mac_tx_word_cnt,
   input logic [31:0]               stat_mac_tx_pkt_cnt,
   
   // mac rx (rx_clk)
   input logic [63:0]               stat_mac_rx_ts,
   input logic [31:0]               stat_mac_rx_word_cnt,
   input logic [31:0]               stat_mac_rx_pkt_cnt,
   input logic [31:0]               stat_mac_rx_err_cnt,

   // misc
   input logic                      tx_clk,
   input logic                      rx_clk,
   input logic                      pcie_clk,
   input logic                      rst
   );

   // --------------------------
   // -- clock domain crossing
   // --------------------------
   logic [63:0]                     stat_mac_tx_ts_l;
   logic [31:0]                     stat_mac_tx_word_cnt_l;
   logic [31:0]                     stat_mac_tx_pkt_cnt_l;
   logic [63:0]                     stat_mac_rx_ts_l;
   logic [31:0]                     stat_mac_rx_word_cnt_l;
   logic [31:0]                     stat_mac_rx_pkt_cnt_l;
   logic [31:0]                     stat_mac_rx_err_cnt_l;
   
   x_signal #(64) x_stat_0(tx_clk, stat_mac_tx_ts,       pcie_clk, stat_mac_tx_ts_l);
   x_signal #(32) x_stat_1(tx_clk, stat_mac_tx_word_cnt, pcie_clk, stat_mac_tx_word_cnt_l);
   x_signal #(32) x_stat_2(tx_clk, stat_mac_tx_pkt_cnt,  pcie_clk, stat_mac_tx_pkt_cnt_l);
   x_signal #(64) x_stat_3(rx_clk, stat_mac_rx_ts,       pcie_clk, stat_mac_rx_ts_l);
   x_signal #(32) x_stat_4(rx_clk, stat_mac_rx_word_cnt, pcie_clk, stat_mac_rx_word_cnt_l);
   x_signal #(32) x_stat_5(rx_clk, stat_mac_rx_pkt_cnt,  pcie_clk, stat_mac_rx_pkt_cnt_l);
   x_signal #(32) x_stat_6(rx_clk, stat_mac_rx_err_cnt,  pcie_clk, stat_mac_rx_err_cnt_l);
   
   // -----------------------
   // -- read logic
   // -----------------------
   always_ff @(posedge pcie_clk) begin
      if(rd_mem_valid) begin
         if(rd_en_lo) begin
            // use lower 1k for pcie debug
            if(rd_addr_lo[(`STATS_ADDR_BITS-2) +: 2] == 2'b00) begin
               case(rd_addr_lo[`STATS_ADDR_BITS-3:7])
`ifdef DEBUG_PCIE
                 0: rd_data_lo <= debug_t_h1[rd_addr_lo[6:3]][0+:32];
                 1: rd_data_lo <= debug_t_h2[rd_addr_lo[6:3]][0+:32];
                 2: rd_data_lo <= debug_t_ts[rd_addr_lo[6:3]][0+:32];
                 3: rd_data_lo <= debug_r_h1[rd_addr_lo[6:3]][0+:32];
                 4: rd_data_lo <= debug_r_h2[rd_addr_lo[6:3]][0+:32];
                 5: rd_data_lo <= debug_r_ts[rd_addr_lo[6:3]][0+:32];              
`endif
                 default: rd_data_lo <= 32'hcafebabe;
               endcase
            end
            else begin
               case(rd_addr_lo[`STATS_ADDR_BITS-3:3])
                 0: rd_data_lo <= stat_pcie_rx_ts[0+:32];
                 1: rd_data_lo <= stat_pcie_rx_word_cnt[0+:32];
                 2: rd_data_lo <= stat_pcie_rx_wr_cnt[0+:32];
                 3: rd_data_lo <= stat_pcie_rx_rd_cnt[0+:32];
                 4: rd_data_lo <= stat_pcie_rx_cm_cnt[0+:32];
                 5: rd_data_lo <= stat_pcie_rx_err_cnt[0+:32];

                 7: rd_data_lo <= stat_pcie_tx_ts[0+:32];
                 8: rd_data_lo <= stat_pcie_tx_word_cnt[0+:32];
                 9: rd_data_lo <= stat_pcie_tx_wr_cnt[0+:32];
                 10: rd_data_lo <= stat_pcie_tx_rd_cnt[0+:32];
                 11: rd_data_lo <= stat_pcie_tx_cm_cnt[0+:32];
                 12: rd_data_lo <= stat_pcie_tx_err_cnt[0+:32];
                 
                 16: rd_data_lo <= stat_mac_tx_ts_l[0+:32];
                 17: rd_data_lo <= stat_mac_tx_word_cnt_l[0+:32];
                 18: rd_data_lo <= stat_mac_tx_pkt_cnt_l[0+:32];

                 20: rd_data_lo <= stat_mac_rx_ts_l[0+:32];
                 21: rd_data_lo <= stat_mac_rx_word_cnt_l[0+:32];
                 22: rd_data_lo <= stat_mac_rx_pkt_cnt_l[0+:32];
                 23: rd_data_lo <= stat_mac_rx_err_cnt_l[0+:32];
                 default: rd_data_lo <= 32'hcafebabe;
               endcase
            end
         end

         if(rd_en_hi) begin
            // use lower 1k for pcie debug
            if(rd_addr_hi[(`STATS_ADDR_BITS-2) +: 2] == 2'b00) begin
               case(rd_addr_hi[`STATS_ADDR_BITS-3:7])
`ifdef DEBUG_PCIE
                 0: rd_data_hi <= debug_t_h1[rd_addr_hi[6:3]][32+:32];
                 1: rd_data_hi <= debug_t_h2[rd_addr_hi[6:3]][32+:32];
                 2: rd_data_hi <= debug_t_ts[rd_addr_hi[6:3]][32+:32];
                 3: rd_data_hi <= debug_r_h1[rd_addr_hi[6:3]][32+:32];
                 4: rd_data_hi <= debug_r_h2[rd_addr_hi[6:3]][32+:32];
                 5: rd_data_hi <= debug_r_ts[rd_addr_hi[6:3]][32+:32];              
`endif
                 default: rd_data_hi <= 32'h0;
               endcase
            end
            else begin
               case(rd_addr_hi[`STATS_ADDR_BITS-3:3])
                 0: rd_data_hi <= stat_pcie_rx_ts[32+:32];

                 7: rd_data_hi <= stat_pcie_tx_ts[32+:32];

                 16: rd_data_hi <= stat_mac_tx_ts_l[32+:32];

                 20: rd_data_hi <= stat_mac_rx_ts_l[32+:32];
                 default: rd_data_hi <= 32'h0;
               endcase
            end
         end
      end
   end
endmodule
