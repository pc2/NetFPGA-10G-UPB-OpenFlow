/*******************************************************************************
 *
 *  NetFPGA-10G http://www.netfpga.org
 *
 *  File:
 *        pcie_tx.v
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
 *        Top level PCIe transmit module that wraps read, write and completion
 *        transmit submodules.
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

module pcie_tx
  (
   // PCIe Transaction TX
   output logic [63:0]                trn_td,
   output logic [7:0]                 trn_trem_n,
   output logic                       trn_tsof_n,
   output logic                       trn_teof_n,
   output logic                       trn_tsrc_rdy_n,
   input logic                        trn_tdst_rdy_n,
   input logic [3:0]                  trn_tbuf_av,

   // PCIe Transaction interrupt
   output logic                       cfg_interrupt_n,
   input logic                        cfg_interrupt_rdy_n,
   output logic [7:0]                 cfg_interrupt_di,
   input logic [7:0]                  cfg_interrupt_do,
   input logic [2:0]                  cfg_interrupt_mmenable,
   input logic                        cfg_interrupt_msienable,

   // PCIe misc
   input logic [15:0]                 pcie_id,
   input logic                        bus_master_en,

   // memory reads
   output logic [1:0]                 rd_if_select,
   output logic [3:0]                 rd_mem_select,
   output logic [`MEM_ADDR_BITS-1:0]  rd_addr_hi,
   input logic [31:0]                 rd_data_hi,
   output logic                       rd_en_hi,
   output logic [`MEM_ADDR_BITS-1:0]  rd_addr_lo,
   input logic [31:0]                 rd_data_lo,
   output logic                       rd_en_lo,

   // write queue interfaces
   input logic [`NUM_PORTS-1:0]       wr_q_req_v,
   input logic [`PCIE_WR_Q_WIDTH-1:0] wr_q_req_data[`NUM_PORTS-1:0],
   output logic [`NUM_PORTS-1:0]      wr_q_req_grant,
   // read queue interfaces
   input logic [`NUM_PORTS-1:0]       rd_q_req_v,
   input logic [`PCIE_RD_Q_WIDTH-1:0] rd_q_req_data[`NUM_PORTS-1:0],
   output logic [`NUM_PORTS-1:0]      rd_q_req_grant,
   // completion queue interfaces
   input logic [`NUM_PORTS-1:0]       cm_q_req_v,
   input logic [`PCIE_CM_Q_WIDTH-1:0] cm_q_req_data[`NUM_PORTS-1:0],
   output logic [`NUM_PORTS-1:0]      cm_q_req_grant,
   
   // ORT request interface
   output logic                       ort_req_v,
   output logic [3:0]                 ort_req_tag,
   output logic [1:0]                 ort_req_iface,
   output logic [3:0]                 ort_req_mem,
   output logic [`MEM_ADDR_BITS-1:0]  ort_req_addr,
   input logic                        ort_next_tag_v,
   input logic [3:0]                  ort_next_tag,

`ifdef DEBUG_PCIE
   // debug
   output logic [63:0]                debug_t_h1[7:0],
   output logic [63:0]                debug_t_h2[7:0],
   output logic [63:0]                debug_t_ts[7:0],
`endif
   
   // AXI rd ready qualifier
   input logic                        mem_cfg_rd_valid,

   // stats
   output logic [63:0]                stat_pcie_tx_ts,
   output logic [31:0]                stat_pcie_tx_word_cnt,
   output logic [31:0]                stat_pcie_tx_wr_cnt,
   output logic [31:0]                stat_pcie_tx_rd_cnt,
   output logic [31:0]                stat_pcie_tx_cm_cnt,
   output logic [31:0]                stat_pcie_tx_err_cnt,

   // misc
   input logic                        tx_clk,
   input logic                        rx_clk,
   input logic                        pcie_clk,
   input logic                        rst
   );

   // flop the reset to help timing
   logic                             rst_reg, rst_d1, rst_d2;
   always_ff @(posedge pcie_clk) begin
      rst_d1  <= rst;
      rst_d2  <= rst_d1;
      rst_reg <= rst_d1 | rst_d2;
   end
      
   // -----------------------------------
   // -- pipeline registers
   // -----------------------------------
   logic [63:0]                 trn_td_wr;
   logic [7:0]                  trn_trem_n_wr;
   logic                        trn_tsof_n_wr;
   logic                        trn_teof_n_wr;
   logic                        trn_tsrc_rdy_n_wr;
   logic                        cfg_interrupt_n_wr;
   logic [7:0]                  cfg_interrupt_di_wr; 

   logic [63:0]                 trn_td_rd;
   logic [7:0]                  trn_trem_n_rd;
   logic                        trn_tsof_n_rd;
   logic                        trn_teof_n_rd;
   logic                        trn_tsrc_rdy_n_rd;

   logic [63:0]                 trn_td_cm;
   logic [7:0]                  trn_trem_n_cm;
   logic                        trn_tsof_n_cm;
   logic                        trn_teof_n_cm;
   logic                        trn_tsrc_rdy_n_cm;
   
   logic [1:0]                  rd_if_select_wr;
   logic [3:0]                  rd_mem_select_wr;
   logic [`MEM_ADDR_BITS-1:0]   rd_addr_hi_wr;
   logic                        rd_en_hi_wr;
   logic [`MEM_ADDR_BITS-1:0]   rd_addr_lo_wr;
   logic                        rd_en_lo_wr;

   logic [1:0]                  rd_if_select_cm;
   logic [3:0]                  rd_mem_select_cm;
   logic [`MEM_ADDR_BITS-1:0]   rd_addr_hi_cm;
   logic                        rd_en_hi_cm;
   logic [`MEM_ADDR_BITS-1:0]   rd_addr_lo_cm;
   logic                        rd_en_lo_cm;

   logic                        pcie_req_v_wr;
   logic                        pcie_req_grant_wr;
   logic                        pcie_req_done_wr;
   logic [2:0]                  ppl_ctrl_wr;

   logic                        pcie_req_v_rd;
   logic                        pcie_req_grant_rd;
   logic                        pcie_req_done_rd;
   
   logic                        pcie_req_v_cm;
   logic                        pcie_req_grant_cm;
   logic                        pcie_req_done_cm;
   logic [2:0]                  ppl_ctrl_cm;

   logic [63:0]                 trn_td_ppl1;
   logic [7:0]                  trn_trem_n_ppl1;
   logic                        trn_tsof_n_ppl1;
   logic                        trn_teof_n_ppl1;
   logic                        trn_tsrc_rdy_n_ppl1;
   logic                        cfg_interrupt_n_ppl1;
   logic [7:0]                  cfg_interrupt_di_ppl1;

   logic [63:0]                 trn_td_ppl2;
   logic [7:0]                  trn_trem_n_ppl2;
   logic                        trn_tsof_n_ppl2;
   logic                        trn_teof_n_ppl2;
   logic                        trn_tsrc_rdy_n_ppl2;
   logic                        cfg_interrupt_n_ppl2;
   logic [7:0]                  cfg_interrupt_di_ppl2;
   logic [31:0]                 rd_data_hi_ppl2;
   logic [31:0]                 rd_data_lo_ppl2;

   logic [2:0]                  ppl_ctrl_ppl1; // (0-tt, 1-tL, 2-tH, 3-HL, 4-LH)
   logic [2:0]                  ppl_ctrl_ppl2;

   logic                        ppl_stall, ppl_stall_d1, ppl_stall_d2;

   // -----------------------------------
   // -- PCIe transmit logic
   // -----------------------------------
   // round robin that selects between {wr,rd,cm}

   pcie_tx_wr u_pcie_tx_wr
     (.trn_td(trn_td_wr),
      .trn_trem_n(trn_trem_n_wr),
      .trn_tsof_n(trn_tsof_n_wr),
      .trn_teof_n(trn_teof_n_wr),
      .trn_tsrc_rdy_n(trn_tsrc_rdy_n_wr),
      .cfg_interrupt_n(cfg_interrupt_n_wr),
      .cfg_interrupt_di(cfg_interrupt_di_wr),
      .cfg_interrupt_do(cfg_interrupt_do),
      .cfg_interrupt_mmenable(cfg_interrupt_mmenable),
      .cfg_interrupt_msienable(cfg_interrupt_msienable),
      .pcie_id(pcie_id),
      .pcie_req_v(pcie_req_v_wr),
      .pcie_req_grant(pcie_req_grant_wr),
      .pcie_req_stall(ppl_stall_d1),
      .pcie_req_done(pcie_req_done_wr),
      .ppl_ctrl(ppl_ctrl_wr),
      .rd_if_select(rd_if_select_wr),
      .rd_mem_select(rd_mem_select_wr),
      .rd_addr_hi(rd_addr_hi_wr),
      .rd_en_hi(rd_en_hi_wr),
      .rd_addr_lo(rd_addr_lo_wr),
      .rd_en_lo(rd_en_lo_wr),
      .wr_q_req_v(wr_q_req_v),
      .wr_q_req_data(wr_q_req_data),
      .wr_q_req_grant(wr_q_req_grant),
      .rx_clk(rx_clk),
      .pcie_clk(pcie_clk),
      .rst(rst_reg)
      );
   
   pcie_tx_rd u_pcie_tx_rd
     (.trn_td(trn_td_rd),
      .trn_trem_n(trn_trem_n_rd),
      .trn_tsof_n(trn_tsof_n_rd),
      .trn_teof_n(trn_teof_n_rd),
      .trn_tsrc_rdy_n(trn_tsrc_rdy_n_rd),      
      .pcie_id(pcie_id),
      .pcie_req_v(pcie_req_v_rd),
      .pcie_req_grant(pcie_req_grant_rd),
      .pcie_req_stall(ppl_stall_d1),
      .pcie_req_done(pcie_req_done_rd),
      .rd_q_req_v(rd_q_req_v),
      .rd_q_req_data(rd_q_req_data),
      .rd_q_req_grant(rd_q_req_grant),
      .ort_req_v(ort_req_v),
      .ort_req_tag(ort_req_tag),
      .ort_req_iface(ort_req_iface),
      .ort_req_mem(ort_req_mem),
      .ort_req_addr(ort_req_addr),
      .ort_next_tag_v(ort_next_tag_v),
      .ort_next_tag(ort_next_tag),
      .tx_clk(tx_clk),
      .pcie_clk(pcie_clk),
      .rst(rst_reg)
      );

   pcie_tx_cm u_pcie_tx_cm
     (.trn_td(trn_td_cm),
      .trn_trem_n(trn_trem_n_cm),
      .trn_tsof_n(trn_tsof_n_cm),
      .trn_teof_n(trn_teof_n_cm),
      .trn_tsrc_rdy_n(trn_tsrc_rdy_n_cm),
      .pcie_id(pcie_id),
      .pcie_req_v(pcie_req_v_cm),
      .pcie_req_grant(pcie_req_grant_cm),
      .pcie_req_stall(ppl_stall_d1),
      .pcie_req_done(pcie_req_done_cm),
      .ppl_ctrl(ppl_ctrl_cm),
      .rd_if_select(rd_if_select_cm),
      .rd_mem_select(rd_mem_select_cm),
      .rd_addr_hi(rd_addr_hi_cm),
      .rd_en_hi(rd_en_hi_cm),
      .rd_addr_lo(rd_addr_lo_cm),
      .rd_en_lo(rd_en_lo_cm),
      .cm_q_req_v(cm_q_req_v),
      .cm_q_req_data(cm_q_req_data),
      .cm_q_req_grant(cm_q_req_grant),
      .mem_cfg_rd_valid(mem_cfg_rd_valid),
      .rx_clk(rx_clk),
      .pcie_clk(pcie_clk),
      .rst(rst_reg)
      );
      
   localparam STATE_IDLE = 0;
   localparam STATE_WAIT = 1;
   localparam STATE_WAIT2 = 2;
   localparam STATE_WAIT3 = 3;

   logic [1:0]                pcie_tx_priority,  pcie_tx_priority_nxt;
   logic [1:0]                state,             state_nxt;

   logic [63:0]               time_stamp;
   logic [3:0]                trn_tbuf_av_reg;

   logic                      pcie_req_grant_wr_nxt;
   logic                      pcie_req_grant_rd_nxt;
   logic                      pcie_req_grant_cm_nxt;   
   
`ifdef DEBUG_PCIE
   // debug
   logic [2:0]                debug_ptr, debug_ptr_nxt;
   logic [63:0]               debug_h1_nxt[7:0];
   logic [63:0]               debug_h2_nxt[7:0];
   logic [63:0]               debug_ts_nxt[7:0];
`endif
   
   // stats
   logic                      stat_pcie_tx_word_cnt_inc;
   logic                      stat_pcie_tx_wr_cnt_inc;
   logic                      stat_pcie_tx_rd_cnt_inc;
   logic                      stat_pcie_tx_cm_cnt_inc;
   logic                      stat_pcie_tx_err_cnt_inc;

   // address generation pipeline stage
   always_comb begin
      pcie_tx_priority_nxt = pcie_tx_priority;
      state_nxt = state;

      pcie_req_grant_wr_nxt = pcie_req_grant_wr;
      pcie_req_grant_rd_nxt = pcie_req_grant_rd;
      pcie_req_grant_cm_nxt = pcie_req_grant_cm;

`ifdef DEBUG_PCIE
      debug_h1_nxt  = debug_t_h1;
      debug_h2_nxt  = debug_t_h2;
      debug_ts_nxt  = debug_t_ts;
      debug_ptr_nxt = debug_ptr;

      if(~trn_tsrc_rdy_n_ppl1 & ~trn_tsof_n_ppl1) begin
         if(pcie_req_grant_wr) begin
            debug_h1_nxt[debug_ptr] = trn_td_ppl1;
            debug_h2_nxt[debug_ptr] = trn_td_wr;
            debug_ts_nxt[debug_ptr] = time_stamp;
            debug_ptr_nxt = debug_ptr + 1;
         end
         else if(pcie_req_grant_rd) begin
            debug_h1_nxt[debug_ptr] = trn_td_ppl1;
            debug_h2_nxt[debug_ptr] = trn_td_rd;
            debug_ts_nxt[debug_ptr] = time_stamp;
            debug_ptr_nxt = debug_ptr + 1;
         end
         else if(pcie_req_grant_cm && (rd_mem_select != `ID_MEM_STAT)) begin
            debug_h1_nxt[debug_ptr] = trn_td_ppl1;
            debug_h2_nxt[debug_ptr] = trn_td_cm;
            debug_ts_nxt[debug_ptr] = time_stamp;
            debug_ptr_nxt = debug_ptr + 1;
         end
      end
`endif
      
      // stats
      stat_pcie_tx_word_cnt_inc = ~trn_tsrc_rdy_n_wr | ~trn_tsrc_rdy_n_rd | ~trn_tsrc_rdy_n_cm;
      stat_pcie_tx_wr_cnt_inc   = pcie_req_done_wr;
      stat_pcie_tx_rd_cnt_inc   = pcie_req_done_rd;
      stat_pcie_tx_cm_cnt_inc   = pcie_req_done_cm;
      stat_pcie_tx_err_cnt_inc  = 0;

      case(state)
        STATE_IDLE: begin                
           // ----------------------------
           // *** WRITE (or INTERRUPT) ***
           // ----------------------------
           if( (((pcie_tx_priority == 0) && (pcie_req_v_wr)) ||
                ((pcie_tx_priority == 1) && (pcie_req_v_wr && !pcie_req_v_rd)) ||
                ((pcie_tx_priority == 2) && (pcie_req_v_wr && !pcie_req_v_cm && !pcie_req_v_rd))) &&
               trn_tbuf_av_reg[1] && bus_master_en) begin
           
              state_nxt = STATE_WAIT;
              pcie_req_grant_wr_nxt = 1;

              pcie_tx_priority_nxt = 2;
           end
           
           // ------------ 
           // *** READ *** 
           // ------------ 
           else if( (((pcie_tx_priority == 1) && (pcie_req_v_rd)) ||
                     ((pcie_tx_priority == 2) && (pcie_req_v_rd && !pcie_req_v_cm)) ||
                     ((pcie_tx_priority == 0) && (pcie_req_v_rd && !pcie_req_v_wr && !pcie_req_v_cm))) &&
                    trn_tbuf_av_reg[0] && bus_master_en) begin
              

              state_nxt = STATE_WAIT;
              pcie_req_grant_rd_nxt = 1;
              
              pcie_tx_priority_nxt = 0;
           end
           
           // ------------------ 
           // *** COMPLETION *** 
           // ------------------
           else if( (((pcie_tx_priority == 2) && (pcie_req_v_cm)) ||
                     ((pcie_tx_priority == 0) && (pcie_req_v_cm && !pcie_req_v_wr)) ||
                     ((pcie_tx_priority == 1) && (pcie_req_v_cm && !pcie_req_v_rd && !pcie_req_v_wr))) &&
                    trn_tbuf_av_reg[2]) begin

              state_nxt = STATE_WAIT;
              pcie_req_grant_cm_nxt = 1;
              
              pcie_tx_priority_nxt = 1;
           end

        end

        STATE_WAIT: begin
           if(pcie_req_done_wr | pcie_req_done_rd | pcie_req_done_cm) begin
              pcie_req_grant_wr_nxt = 0;
              pcie_req_grant_rd_nxt = 0;
              pcie_req_grant_cm_nxt = 0;
              state_nxt = STATE_WAIT2;
           end           
        end

		// add delay cycles that ids for outstanding requests are always unique
        STATE_WAIT2: begin
           state_nxt = STATE_WAIT3;
        end

		STATE_WAIT3: begin
           state_nxt = STATE_IDLE;
        end

      endcase

   end

   logic [31:0]                 rd_data_hi_reg;
   logic [31:0]                 rd_data_lo_reg;
   logic [63:0]                 trn_td_reg;
   logic [7:0]                  trn_trem_n_reg;
   logic                        trn_tsof_n_reg;
   logic                        trn_teof_n_reg;
   logic                        trn_tsrc_rdy_n_reg;
   logic                        cfg_interrupt_n_reg;
   logic [7:0]                  cfg_interrupt_di_reg;

   assign ppl_stall = (~cfg_interrupt_n & cfg_interrupt_rdy_n) ? 1 : trn_tdst_rdy_n;
   
   // --- PCIe pipeline stage
   always_ff @(posedge pcie_clk) begin

      if(rst_reg) begin
         cfg_interrupt_n     <= 1;
         trn_tsrc_rdy_n      <= 1;
         cfg_interrupt_n_reg <= 1;
         trn_tsrc_rdy_n_reg  <= 1;
      end
      else if(~ppl_stall & ppl_stall_d1) begin // first cycle after stall
         cfg_interrupt_n <= cfg_interrupt_n_reg;
         trn_tsrc_rdy_n  <= trn_tsrc_rdy_n_reg;
      end
      else if(~ppl_stall) begin // no stall
         cfg_interrupt_n <= cfg_interrupt_n_ppl2;
         trn_tsrc_rdy_n  <= trn_tsrc_rdy_n_ppl2;
      end
      else if(~ppl_stall_d1) begin // first cycle of stall (store next data)
         cfg_interrupt_n_reg <= cfg_interrupt_n_ppl2;
         trn_tsrc_rdy_n_reg  <= trn_tsrc_rdy_n_ppl2;
      end
      
      if(~ppl_stall & ppl_stall_d1) begin // first cycle after stall
         trn_tsof_n <= trn_tsof_n_reg;
         trn_teof_n <= trn_teof_n_reg;
         trn_trem_n <= trn_trem_n_reg;

         cfg_interrupt_di <= cfg_interrupt_di_reg;

         trn_td <= trn_td_reg;         
      end
      else if(~ppl_stall) begin // no stall
         trn_tsof_n <= trn_tsof_n_ppl2;
         trn_teof_n <= trn_teof_n_ppl2;
         trn_trem_n <= trn_trem_n_ppl2;

         cfg_interrupt_di <= cfg_interrupt_di_ppl2;

         case(ppl_ctrl_ppl2)
           3'h1: begin
              trn_td[63:32] <= trn_td_ppl2[63:32];
              trn_td[31:0]  <= {rd_data_lo_ppl2[7:0],rd_data_lo_ppl2[15:8],rd_data_lo_ppl2[23:16],rd_data_lo_ppl2[31:24]};
           end
           3'h2: begin
              trn_td[63:32] <= trn_td_ppl2[63:32];
              trn_td[31:0]  <= {rd_data_hi_ppl2[7:0],rd_data_hi_ppl2[15:8],rd_data_hi_ppl2[23:16],rd_data_hi_ppl2[31:24]};
           end
           3'h3: begin
              trn_td[63:32] <= {rd_data_hi_ppl2[7:0],rd_data_hi_ppl2[15:8],rd_data_hi_ppl2[23:16],rd_data_hi_ppl2[31:24]};
              trn_td[31:0]  <= {rd_data_lo_ppl2[7:0],rd_data_lo_ppl2[15:8],rd_data_lo_ppl2[23:16],rd_data_lo_ppl2[31:24]};
           end
           3'h4: begin
              trn_td[63:32] <= {rd_data_lo_ppl2[7:0],rd_data_lo_ppl2[15:8],rd_data_lo_ppl2[23:16],rd_data_lo_ppl2[31:24]};
              trn_td[31:0]  <= {rd_data_hi_ppl2[7:0],rd_data_hi_ppl2[15:8],rd_data_hi_ppl2[23:16],rd_data_hi_ppl2[31:24]};
           end
           default: trn_td <= trn_td_ppl2;
         endcase
      end
      else if(~ppl_stall_d1) begin // first cycle of stall (store next data)
         trn_tsof_n_reg <= trn_tsof_n_ppl2;
         trn_teof_n_reg <= trn_teof_n_ppl2;
         trn_trem_n_reg <= trn_trem_n_ppl2;

         cfg_interrupt_di_reg <= cfg_interrupt_di_ppl2;

         case(ppl_ctrl_ppl2)
           3'h1: begin
              trn_td_reg[63:32] <= trn_td_ppl2[63:32];
              trn_td_reg[31:0]  <= {rd_data_lo_ppl2[7:0],rd_data_lo_ppl2[15:8],rd_data_lo_ppl2[23:16],rd_data_lo_ppl2[31:24]};
           end
           3'h2: begin
              trn_td_reg[63:32] <= trn_td_ppl2[63:32];
              trn_td_reg[31:0]  <= {rd_data_hi_ppl2[7:0],rd_data_hi_ppl2[15:8],rd_data_hi_ppl2[23:16],rd_data_hi_ppl2[31:24]};
           end
           3'h3: begin
              trn_td_reg[63:32] <= {rd_data_hi_ppl2[7:0],rd_data_hi_ppl2[15:8],rd_data_hi_ppl2[23:16],rd_data_hi_ppl2[31:24]};
              trn_td_reg[31:0]  <= {rd_data_lo_ppl2[7:0],rd_data_lo_ppl2[15:8],rd_data_lo_ppl2[23:16],rd_data_lo_ppl2[31:24]};
           end
           3'h4: begin
              trn_td_reg[63:32] <= {rd_data_lo_ppl2[7:0],rd_data_lo_ppl2[15:8],rd_data_lo_ppl2[23:16],rd_data_lo_ppl2[31:24]};
              trn_td_reg[31:0]  <= {rd_data_hi_ppl2[7:0],rd_data_hi_ppl2[15:8],rd_data_hi_ppl2[23:16],rd_data_hi_ppl2[31:24]};
           end
           default: trn_td_reg <= trn_td_ppl2;
         endcase
      end
      
   end

   always_ff @(posedge pcie_clk) begin
      integer i;
      if(rst_reg) begin
         pcie_tx_priority  <= 0;
         state             <= STATE_IDLE;

         pcie_req_grant_wr <= 0;
         pcie_req_grant_rd <= 0;
         pcie_req_grant_cm <= 0;

`ifdef DEBUG_PCIE
         // debug
         for(i=0; i<16; i++) begin
            debug_t_h1[i]  <= 0;
            debug_t_h2[i]  <= 0;
            debug_t_ts[i]  <= 0;
         end
         debug_ptr   <= 0;
`endif
         // stats
         stat_pcie_tx_ts       <= 0;
         stat_pcie_tx_word_cnt <= 0;
         stat_pcie_tx_wr_cnt   <= 0;
         stat_pcie_tx_rd_cnt   <= 0;
         stat_pcie_tx_cm_cnt   <= 0;
         stat_pcie_tx_err_cnt  <= 0;

         time_stamp  <= 0;

         trn_tsrc_rdy_n_ppl1   <= 1;
         cfg_interrupt_n_ppl1  <= 1;
         trn_tsrc_rdy_n_ppl2   <= 1;
         cfg_interrupt_n_ppl2  <= 1;
         
      end
      else begin
         pcie_tx_priority  <= pcie_tx_priority_nxt;
         state             <= state_nxt;

         pcie_req_grant_wr <= pcie_req_grant_wr_nxt;
         pcie_req_grant_rd <= pcie_req_grant_rd_nxt;
         pcie_req_grant_cm <= pcie_req_grant_cm_nxt;

`ifdef DEBUG_PCIE
         // debug
         debug_t_h1 <= debug_h1_nxt;
         debug_t_h2 <= debug_h2_nxt;
         debug_t_ts <= debug_ts_nxt;
         debug_ptr  <= debug_ptr_nxt;
`endif
         // stats
         if(stat_pcie_tx_word_cnt_inc) stat_pcie_tx_word_cnt <= stat_pcie_tx_word_cnt + 1;
         if(stat_pcie_tx_wr_cnt_inc)   stat_pcie_tx_wr_cnt   <= stat_pcie_tx_wr_cnt + 1;
         if(stat_pcie_tx_rd_cnt_inc)   stat_pcie_tx_rd_cnt   <= stat_pcie_tx_rd_cnt + 1;
         if(stat_pcie_tx_cm_cnt_inc)   stat_pcie_tx_cm_cnt   <= stat_pcie_tx_cm_cnt + 1;
         if(stat_pcie_tx_err_cnt_inc)  stat_pcie_tx_err_cnt  <= stat_pcie_tx_err_cnt + 1;
         if(stat_pcie_tx_word_cnt_inc | stat_pcie_tx_wr_cnt_inc | stat_pcie_tx_rd_cnt_inc |
            stat_pcie_tx_cm_cnt_inc   | stat_pcie_tx_err_cnt_inc) stat_pcie_tx_ts <= time_stamp;
         
         time_stamp <= time_stamp + 1;

         if(pcie_req_grant_wr & ~ppl_stall_d1)
           trn_tsrc_rdy_n_ppl1 <= trn_tsrc_rdy_n_wr;
         else if(pcie_req_grant_rd & ~ppl_stall_d1)
           trn_tsrc_rdy_n_ppl1 <= trn_tsrc_rdy_n_rd;
         else if(pcie_req_grant_cm & ~ppl_stall_d1)
           trn_tsrc_rdy_n_ppl1 <= trn_tsrc_rdy_n_cm;
         else if(~ppl_stall_d1)
           trn_tsrc_rdy_n_ppl1 <= 1;

         if(~ppl_stall_d1) begin
            trn_tsrc_rdy_n_ppl2   <= trn_tsrc_rdy_n_ppl1;
            cfg_interrupt_n_ppl1  <= cfg_interrupt_n_wr;
            cfg_interrupt_n_ppl2  <= cfg_interrupt_n_ppl1;
         end
      end
      
      trn_tbuf_av_reg   <= trn_tbuf_av;
      
      if(pcie_req_grant_wr) begin
         rd_if_select  <= rd_if_select_wr;
         rd_mem_select <= rd_mem_select_wr;
         rd_addr_hi    <= rd_addr_hi_wr;
         rd_en_hi      <= rd_en_hi_wr;
         rd_addr_lo    <= rd_addr_lo_wr;
         rd_en_lo      <= rd_en_lo_wr;

         if(~ppl_stall_d1) begin
            trn_td_ppl1     <= trn_td_wr;
            trn_trem_n_ppl1 <= trn_trem_n_wr;
            trn_tsof_n_ppl1 <= trn_tsof_n_wr;
            trn_teof_n_ppl1 <= trn_teof_n_wr;
            ppl_ctrl_ppl1   <= ppl_ctrl_wr;
         end
      end
      else if(pcie_req_grant_rd) begin
         if(~ppl_stall_d1) begin
            trn_td_ppl1     <= trn_td_rd;
            trn_trem_n_ppl1 <= trn_trem_n_rd;
            trn_tsof_n_ppl1 <= trn_tsof_n_rd;
            trn_teof_n_ppl1 <= trn_teof_n_rd;
            ppl_ctrl_ppl1   <= 0;
         end
      end
      else if(pcie_req_grant_cm) begin
         rd_if_select  <= rd_if_select_cm;
         rd_mem_select <= rd_mem_select_cm;
         rd_addr_hi    <= rd_addr_hi_cm;
         rd_en_hi      <= rd_en_hi_cm;
         rd_addr_lo    <= rd_addr_lo_cm;
         rd_en_lo      <= rd_en_lo_cm;

         if(~ppl_stall_d1) begin
            trn_td_ppl1     <= trn_td_cm;
            trn_trem_n_ppl1 <= trn_trem_n_cm;
            trn_tsof_n_ppl1 <= trn_tsof_n_cm;
            trn_teof_n_ppl1 <= trn_teof_n_cm;
            ppl_ctrl_ppl1   <= ppl_ctrl_cm;
         end
      end
      else begin
         rd_en_hi      <= 0;
         rd_en_lo      <= 0;

         if(~ppl_stall_d1) begin
            trn_td_ppl1     <= 0;
            trn_trem_n_ppl1 <= 1;
            trn_tsof_n_ppl1 <= 1;
            trn_teof_n_ppl1 <= 1;
            ppl_ctrl_ppl1 <= 0;
         end
      end
      cfg_interrupt_di_ppl1 <= cfg_interrupt_di_wr;
         
      // pipeline registers     
      if(~ppl_stall_d1 & ppl_stall_d2) begin
         rd_data_lo_ppl2 <= rd_data_lo_reg;
         rd_data_hi_ppl2 <= rd_data_hi_reg;
      end     
      else if(~ppl_stall_d1) begin
         rd_data_lo_ppl2 <= rd_data_lo;
         rd_data_hi_ppl2 <= rd_data_hi;
      end
      else if(~ppl_stall_d2) begin
         rd_data_lo_reg <= rd_data_lo;
         rd_data_hi_reg <= rd_data_hi;
      end
      
      if(~ppl_stall_d1) begin
         trn_td_ppl2           <= trn_td_ppl1;
         trn_trem_n_ppl2       <= trn_trem_n_ppl1;
         trn_tsof_n_ppl2       <= trn_tsof_n_ppl1;
         trn_teof_n_ppl2       <= trn_teof_n_ppl1;
         cfg_interrupt_di_ppl2 <= cfg_interrupt_di_ppl1;

         ppl_ctrl_ppl2 <= ppl_ctrl_ppl1;
      end

      ppl_stall_d1 <= ppl_stall;
      ppl_stall_d2 <= ppl_stall_d1;
      
   end
endmodule

