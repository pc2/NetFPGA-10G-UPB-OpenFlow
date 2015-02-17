/*******************************************************************************
 *
 *  NetFPGA-10G http://www.netfpga.org
 *
 *  File:
 *        axi.v
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
 *        AXI lite master interface module. This module converts the interface
 *        between the Xilinx AXI lite master module and the custom interface
 *        used in this DMA module implementation.
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

module axi
   (
    // AXI lite master core interface
    output logic        IP2Bus_MstRd_Req,
    output logic        IP2Bus_MstWr_Req,
    output logic [31:0] IP2Bus_Mst_Addr,
    output logic [3:0]  IP2Bus_Mst_BE,
    output logic        IP2Bus_Mst_Lock,
    output logic        IP2Bus_Mst_Reset,
    input logic         Bus2IP_Mst_CmdAck,
    input logic         Bus2IP_Mst_Cmplt,
    input logic         Bus2IP_Mst_Error,
    input logic         Bus2IP_Mst_Rearbitrate,
    input logic         Bus2IP_Mst_Timeout,
    input logic [31:0]  Bus2IP_MstRd_d,
    input logic         Bus2IP_MstRd_src_rdy_n,
    output logic [31:0] IP2Bus_MstWr_d,
    input logic         Bus2IP_MstWr_dst_rdy_n,

    // config space interface
    input logic [31:0]  axi_rdwr_addr,
    output logic [31:0] axi_rd_data,
    input logic [31:0]  axi_wr_data,
    input logic         axi_rd_go,
    input logic         axi_wr_go,
    output logic        axi_rd_done,
    output logic        axi_wr_done,
    output logic        axi_error,

    // misc
    input logic         axi_clk,
    input logic         rst
    );
   
   logic [27:0]         wdt_counter, wdt_counter_nxt;

   assign IP2Bus_Mst_Reset = rst | Bus2IP_Mst_Error | (&wdt_counter); //reset
   assign IP2Bus_Mst_Lock = 0;    // unused
   assign IP2Bus_Mst_BE = 4'hf;   // support only 32bit writes

   assign IP2Bus_Mst_Addr = axi_rdwr_addr;
   assign IP2Bus_MstWr_d  = axi_wr_data;
   assign axi_rd_data     = Bus2IP_MstRd_d;
   assign axi_wr_done     = Bus2IP_Mst_Cmplt;
   assign axi_rd_done     = Bus2IP_Mst_Cmplt & ~Bus2IP_MstRd_src_rdy_n;
   assign axi_error       = Bus2IP_Mst_Error | (&wdt_counter);

   localparam STATE_IDLE        = 0;
   localparam STATE_WR_ACK_WAIT = 1;
   localparam STATE_WR_DNE_WAIT = 2;
   localparam STATE_RD_ACK_WAIT = 3;
   localparam STATE_RD_DNE_WAIT = 4;

   logic [2:0]          state, state_nxt;
   
   always_comb begin
      state_nxt = state;
      wdt_counter_nxt = wdt_counter + 1;
      
      IP2Bus_MstRd_Req = 0;
      IP2Bus_MstWr_Req = 0;
      
      case(state)
        STATE_IDLE: begin
           wdt_counter_nxt = 0;
           if(axi_rd_go) begin
              IP2Bus_MstRd_Req = 1;
              state_nxt = STATE_RD_ACK_WAIT;
           end
           else if(axi_wr_go) begin
              IP2Bus_MstWr_Req = 1;
              state_nxt = STATE_WR_ACK_WAIT;
           end
        end
        STATE_WR_ACK_WAIT: begin
           IP2Bus_MstWr_Req = 1;
           if(Bus2IP_Mst_CmdAck) state_nxt = STATE_WR_DNE_WAIT;
        end
        STATE_WR_DNE_WAIT: begin
           if(axi_wr_done) state_nxt = STATE_IDLE;
        end
        STATE_RD_ACK_WAIT: begin
           IP2Bus_MstRd_Req = 1; 
           if(Bus2IP_Mst_CmdAck) state_nxt = STATE_RD_DNE_WAIT;
        end
        STATE_RD_DNE_WAIT: begin
           if(axi_rd_done) state_nxt = STATE_IDLE;
        end
        default: begin
           state_nxt = STATE_IDLE;
        end
      endcase
   end
   always_ff @(posedge axi_clk) begin
      if(rst | Bus2IP_Mst_Error) begin
         state <= STATE_IDLE;
         wdt_counter <= 0;
      end
      else if(wdt_counter == 28'hfffffff) begin
         state <= STATE_IDLE;
         wdt_counter <= 0;
      end
      else begin
         state <= state_nxt;
         wdt_counter <= wdt_counter_nxt;
      end
   end
   
endmodule
