/*******************************************************************************
 *
 *  NetFPGA-10G http://www.netfpga.org
 *
 *  File:
 *        pcie_tx_q.v
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
 *        This is an arbiter module for multiple AXIS interface modules
 *        accessing PCIe transmit queues.
 *        NOTE: Because only one AXIS interface is used in this design, that
 *        functionality was stripped away to help with timing. This module
 *        is now only a wrapper around a fifo.
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

module pcie_tx_q 
  #(parameter WIDTH=32, 
    parameter DEPTH=32, 
    parameter NUM_PORTS=1, 
    parameter PORT_BITS=1
    )
   (
    input logic              req_v,
    input logic [WIDTH-1:0]  req_data,
    output logic             req_grant,
   
    input logic              deq_en,
    output logic [WIDTH-1:0] deq_data,
   
    output logic             empty,

    input logic              enq_clk,
    input logic              deq_clk,
    input logic              rst
   );


   logic                         full;
   logic                         enq_en;
   logic [WIDTH-1:0]             enq_data;


   // flop dequeue interface for timing
   logic [WIDTH-1:0]             deq_data_nxt;
   logic                         empty_nxt;

   always_ff @(posedge deq_clk) begin
      if(rst) begin
         empty <= 1;
         deq_data <= 0;
      end
      else begin
         deq_data <= deq_data_nxt;
         if(~deq_en)
           empty <= empty_nxt;
         else
           empty <= 1;
      end
   end
   
   always_comb begin
      req_grant = 0;

      enq_data = 0;
      enq_en   = 0;

      if(req_v & ~full) begin
         enq_data = req_data;
         enq_en = 1;
         req_grant = 1;
      end         

   end
   
   fifo #(.WIDTH(WIDTH), .DEPTH(DEPTH)) 
   u_q (
        .enq_en(enq_en),
        .enq_data(enq_data),
        .deq_en(deq_en),
        .deq_data(deq_data_nxt),
        .empty(empty_nxt),
        .almost_full(),
        .full(full),
        .enq_clk(enq_clk),
        .deq_clk(deq_clk),
        .rst(rst)
        );
   
endmodule