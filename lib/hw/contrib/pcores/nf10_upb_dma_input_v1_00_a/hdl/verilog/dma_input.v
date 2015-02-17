/*
 * Copyright (c) 2014, 2015 Michael Lass
 * bevan@bi-co.net
 *
 * This file is part of the NetFPGA 10G UPB OpenFlow Switch project:
 *
 * Project Group "On-the-Fly Networking for Big Data"
 * SFB 901 "On-The-Fly Computing"
 *
 * University of Paderborn
 * Computer Engineering Group
 * Pohlweg 47 - 49
 * 33098 Paderborn
 * Germany
 *
 * 
 * This file is free code: you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License version 2.1 as
 * published by the Free Software Foundation.
 *
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this project. If not, see <http://www.gnu.org/licenses/>.
 * 
 */

`default_nettype none

module nf10_upb_dma_input #(
    parameter C_PORT_NUMBER = 0
)(
   // Common inputs
    input CLK,
    input axi_resetn, // active low
   
   //// Arbiter
   // Master
    output [255:0] arbiter_m_axis_tdata,
    output [31:0] arbiter_m_axis_tkeep,
    output [2:0] arbiter_m_axis_tuser_in_port,
    output [2:0] arbiter_m_axis_tuser_in_vport,
    output [7:0] arbiter_m_axis_tuser_out_port,
    output [7:0] arbiter_m_axis_tuser_out_vport,
    output [13:0] arbiter_m_axis_tuser_packet_length,
    output arbiter_m_axis_tvalid,
    input arbiter_m_axis_tready,
    output arbiter_m_axis_tlast,   
   
   //// Output queue
   // Slave
    input [255:0] output_queue_s_axis_tdata,
    input [31:0] output_queue_s_axis_tkeep,
    input [2:0] output_queue_s_axis_tuser_in_port,
    input [2:0] output_queue_s_axis_tuser_in_vport,
    input [7:0] output_queue_s_axis_tuser_out_port,
    input [7:0] output_queue_s_axis_tuser_out_vport,
    input [13:0] output_queue_s_axis_tuser_packet_length,
    input output_queue_s_axis_tvalid,
    output output_queue_s_axis_tready,
    input output_queue_s_axis_tlast,
   
   //// DMA
   // Master
    output [63:0] dma_m_axis_tdata,
    output [7:0] dma_m_axis_tkeep,
    output [127:0] dma_m_axis_tuser,
    output dma_m_axis_tvalid,
    input dma_m_axis_tready,
    output dma_m_axis_tlast,
   // Slave
    input [63:0] dma_s_axis_tdata,
    input [7:0] dma_s_axis_tkeep,
    input [127:0] dma_s_axis_tuser,
    input dma_s_axis_tvalid,
    output dma_s_axis_tready,
    input dma_s_axis_tlast
);

   `include "../../../nf10_upb_lib/hdl/verilog/tkeep_coder.v"

   // Misc. regs and constants
   localparam [31:0] TKEEP_ALL = {(32){1'b1}};
   wire reset = ~axi_resetn;

   // Connections for PA FIFO
   wire pa_fifo_rden,
        pa_fifo_eop,
        pa_fifo_full,
        pa_fifo_empty,
        pa_fifo_wren,
        pa_fifo_commit;
   reg  pa_fifo_revert = 0; // we do not need revert functionality for the dma
   wire [21:0] pa_fifo_mi, // 5bit TKEEP encoded + 3bit tuser_in_port + 14bit packet size
               pa_fifo_mo;
   wire [255:0] pa_fifo_di,
                pa_fifo_do;

   // Connections for width divider and multiplier
   wire [63:0] width_multiplier_m_axis_tdata;
   wire [2:0]  width_multiplier_m_axis_tkeep;
   wire width_multiplier_m_axis_tvalid,
        width_multiplier_m_axis_tready,
        width_multiplier_m_axis_tlast;
   wire [255:0] width_multiplier_s_axis_tdata;
   wire [4:0]  width_multiplier_s_axis_tkeep;
   wire width_multiplier_s_axis_tvalid,
        width_multiplier_s_axis_tready,
        width_multiplier_s_axis_tlast;
   wire [255:0] width_divider_m_axis_tdata;
   wire [31:0]  width_divider_m_axis_tkeep;
   wire width_divider_m_axis_tvalid,
        width_divider_m_axis_tready,
        width_divider_m_axis_tlast;
   wire [63:0] width_divider_s_axis_tdata;
   wire [7:0]  width_divider_s_axis_tkeep;
   wire width_divider_s_axis_tvalid,
        width_divider_s_axis_tready,
        width_divider_s_axis_tlast;

   // Instantiate width divider and multiplier
   width_multiplier #(
      .INPUT_WIDTH(64),
      .MULTIPLY_VALUE(4)
   ) mult (
      .clk(CLK),
      .reset(reset),
      .m_axis_tdata(width_multiplier_s_axis_tdata),
      .m_axis_tkeep(width_multiplier_s_axis_tkeep),
      .m_axis_tvalid(width_multiplier_s_axis_tvalid),
      .m_axis_tready(width_multiplier_s_axis_tready),
      .m_axis_tlast(width_multiplier_s_axis_tlast),
      .s_axis_tdata(width_multiplier_m_axis_tdata),
      .s_axis_tkeep(width_multiplier_m_axis_tkeep),
      .s_axis_tvalid(width_multiplier_m_axis_tvalid),
      .s_axis_tready(width_multiplier_m_axis_tready),
      .s_axis_tlast(width_multiplier_m_axis_tlast)
   );
   width_divider #(
      .OUTPUT_WIDTH(64),
      .DIVISOR(4)
   ) div (
      .clk(CLK),
      .reset(reset),
      .m_axis_tdata(width_divider_s_axis_tdata),
      .m_axis_tkeep(width_divider_s_axis_tkeep),
      .m_axis_tvalid(width_divider_s_axis_tvalid),
      .m_axis_tready(width_divider_s_axis_tready),
      .m_axis_tlast(width_divider_s_axis_tlast),
      .s_axis_tdata(width_divider_m_axis_tdata),
      .s_axis_tkeep(width_divider_m_axis_tkeep),
      .s_axis_tvalid(width_divider_m_axis_tvalid),
      .s_axis_tready(width_divider_m_axis_tready),
      .s_axis_tlast(width_divider_m_axis_tlast)
   );


   // Instantiate the packet aware fifo
   nf10_upb_packet_fifo #(
      .DATA_WIDTH(256),
      .METADATA_WIDTH(22), // 5bit TKEEP encoded + 3bit tuser_in_port + 14 bit packet size
      .DATA_DEPTH(10),
      .METADATA_DEPTH(10),
      .LOW_THRESHOLD(256),
      .HIGH_THRESHOLD(768)
   ) pa_fifo (
      .CLK(CLK),
      .DI(pa_fifo_di),
      .MI(pa_fifo_mi),
      .RDEN(pa_fifo_rden),
      .WREN(pa_fifo_wren),
      .COMMIT(pa_fifo_commit),
      .REVERT(pa_fifo_revert),
      .RST(reset),
      .DO(pa_fifo_do),
      .MO(pa_fifo_mo),
      .EOP(pa_fifo_eop),
      .FULL(pa_fifo_full),
      .EMPTY(pa_fifo_empty),
      .RDERR(),
      .WRERR(),
      .BELOW_LOW(),
      .ABOVE_HIGH()
   );

   
   // Output side: Convert width from 256bit down to 64bit
   assign width_divider_m_axis_tdata = output_queue_s_axis_tdata;
   assign width_divider_m_axis_tkeep = output_queue_s_axis_tkeep;
   assign width_divider_m_axis_tvalid = output_queue_s_axis_tvalid;
   assign width_divider_m_axis_tlast = output_queue_s_axis_tlast;
   assign output_queue_s_axis_tready = width_divider_m_axis_tready;
   assign dma_m_axis_tdata = width_divider_s_axis_tdata;
   assign dma_m_axis_tkeep = width_divider_s_axis_tkeep;
   assign dma_m_axis_tuser = {96'b0, // ignored by DMA
                              2'b0, // waisted
                              output_queue_s_axis_tuser_in_port, // 3 bit in_port (binary)
                              output_queue_s_axis_tuser_in_vport, // 3 bit in_vport (binary)
                              output_queue_s_axis_tuser_out_vport, // 8 bit out_vport (one hot)
                              16'b0 // DMA will use this for packet length
                             };
   assign dma_m_axis_tvalid = width_divider_s_axis_tvalid;
   assign dma_m_axis_tlast = width_divider_s_axis_tlast;
   assign width_divider_s_axis_tready = dma_m_axis_tready;
   
   // Input side: Feed data into PA FIFO
   assign width_multiplier_m_axis_tdata = dma_s_axis_tdata;
   assign width_multiplier_m_axis_tkeep = encode(dma_s_axis_tkeep);
   assign dma_s_axis_tready = width_multiplier_m_axis_tready;
   assign width_multiplier_m_axis_tvalid = dma_s_axis_tvalid;
   assign width_multiplier_m_axis_tlast = dma_s_axis_tlast;
   assign pa_fifo_di = width_multiplier_s_axis_tdata;
   assign pa_fifo_mi = {dma_s_axis_tuser[18:16], // src port (3 bit)
                        dma_s_axis_tuser[13:0], // packet size (14 bit)
                        width_multiplier_s_axis_tkeep // tkeep encoded (5 bit)
                       };
   assign width_multiplier_s_axis_tready = !pa_fifo_full;
   assign pa_fifo_wren   = !pa_fifo_full && width_multiplier_s_axis_tvalid;
   assign pa_fifo_commit =  pa_fifo_wren && width_multiplier_s_axis_tlast;
   
   // Input side: Connect PA FIFO to Arbiter
   assign arbiter_m_axis_tdata =               pa_fifo_do;
   assign arbiter_m_axis_tuser_in_port =       C_PORT_NUMBER;
   assign arbiter_m_axis_tuser_in_vport =      pa_fifo_mo[21:19];
   assign arbiter_m_axis_tuser_out_port =      8'b0; // user logic will have to decide...
   assign arbiter_m_axis_tuser_out_vport =     8'b0; //  -- " --
   assign arbiter_m_axis_tuser_packet_length = pa_fifo_mo[18:5];
   assign arbiter_m_axis_tvalid =              (!pa_fifo_empty);
   assign arbiter_m_axis_tlast =               pa_fifo_eop;
   assign pa_fifo_rden =                       (!pa_fifo_empty && arbiter_m_axis_tready);
   assign arbiter_m_axis_tkeep = (pa_fifo_eop) ? decode(pa_fifo_mo[4:0])
                                               : TKEEP_ALL;
endmodule
