`timescale 1ns / 1ps
/*
 * Copyright (c) 2014, 2015 Thomas Löcke
 * tloecke@mail.uni-paderborn.de
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
module frame_gen_and_check #( 
    parameter C_PORT_NUMBER = 0,
    parameter C_INPORT_WIDTH = 3,
    parameter C_OUTPORT_WIDTH = 8,
    parameter C_PACKET_LENGTH_WIDTH = 14,
    parameter C_AXIS_DATA_WIDTH = 256 // multiple of 16
)
(
    output reg [7:0] error_count = 0,
    input wire s_axis_clk,
    
    // axi clock and reset //
    input wire axi_aclk,
    input wire axi_resetn,    
    
    // Master Stream Ports
    output wire [C_AXIS_DATA_WIDTH-1:0] m_axis_tdata,
    output wire [(C_AXIS_DATA_WIDTH/8)-1:0] m_axis_tkeep, 
    output wire [C_PACKET_LENGTH_WIDTH-1:0] m_axis_tuser_packet_length,
    output reg [C_INPORT_WIDTH-1:0] m_axis_tuser_in_port = C_PORT_NUMBER,
    output reg [C_OUTPORT_WIDTH-1:0] m_axis_tuser_out_port = 0,
    output reg [C_INPORT_WIDTH-1:0] m_axis_tuser_in_vport = 0,
    output reg [C_OUTPORT_WIDTH-1:0] m_axis_tuser_out_vport = 0,
    output wire m_axis_tvalid,
    input  wire m_axis_tready,
    output wire m_axis_tlast,
    
    // Slave Stream Ports
    input wire [C_AXIS_DATA_WIDTH-1:0] s_axis_tdata,
    input wire [(C_AXIS_DATA_WIDTH/8)-1:0] s_axis_tkeep,
    input wire [C_PACKET_LENGTH_WIDTH-1:0] s_axis_tuser_packet_length,
    input wire [C_INPORT_WIDTH-1:0] s_axis_tuser_in_port,
    input wire [C_OUTPORT_WIDTH-1:0] s_axis_tuser_out_port,
    input wire [C_INPORT_WIDTH-1:0] s_axis_tuser_in_vport,
    input wire [C_OUTPORT_WIDTH-1:0] s_axis_tuser_out_vport,
    input wire s_axis_tvalid,
    output wire s_axis_tready,
    input wire s_axis_tlast
    );
  //`include "../../../../nf10_upb_lib/hdl/verilog/tkeep_coder.v"
  `include "tkeep_coder.v"
  
  localparam m_seed = 16'hABCD;
  localparam s_seed = 16'hABCD;
  //localparam s_seed = 16'hD5E6;
     
  wire clk = axi_aclk;  
  wire reset = !axi_resetn;
  
  // master //
  
  reg [0:15] m_data_lfsr = m_seed;   
  wire [0:C_AXIS_DATA_WIDTH-1] m_data_lfsr_concat;
  assign m_axis_tdata = m_data_lfsr_concat;
  assign m_data_lfsr_concat = {C_AXIS_DATA_WIDTH/16{m_data_lfsr}};
  assign m_axis_tkeep = {C_AXIS_DATA_WIDTH/8{1'b1}};
  assign m_axis_tvalid = 1;
  assign m_axis_tlast = 1;
  assign m_axis_tuser_packet_length = 32;
  
  always @(posedge clk)
    if(reset)
      m_data_lfsr <= m_seed; 
    else if(m_axis_tready)
      m_data_lfsr <= {!{m_data_lfsr[3]^m_data_lfsr[12]^m_data_lfsr[14]^m_data_lfsr[15]}, m_data_lfsr[0:14]};
  
  // slave //  
  
  wire empty;
  wire [C_AXIS_DATA_WIDTH-1:0] fifo_do;
  wire [31:0] fifo_dop;
  wire fifo_last;
  wire [4:0] fifo_keep_enc;
  wire [C_AXIS_DATA_WIDTH/8-1:0] fifo_keep;
  assign {fifo_last, fifo_keep_enc} = fifo_dop[5:0];
  assign fifo_keep = decode(fifo_keep_enc);
  // instantiate an async fifo to change clock
  wide_fifo #(
   	.C_SIM_MODE("FAST"),
   	.C_ALMOST_FULL_OFFSET(9'hA), 
   	.C_ALMOST_EMPTY_OFFSET(9'hA),
   	.C_EN_ECC_READ("FALSE"),
   	.C_EN_ECC_WRITE("FALSE"),
   	.C_EN_SYN("FALSE"),
   	.C_FIRST_WORD_FALL_THROUGH("TRUE"),
    .C_NUMBER_FIFOS(4)
  ) wide_fifo (
    .ALMOSTEMPTY(), 
    .ALMOSTFULL(), 
    .DO(fifo_do), 
    .DOP(fifo_dop),
    .EMPTY(empty), 
    .FULL(), 
    .RDERR(), 
    .WRERR(), 
    .DI(s_axis_tdata), 
    .DIP({26'b0, s_axis_tlast, encode(s_axis_tkeep)}), 
    .RDCLK(clk), 
    .RDEN(!empty), 
    .RST(!axi_resetn), 
    .WRCLK(s_axis_clk),
    .WREN(s_axis_tvalid)
    );
  
  reg [0:15] s_data_lfsr = s_seed;  
  wire [0:C_AXIS_DATA_WIDTH-1] s_data_lfsr_concat;
  assign s_data_lfsr_concat = {C_AXIS_DATA_WIDTH/16{s_data_lfsr}};
  assign s_axis_tready = 1;
  
  always @(posedge clk)
    if(reset)
      s_data_lfsr <= s_seed; 
    else if(!empty)
      s_data_lfsr <= {!{s_data_lfsr[3]^s_data_lfsr[12]^s_data_lfsr[14]^s_data_lfsr[15]}, s_data_lfsr[0:14]};
          
  // error //
  
  wire error;
  assign error = !empty && (
    (| (s_data_lfsr_concat ^ fifo_do)) ||
    //s_data_lfsr != fifo_do ||
    !fifo_last ||
    !(& fifo_keep)
    //fifo_keep != {C_AXIS_DATA_WIDTH/8{1'b1}}
  );
  
  always @(posedge clk)
    if (error && error_count < {8{1'b1}})
      error_count <= error_count + 1;


endmodule
