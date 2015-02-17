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

`default_nettype none

module encodeInOneCycle #( 
    parameter C_INPORT_WIDTH = 3,
    parameter C_OUTPORT_WIDTH = 8,
    parameter C_PACKET_LENGTH_WIDTH = 14,
    parameter C_AXIS_DATA_WIDTH = 256
)
(
    input wire axi_aclk,
    input wire axi_resetn,    
    
    // Master Stream Ports
    output wire [C_AXIS_DATA_WIDTH-1:0] m_axis_tdata,
    output wire [log2(C_AXIS_DATA_WIDTH/8)-1:0] m_axis_tkeep_enc, 
    output wire [C_PACKET_LENGTH_WIDTH-1:0] m_axis_tuser_packet_length,
    output wire [C_INPORT_WIDTH-1:0] m_axis_tuser_in_port,
    output wire [C_OUTPORT_WIDTH-1:0] m_axis_tuser_out_port,
    output wire [C_INPORT_WIDTH-1:0] m_axis_tuser_in_vport,
    output wire [C_OUTPORT_WIDTH-1:0] m_axis_tuser_out_vport,
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
  `include "../../../../nf10_upb_lib/hdl/verilog/tkeep_coder.v"
  
  reg [C_AXIS_DATA_WIDTH-1:0] axis_tdata = 0;
  reg [(C_AXIS_DATA_WIDTH/8)-1:0] axis_tkeep_enc = 0;
  reg [C_PACKET_LENGTH_WIDTH-1:0] axis_tuser_packet_length = 0;
  reg [C_INPORT_WIDTH-1:0] axis_tuser_in_port = 0;
  reg [C_OUTPORT_WIDTH-1:0] axis_tuser_out_port = 0;
  reg [C_INPORT_WIDTH-1:0] axis_tuser_in_vport = 0;
  reg [C_OUTPORT_WIDTH-1:0] axis_tuser_out_vport = 0;
  reg axis_tvalid = 0;
  reg axis_tlast = 0;
  
  always @(posedge axi_aclk) begin
  	if (!axis_tvalid || m_axis_tready) begin
  		axis_tdata <= s_axis_tdata;
  		axis_tkeep_enc <= encode(s_axis_tkeep);
  		axis_tuser_packet_length <= s_axis_tuser_packet_length;
  		axis_tuser_in_port <= s_axis_tuser_in_port;
  		axis_tuser_out_port <= s_axis_tuser_out_port;
  		axis_tuser_out_port <= s_axis_tuser_in_vport;
  		axis_tuser_out_vport <= s_axis_tuser_out_vport;
  		axis_tvalid <= s_axis_tvalid;
  		axis_tlast <= s_axis_tlast;
  	end
  end
  
  assign s_axis_tready = !axis_tvalid || m_axis_tready;
  
  assign m_axis_tdata = axis_tdata;
  assign m_axis_tkeep_enc = axis_tkeep_enc;
  assign m_axis_tuser_packet_length = axis_tuser_packet_length;
  assign m_axis_tuser_in_port = axis_tuser_in_port;
  assign m_axis_tuser_out_port = axis_tuser_out_port;
  assign m_axis_tuser_in_vport = axis_tuser_in_vport;
  assign m_axis_tuser_out_vport = axis_tuser_out_vport;
  assign m_axis_tvalid = axis_tvalid;
  assign m_axis_tlast = axis_tlast;
  
  
  function integer log2;
    input integer number;
    begin
      log2=0;
      while(2**log2<number) begin
        log2=log2+1;
      end
    end
  endfunction // log2
  
endmodule
