/*
 * UPB demultiplexerfortest
 *
 * Used for advanced loopback project: Connects Port 0 <-> 3 and 1 <-> 2
 *
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

module demultiplexerfortest #( 
    parameter C_PORT_NUMBER = 0,
    parameter C_INPORT_WIDTH = 3,
    parameter C_OUTPORT_WIDTH = 8,
    parameter C_PACKET_LENGTH_WIDTH = 14,
    parameter C_AXIS_DATA_WIDTH = 256
)
(
    output wire [C_AXIS_DATA_WIDTH-1:0] port0_m_axis_tdata,
    output wire [(C_AXIS_DATA_WIDTH/8)-1:0] port0_m_axis_tkeep, 
    output wire [C_PACKET_LENGTH_WIDTH-1:0] port0_m_axis_tuser_packet_length,
    output wire [C_INPORT_WIDTH-1:0] port0_m_axis_tuser_in_port,
    output wire [C_OUTPORT_WIDTH-1:0] port0_m_axis_tuser_out_port,
    output wire [C_INPORT_WIDTH-1:0] port0_m_axis_tuser_in_vport,
    output wire [C_OUTPORT_WIDTH-1:0] port0_m_axis_tuser_out_vport,
    output wire port0_m_axis_tvalid,
    input  wire port0_m_axis_tready,
    output wire port0_m_axis_tlast,	 
	 
    output wire [C_AXIS_DATA_WIDTH-1:0] port1_m_axis_tdata,
    output wire [(C_AXIS_DATA_WIDTH/8)-1:0] port1_m_axis_tkeep, 
    output wire [C_PACKET_LENGTH_WIDTH-1:0] port1_m_axis_tuser_packet_length,
    output wire [C_INPORT_WIDTH-1:0] port1_m_axis_tuser_in_port,
    output wire [C_OUTPORT_WIDTH-1:0] port1_m_axis_tuser_out_port,
    output wire [C_INPORT_WIDTH-1:0] port1_m_axis_tuser_in_vport,
    output wire [C_OUTPORT_WIDTH-1:0] port1_m_axis_tuser_out_vport,
    output wire port1_m_axis_tvalid,
    input  wire port1_m_axis_tready,
    output wire port1_m_axis_tlast,	 
	 
    output wire [C_AXIS_DATA_WIDTH-1:0] port2_m_axis_tdata,
    output wire [(C_AXIS_DATA_WIDTH/8)-1:0] port2_m_axis_tkeep, 
    output wire [C_PACKET_LENGTH_WIDTH-1:0] port2_m_axis_tuser_packet_length,
    output wire [C_INPORT_WIDTH-1:0] port2_m_axis_tuser_in_port,
    output wire [C_OUTPORT_WIDTH-1:0] port2_m_axis_tuser_out_port,
    output wire [C_INPORT_WIDTH-1:0] port2_m_axis_tuser_in_vport,
    output wire [C_OUTPORT_WIDTH-1:0] port2_m_axis_tuser_out_vport,
    output wire port2_m_axis_tvalid,
    input  wire port2_m_axis_tready,
    output wire port2_m_axis_tlast,	 
	 
    output wire [C_AXIS_DATA_WIDTH-1:0] port3_m_axis_tdata,
    output wire [(C_AXIS_DATA_WIDTH/8)-1:0] port3_m_axis_tkeep, 
    output wire [C_PACKET_LENGTH_WIDTH-1:0] port3_m_axis_tuser_packet_length,
    output wire [C_INPORT_WIDTH-1:0] port3_m_axis_tuser_in_port,
    output wire [C_OUTPORT_WIDTH-1:0] port3_m_axis_tuser_out_port,
    output wire [C_INPORT_WIDTH-1:0] port3_m_axis_tuser_in_vport,
    output wire [C_OUTPORT_WIDTH-1:0] port3_m_axis_tuser_out_vport,
    output wire port3_m_axis_tvalid,
    input  wire port3_m_axis_tready,
    output wire port3_m_axis_tlast,
	 
	 
    input wire [C_AXIS_DATA_WIDTH-1:0] arbiter_s_axis_tdata,
    input wire [(C_AXIS_DATA_WIDTH/8)-1:0] arbiter_s_axis_tkeep,
    input wire [C_PACKET_LENGTH_WIDTH-1:0] arbiter_s_axis_tuser_packet_length,
    input wire [C_INPORT_WIDTH-1:0] arbiter_s_axis_tuser_in_port,
    input wire [C_OUTPORT_WIDTH-1:0] arbiter_s_axis_tuser_out_port,
    input wire [C_INPORT_WIDTH-1:0] arbiter_s_axis_tuser_in_vport,
    input wire [C_OUTPORT_WIDTH-1:0] arbiter_s_axis_tuser_out_vport,
    input wire arbiter_s_axis_tvalid,
    output wire arbiter_s_axis_tready,
    input wire arbiter_s_axis_tlast
    );

  assign port0_m_axis_tdata = arbiter_s_axis_tdata;
  assign port1_m_axis_tdata = arbiter_s_axis_tdata;
  assign port2_m_axis_tdata = arbiter_s_axis_tdata;
  assign port3_m_axis_tdata = arbiter_s_axis_tdata;
  
  assign port0_m_axis_tkeep = arbiter_s_axis_tkeep && (arbiter_s_axis_tuser_in_port == 4'b0011);
  assign port1_m_axis_tkeep = arbiter_s_axis_tkeep && (arbiter_s_axis_tuser_in_port == 4'b0010);
  assign port2_m_axis_tkeep = arbiter_s_axis_tkeep && (arbiter_s_axis_tuser_in_port == 4'b0001);
  assign port3_m_axis_tkeep = arbiter_s_axis_tkeep && (arbiter_s_axis_tuser_in_port == 4'b0000);
  
  assign port0_m_axis_tuser_packet_length = arbiter_s_axis_tuser_packet_length;
  assign port1_m_axis_tuser_packet_length = arbiter_s_axis_tuser_packet_length;
  assign port2_m_axis_tuser_packet_length = arbiter_s_axis_tuser_packet_length;
  assign port3_m_axis_tuser_packet_length = arbiter_s_axis_tuser_packet_length;
  
  assign port0_m_axis_tuser_in_port = arbiter_s_axis_tuser_in_port;
  assign port1_m_axis_tuser_in_port = arbiter_s_axis_tuser_in_port;
  assign port2_m_axis_tuser_in_port = arbiter_s_axis_tuser_in_port;
  assign port3_m_axis_tuser_in_port = arbiter_s_axis_tuser_in_port;
  
  assign port0_m_axis_tuser_out_port = 8'b00000001;
  assign port1_m_axis_tuser_out_port = 8'b00000010;
  assign port2_m_axis_tuser_out_port = 8'b00000100;
  assign port3_m_axis_tuser_out_port = 8'b00001000;
  
  assign port0_m_axis_tuser_in_vport = arbiter_s_axis_tuser_in_vport;
  assign port1_m_axis_tuser_in_vport = arbiter_s_axis_tuser_in_vport;
  assign port2_m_axis_tuser_in_vport = arbiter_s_axis_tuser_in_vport;
  assign port3_m_axis_tuser_in_vport = arbiter_s_axis_tuser_in_vport;
  
  assign port0_m_axis_tuser_out_vport = port3_m_axis_tuser_out_vport;
  assign port1_m_axis_tuser_out_vport = port3_m_axis_tuser_out_vport;
  assign port2_m_axis_tuser_out_vport = port3_m_axis_tuser_out_vport;
  assign port3_m_axis_tuser_out_vport = port3_m_axis_tuser_out_vport;
  
  assign port0_m_axis_tvalid = arbiter_s_axis_tvalid && (arbiter_s_axis_tuser_in_port == 4'b0011);
  assign port1_m_axis_tvalid = arbiter_s_axis_tvalid && (arbiter_s_axis_tuser_in_port == 4'b0010);
  assign port2_m_axis_tvalid = arbiter_s_axis_tvalid && (arbiter_s_axis_tuser_in_port == 4'b0001);
  assign port3_m_axis_tvalid = arbiter_s_axis_tvalid && (arbiter_s_axis_tuser_in_port == 4'b0000);
  
  assign arbiter_s_axis_tready = port0_m_axis_tready && (arbiter_s_axis_tuser_in_port == 4'b0011)
                              || port1_m_axis_tready && (arbiter_s_axis_tuser_in_port == 4'b0010)
                              || port2_m_axis_tready && (arbiter_s_axis_tuser_in_port == 4'b0001)
                              || port3_m_axis_tready && (arbiter_s_axis_tuser_in_port == 4'b0000);
  
  assign port0_m_axis_tlast = arbiter_s_axis_tlast && (arbiter_s_axis_tuser_in_port == 4'b0011);
  assign port1_m_axis_tlast = arbiter_s_axis_tlast && (arbiter_s_axis_tuser_in_port == 4'b0010);
  assign port2_m_axis_tlast = arbiter_s_axis_tlast && (arbiter_s_axis_tuser_in_port == 4'b0001);
  assign port3_m_axis_tlast = arbiter_s_axis_tlast && (arbiter_s_axis_tuser_in_port == 4'b0000);

endmodule
