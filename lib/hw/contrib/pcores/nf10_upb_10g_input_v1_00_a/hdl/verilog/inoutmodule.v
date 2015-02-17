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
module inoutmodule #( 
    parameter C_INPORT_WIDTH = 3,
    parameter C_OUTPORT_WIDTH = 8,
    parameter C_PACKET_LENGTH_WIDTH = 14,
    parameter C_AXIS_DATA_WIDTH = 256
)
(
    input clk156,
    input axi_aclk,
    input axi_resetn,    
    
    // Master Stream Ports
    output [63:0] mac_m_axis_tdata,
    output [7:0] mac_m_axis_tkeep, 
    output [0:0] mac_m_axis_tuser,
    output mac_m_axis_tvalid,
    input  mac_m_axis_tready,
    output mac_m_axis_tlast,

    // Slave Stream Ports
    input [63:0] mac_s_axis_tdata,
    input [7:0] mac_s_axis_tkeep,
    input [0:0] mac_s_axis_tuser,
    input mac_s_axis_tvalid,
    output mac_s_axis_tready,
    input mac_s_axis_tlast
    );
    
    wire [C_AXIS_DATA_WIDTH-1:0] m_axis_tdata;
    wire [(C_AXIS_DATA_WIDTH/8)-1:0] m_axis_tkeep;
    wire [C_PACKET_LENGTH_WIDTH-1:0] m_axis_tuser_packet_length;
    wire [C_INPORT_WIDTH-1:0] m_axis_tuser_in_port;
    wire [C_OUTPORT_WIDTH-1:0] m_axis_tuser_out_port;
    wire [C_INPORT_WIDTH-1:0] m_axis_tuser_in_vport;
    wire [C_OUTPORT_WIDTH-1:0] m_axis_tuser_out_vport;
    wire m_axis_tvalid;
    wire  m_axis_tready;
    wire m_axis_tlast;
    
    wire [C_AXIS_DATA_WIDTH-1:0] s_axis_tdata;
    wire [(C_AXIS_DATA_WIDTH/8)-1:0] s_axis_tkeep;
    wire [C_PACKET_LENGTH_WIDTH-1:0] s_axis_tuser_packet_length;
    wire [C_INPORT_WIDTH-1:0] s_axis_tuser_in_port;
    wire [C_OUTPORT_WIDTH-1:0] s_axis_tuser_out_port;
    wire [C_INPORT_WIDTH-1:0] s_axis_tuser_in_vport;
    wire [C_OUTPORT_WIDTH-1:0] s_axis_tuser_out_vport;
    wire s_axis_tvalid;
    wire s_axis_tready;
    wire s_axis_tlast;
    
    wire output_queue_clk;

nf10_upb_10g_input input_module (
    .clk156(clk156), 
    .mac_m_axis_tdata(mac_m_axis_tdata), 
    .mac_m_axis_tkeep(mac_m_axis_tkeep), 
    .mac_m_axis_tuser(mac_m_axis_tuser), 
    .mac_m_axis_tvalid(mac_m_axis_tvalid), 
    .mac_m_axis_tready(mac_m_axis_tready), 
    .mac_m_axis_tlast(mac_m_axis_tlast), 
    .mac_s_axis_tdata(mac_s_axis_tdata), 
    .mac_s_axis_tkeep(mac_s_axis_tkeep), 
    .mac_s_axis_tuser(mac_s_axis_tuser), 
    .mac_s_axis_tvalid(mac_s_axis_tvalid), 
    .mac_s_axis_tready(mac_s_axis_tready), 
    .mac_s_axis_tlast(mac_s_axis_tlast), 
    .axi_aclk(axi_aclk), 
    .axi_resetn(axi_resetn), 
    .arbiter_m_axis_tdata(s_axis_tdata), 
    .arbiter_m_axis_tkeep(s_axis_tkeep), 
    .arbiter_m_axis_tuser_packet_length(s_axis_tuser_packet_length), 
    .arbiter_m_axis_tuser_in_port(s_axis_tuser_in_port), 
    .arbiter_m_axis_tuser_out_port(s_axis_tuser_out_port),
    .arbiter_m_axis_tuser_in_vport(s_axis_tuser_in_vport), 
    .arbiter_m_axis_tuser_out_vport(s_axis_tuser_out_vport),  
    .arbiter_m_axis_tvalid(s_axis_tvalid), 
    .arbiter_m_axis_tready(s_axis_tready), 
    .arbiter_m_axis_tlast(s_axis_tlast), 
    .output_queue_clk(output_queue_clk), 
    .output_queue_s_axis_tdata(m_axis_tdata), 
    .output_queue_s_axis_tkeep(m_axis_tkeep), 
    .output_queue_s_axis_tuser_packet_length(m_axis_tuser_packet_length), 
    .output_queue_s_axis_tuser_in_port(m_axis_tuser_in_port), 
    .output_queue_s_axis_tuser_out_port(m_axis_tuser_out_port), 
    .output_queue_s_axis_tuser_in_vport(m_axis_tuser_in_vport), 
    .output_queue_s_axis_tuser_out_vport(m_axis_tuser_out_vport), 
    .output_queue_s_axis_tvalid(m_axis_tvalid), 
    .output_queue_s_axis_tready(m_axis_tready), 
    .output_queue_s_axis_tlast(m_axis_tlast)
    );

OutputBufferForTest buffer (
    .axi_aclk(axi_aclk), 
    .axi_resetn(axi_resetn), 
    .clk156(output_queue_clk), 
    .m_axis_tdata(m_axis_tdata), 
    .m_axis_tkeep(m_axis_tkeep), 
    .m_axis_tuser_packet_length(m_axis_tuser_packet_length), 
    .m_axis_tuser_in_port(m_axis_tuser_in_port), 
    .m_axis_tuser_out_port(m_axis_tuser_out_port),
    .m_axis_tuser_in_vport(m_axis_tuser_in_vport), 
    .m_axis_tuser_out_vport(m_axis_tuser_out_vport),  
    .m_axis_tvalid(m_axis_tvalid), 
    .m_axis_tready(m_axis_tready), 
    .m_axis_tlast(m_axis_tlast), 
    .s_axis_tdata(s_axis_tdata), 
    .s_axis_tkeep(s_axis_tkeep), 
    .s_axis_tuser_packet_length(s_axis_tuser_packet_length), 
    .s_axis_tuser_in_port(s_axis_tuser_in_port), 
    .s_axis_tuser_out_port(s_axis_tuser_out_port),
    .s_axis_tuser_in_vport(s_axis_tuser_in_vport), 
    .s_axis_tuser_out_vport(s_axis_tuser_out_vport),  
    .s_axis_tvalid(s_axis_tvalid), 
    .s_axis_tready(s_axis_tready), 
    .s_axis_tlast(s_axis_tlast)
    );
    
endmodule
