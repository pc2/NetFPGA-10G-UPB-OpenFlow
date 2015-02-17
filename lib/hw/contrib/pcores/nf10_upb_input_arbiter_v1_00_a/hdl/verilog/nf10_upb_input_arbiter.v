/*
 * UPB Input Arbiter
 *
 * Copyright (c) 2014, 2015 Felix Wallaschek
 * felix@elektronenversand.de
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
 */
 
 module nf10_upb_input_arbiter

#(
    parameter C_NUM_INPUTS=5,
    parameter C_DATA_WIDTH=256,
    parameter C_PACKET_LENGTH_WIDTH=14,
    parameter C_TKEEP_WIDTH=32,
    parameter C_IN_PORT_WIDTH=3,
    parameter C_OUT_PORT_WIDTH=8,
    parameter C_TIMESLICE_WIDTH=9,
    parameter C_TIMESLICE_0=280,
    parameter C_TIMESLICE_1=280,
    parameter C_TIMESLICE_2=280,
    parameter C_TIMESLICE_3=280,
    parameter C_TIMESLICE_4=280,
    parameter C_TIMESLICE_5=280,
    parameter C_TIMESLICE_6=280,
    parameter C_TIMESLICE_7=280
)
(
    input clk,
    input reset,
    
    input [C_DATA_WIDTH-1:0] s_axis_tdata_0,
    input [C_TKEEP_WIDTH-1:0] s_axis_tkeep_0,
    input s_axis_tlast_0,
    input s_axis_tvalid_0,
    input [C_PACKET_LENGTH_WIDTH-1:0] s_axis_tuser_packet_length_0,
    input [C_IN_PORT_WIDTH-1:0] s_axis_tuser_in_port_0,
    input [C_OUT_PORT_WIDTH-1:0] s_axis_tuser_out_port_0,
    input [C_IN_PORT_WIDTH-1:0] s_axis_tuser_in_vport_0,
    input [C_OUT_PORT_WIDTH-1:0] s_axis_tuser_out_vport_0,
    output s_axis_tready_0,
    
    input [C_DATA_WIDTH-1:0] s_axis_tdata_1,
    input [C_TKEEP_WIDTH-1:0] s_axis_tkeep_1,
    input s_axis_tlast_1,
    input s_axis_tvalid_1,
    input [C_PACKET_LENGTH_WIDTH-1:0] s_axis_tuser_packet_length_1,
    input [C_IN_PORT_WIDTH-1:0] s_axis_tuser_in_port_1,
    input [C_OUT_PORT_WIDTH-1:0] s_axis_tuser_out_port_1,
    input [C_IN_PORT_WIDTH-1:0] s_axis_tuser_in_vport_1,
    input [C_OUT_PORT_WIDTH-1:0] s_axis_tuser_out_vport_1,
    output s_axis_tready_1,
    
    input [C_DATA_WIDTH-1:0] s_axis_tdata_2,
    input [C_TKEEP_WIDTH-1:0] s_axis_tkeep_2,
    input s_axis_tlast_2,
    input s_axis_tvalid_2,
    input [C_PACKET_LENGTH_WIDTH-1:0] s_axis_tuser_packet_length_2,
    input [C_IN_PORT_WIDTH-1:0] s_axis_tuser_in_port_2,
    input [C_OUT_PORT_WIDTH-1:0] s_axis_tuser_out_port_2,
    input [C_IN_PORT_WIDTH-1:0] s_axis_tuser_in_vport_2,
    input [C_OUT_PORT_WIDTH-1:0] s_axis_tuser_out_vport_2,
    output s_axis_tready_2,
    
    input [C_DATA_WIDTH-1:0] s_axis_tdata_3,
    input [C_TKEEP_WIDTH-1:0] s_axis_tkeep_3,
    input s_axis_tlast_3,
    input s_axis_tvalid_3,
    input [C_PACKET_LENGTH_WIDTH-1:0] s_axis_tuser_packet_length_3,
    input [C_IN_PORT_WIDTH-1:0] s_axis_tuser_in_port_3,
    input [C_OUT_PORT_WIDTH-1:0] s_axis_tuser_out_port_3,
    input [C_IN_PORT_WIDTH-1:0] s_axis_tuser_in_vport_3,
    input [C_OUT_PORT_WIDTH-1:0] s_axis_tuser_out_vport_3,
    output s_axis_tready_3,
    
    input [C_DATA_WIDTH-1:0] s_axis_tdata_4,
    input [C_TKEEP_WIDTH-1:0] s_axis_tkeep_4,
    input s_axis_tlast_4,
    input s_axis_tvalid_4,
    input [C_PACKET_LENGTH_WIDTH-1:0] s_axis_tuser_packet_length_4,
    input [C_IN_PORT_WIDTH-1:0] s_axis_tuser_in_port_4,
    input [C_OUT_PORT_WIDTH-1:0] s_axis_tuser_out_port_4,
    input [C_IN_PORT_WIDTH-1:0] s_axis_tuser_in_vport_4,
    input [C_OUT_PORT_WIDTH-1:0] s_axis_tuser_out_vport_4,
    output s_axis_tready_4,
    
    input [C_DATA_WIDTH-1:0] s_axis_tdata_5,
    input [C_TKEEP_WIDTH-1:0] s_axis_tkeep_5,
    input s_axis_tlast_5,
    input s_axis_tvalid_5,
    input [C_PACKET_LENGTH_WIDTH-1:0] s_axis_tuser_packet_length_5,
    input [C_IN_PORT_WIDTH-1:0] s_axis_tuser_in_port_5,
    input [C_OUT_PORT_WIDTH-1:0] s_axis_tuser_out_port_5,
    input [C_IN_PORT_WIDTH-1:0] s_axis_tuser_in_vport_5,
    input [C_OUT_PORT_WIDTH-1:0] s_axis_tuser_out_vport_5,
    output s_axis_tready_5,
    
    input [C_DATA_WIDTH-1:0] s_axis_tdata_6,
    input [C_TKEEP_WIDTH-1:0] s_axis_tkeep_6,
    input s_axis_tlast_6,
    input s_axis_tvalid_6,
    input [C_PACKET_LENGTH_WIDTH-1:0] s_axis_tuser_packet_length_6,
    input [C_IN_PORT_WIDTH-1:0] s_axis_tuser_in_port_6,
    input [C_OUT_PORT_WIDTH-1:0] s_axis_tuser_out_port_6,
    input [C_IN_PORT_WIDTH-1:0] s_axis_tuser_in_vport_6,
    input [C_OUT_PORT_WIDTH-1:0] s_axis_tuser_out_vport_6,
    output s_axis_tready_6,
    
    input [C_DATA_WIDTH-1:0] s_axis_tdata_7,
    input [C_TKEEP_WIDTH-1:0] s_axis_tkeep_7,
    input s_axis_tlast_7,
    input s_axis_tvalid_7,
    input [C_PACKET_LENGTH_WIDTH-1:0] s_axis_tuser_packet_length_7,
    input [C_IN_PORT_WIDTH-1:0] s_axis_tuser_in_port_7,
    input [C_OUT_PORT_WIDTH-1:0] s_axis_tuser_out_port_7,
    input [C_IN_PORT_WIDTH-1:0] s_axis_tuser_in_vport_7,
    input [C_OUT_PORT_WIDTH-1:0] s_axis_tuser_out_vport_7,
    output s_axis_tready_7,
    
    
    output [C_DATA_WIDTH-1:0] m_axis_tdata,
    output [C_TKEEP_WIDTH-1:0] m_axis_tkeep,
    output m_axis_tlast,
    output m_axis_tvalid,
    output [C_PACKET_LENGTH_WIDTH-1:0]m_axis_tuser_packet_length,
    output [C_IN_PORT_WIDTH-1:0]m_axis_tuser_in_port,
    output [C_OUT_PORT_WIDTH-1:0]m_axis_tuser_out_port,
    output [C_IN_PORT_WIDTH-1:0]m_axis_tuser_in_vport,
    output [C_OUT_PORT_WIDTH-1:0]m_axis_tuser_out_vport,
    input m_axis_tready
);
   reg [C_TIMESLICE_WIDTH-1:0] cfg_timeslices_0 = C_TIMESLICE_0;
   reg [C_TIMESLICE_WIDTH-1:0] cfg_timeslices_1 = C_TIMESLICE_1;
   reg [C_TIMESLICE_WIDTH-1:0] cfg_timeslices_2 = C_TIMESLICE_2;
   reg [C_TIMESLICE_WIDTH-1:0] cfg_timeslices_3 = C_TIMESLICE_3;
   reg [C_TIMESLICE_WIDTH-1:0] cfg_timeslices_4 = C_TIMESLICE_4;
   reg [C_TIMESLICE_WIDTH-1:0] cfg_timeslices_5 = C_TIMESLICE_5;
   reg [C_TIMESLICE_WIDTH-1:0] cfg_timeslices_6 = C_TIMESLICE_6;
   reg [C_TIMESLICE_WIDTH-1:0] cfg_timeslices_7 = C_TIMESLICE_7;
   
   wire [C_DATA_WIDTH*8-1:0] s_axis_tdata = {s_axis_tdata_7,s_axis_tdata_6,s_axis_tdata_5,s_axis_tdata_4,s_axis_tdata_3,s_axis_tdata_2,s_axis_tdata_1,s_axis_tdata_0};
   wire [C_TKEEP_WIDTH*8-1:0] s_axis_tkeep = {s_axis_tkeep_7,s_axis_tkeep_6,s_axis_tkeep_5,s_axis_tkeep_4,s_axis_tkeep_3,s_axis_tkeep_2,s_axis_tkeep_1,s_axis_tkeep_0};
   wire [7:0] s_axis_tlast = {s_axis_tlast_7,s_axis_tlast_6,s_axis_tlast_5,s_axis_tlast_4,s_axis_tlast_3,s_axis_tlast_2,s_axis_tlast_1,s_axis_tlast_0};
   wire [7:0] s_axis_tvalid = {s_axis_tvalid_7,s_axis_tvalid_6,s_axis_tvalid_5,s_axis_tvalid_4,s_axis_tvalid_3,s_axis_tvalid_2,s_axis_tvalid_1,s_axis_tvalid_0};
   wire [C_PACKET_LENGTH_WIDTH*8-1:0] s_axis_tuser_packet_length = {s_axis_tuser_packet_length_7,s_axis_tuser_packet_length_6,s_axis_tuser_packet_length_5,s_axis_tuser_packet_length_4,s_axis_tuser_packet_length_3,s_axis_tuser_packet_length_2,s_axis_tuser_packet_length_1,s_axis_tuser_packet_length_0};
   wire [C_IN_PORT_WIDTH*8-1:0] s_axis_tuser_in_port = {s_axis_tuser_in_port_7,s_axis_tuser_in_port_6,s_axis_tuser_in_port_5,s_axis_tuser_in_port_4,s_axis_tuser_in_port_3,s_axis_tuser_in_port_2,s_axis_tuser_in_port_1,s_axis_tuser_in_port_0};
   wire [C_OUT_PORT_WIDTH*8-1:0] s_axis_tuser_out_port = {s_axis_tuser_out_port_7,s_axis_tuser_out_port_6,s_axis_tuser_out_port_5,s_axis_tuser_out_port_4,s_axis_tuser_out_port_3,s_axis_tuser_out_port_2,s_axis_tuser_out_port_1,s_axis_tuser_out_port_0};
   wire [C_IN_PORT_WIDTH*8-1:0] s_axis_tuser_in_vport = {s_axis_tuser_in_vport_7,s_axis_tuser_in_vport_6,s_axis_tuser_in_vport_5,s_axis_tuser_in_vport_4,s_axis_tuser_in_vport_3,s_axis_tuser_in_vport_2,s_axis_tuser_in_vport_1,s_axis_tuser_in_vport_0};
   wire [C_OUT_PORT_WIDTH*8-1:0] s_axis_tuser_out_vport = {s_axis_tuser_out_vport_7,s_axis_tuser_out_vport_6,s_axis_tuser_out_vport_5,s_axis_tuser_out_vport_4,s_axis_tuser_out_vport_3,s_axis_tuser_out_vport_2,s_axis_tuser_out_vport_1,s_axis_tuser_out_vport_0};
   wire [C_TIMESLICE_WIDTH*8-1:0] cfg_timeslices = {cfg_timeslices_7,cfg_timeslices_6,cfg_timeslices_5,cfg_timeslices_4,cfg_timeslices_3,cfg_timeslices_2,cfg_timeslices_1,cfg_timeslices_0};
   wire [7:0] s_axis_tready;
   assign s_axis_tready_0 = s_axis_tready[0];
   assign s_axis_tready_1 = s_axis_tready[1];
   assign s_axis_tready_2 = s_axis_tready[2];
   assign s_axis_tready_3 = s_axis_tready[3];
   assign s_axis_tready_4 = s_axis_tready[4];
   assign s_axis_tready_5 = s_axis_tready[5];
   assign s_axis_tready_6 = s_axis_tready[6];
   assign s_axis_tready_7 = s_axis_tready[7];
   
   nf10_upb_input_arbiter_flex #(
		.C_DATA_WIDTH(C_DATA_WIDTH),
		.C_PACKET_LENGTH_WIDTH(C_PACKET_LENGTH_WIDTH),
		.C_TKEEP_WIDTH(C_TKEEP_WIDTH),
		.C_IN_PORT_WIDTH(C_IN_PORT_WIDTH),
		.C_OUT_PORT_WIDTH(C_OUT_PORT_WIDTH),
		.C_NUM_INPUTS(C_NUM_INPUTS),
		.C_TIMESLICE_WIDTH(C_TIMESLICE_WIDTH)

	) arbiter (
        .clk(clk), 
        .s_axis_tdata(s_axis_tdata), 
        .s_axis_tkeep(s_axis_tkeep), 
        .s_axis_tlast(s_axis_tlast), 
        .s_axis_tvalid(s_axis_tvalid), 
        .s_axis_tuser_packet_length(s_axis_tuser_packet_length), 
        .s_axis_tuser_in_port(s_axis_tuser_in_port), 
        .s_axis_tuser_out_port(s_axis_tuser_out_port),
        .s_axis_tuser_in_vport(s_axis_tuser_in_vport), 
        .s_axis_tuser_out_vport(s_axis_tuser_out_vport),  
        .s_axis_tready(s_axis_tready), 
        .m_axis_tdata(m_axis_tdata), 
        .m_axis_tkeep(m_axis_tkeep), 
        .m_axis_tlast(m_axis_tlast), 
        .m_axis_tvalid(m_axis_tvalid), 
        .m_axis_tuser_packet_length(m_axis_tuser_packet_length), 
        .m_axis_tuser_in_port(m_axis_tuser_in_port), 
        .m_axis_tuser_out_port(m_axis_tuser_out_port), 
        .m_axis_tuser_in_vport(m_axis_tuser_in_vport), 
        .m_axis_tuser_out_vport(m_axis_tuser_out_vport),
        .m_axis_tready(m_axis_tready),
        .cfg_timeslices(cfg_timeslices),
        .reset(reset)
    );
endmodule
