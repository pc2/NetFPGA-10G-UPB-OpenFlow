/*
 * UPB Output Queue core
 *
 * Copyright (c) 2014, 2015 Jörg Niklas
 * osjsn@niklasfamily.de
 *
 * This file is part of the NetFPGA 10G UPB OpenFlow Switch project.
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

`default_nettype none

module nf10_upb_output_queue #(

	parameter output_ports 										= 7,
	parameter axis_data_width									= 256,
	parameter axis_tkeep_width									= axis_data_width / 8,
	parameter axis_tuser_in_port_width						= 3,
	parameter axis_tuser_out_port_width						= 8,
	parameter axis_tuser_packet_length_width				= 14

)(

	input	wire														clk,
	input	wire														reset,
	
	input	wire														clk2x,
	input	wire														clk2x90,
	
	input wire [axis_data_width-1:0]							s_axis_tdata,
	input wire [axis_tkeep_width-1:0]						s_axis_tkeep,
	input wire														s_axis_tlast,
	input wire 														s_axis_tvalid,
	output wire														s_axis_tready,
	input wire [axis_tuser_in_port_width-1:0]				s_axis_tuser_in_port, s_axis_tuser_in_vport,
	input wire [axis_tuser_out_port_width-1:0]			s_axis_tuser_out_port, s_axis_tuser_out_vport,
	input wire [axis_tuser_packet_length_width-1:0]		s_axis_tuser_packet_length,
	
	input wire														m_axis_0_clk,
	input wire														m_axis_0_reset,
	output wire [axis_data_width-1:0]						m_axis_0_tdata,
	output wire [axis_tkeep_width-1:0]						m_axis_0_tkeep,
	output wire														m_axis_0_tlast,
	output wire 													m_axis_0_tvalid,
	input wire														m_axis_0_tready,
	output wire [axis_tuser_in_port_width-1:0]			m_axis_0_tuser_in_port, m_axis_0_tuser_in_vport,
	output wire [axis_tuser_out_port_width-1:0]			m_axis_0_tuser_out_port, m_axis_0_tuser_out_vport,
	output wire [axis_tuser_packet_length_width-1:0]	m_axis_0_tuser_packet_length,
	
	input wire														m_axis_1_clk,
	input wire														m_axis_1_reset,
	output wire [axis_data_width-1:0]						m_axis_1_tdata,
	output wire [axis_tkeep_width-1:0]						m_axis_1_tkeep,
	output wire														m_axis_1_tlast,
	output wire 													m_axis_1_tvalid,
	input wire														m_axis_1_tready,
	output wire [axis_tuser_in_port_width-1:0]			m_axis_1_tuser_in_port, m_axis_1_tuser_in_vport,
	output wire [axis_tuser_out_port_width-1:0]			m_axis_1_tuser_out_port, m_axis_1_tuser_out_vport,
	output wire [axis_tuser_packet_length_width-1:0]	m_axis_1_tuser_packet_length,
	
	input wire														m_axis_2_clk,
	input wire														m_axis_2_reset,
	output wire [axis_data_width-1:0]						m_axis_2_tdata,
	output wire [axis_tkeep_width-1:0]						m_axis_2_tkeep,
	output wire														m_axis_2_tlast,
	output wire 													m_axis_2_tvalid,
	input wire														m_axis_2_tready,
	output wire [axis_tuser_in_port_width-1:0]			m_axis_2_tuser_in_port, m_axis_2_tuser_in_vport,
	output wire [axis_tuser_out_port_width-1:0]			m_axis_2_tuser_out_port, m_axis_2_tuser_out_vport,
	output wire [axis_tuser_packet_length_width-1:0]	m_axis_2_tuser_packet_length,
	
	input wire														m_axis_3_clk,
	input wire														m_axis_3_reset,
	output wire [axis_data_width-1:0]						m_axis_3_tdata,
	output wire [axis_tkeep_width-1:0]						m_axis_3_tkeep,
	output wire														m_axis_3_tlast,
	output wire 													m_axis_3_tvalid,
	input wire														m_axis_3_tready,
	output wire [axis_tuser_in_port_width-1:0]			m_axis_3_tuser_in_port, m_axis_3_tuser_in_vport,
	output wire [axis_tuser_out_port_width-1:0]			m_axis_3_tuser_out_port, m_axis_3_tuser_out_vport,
	output wire [axis_tuser_packet_length_width-1:0]	m_axis_3_tuser_packet_length,
	
		
	input wire														m_axis_4_clk,
	input wire														m_axis_4_reset,
	output wire [axis_data_width-1:0]						m_axis_4_tdata,
	output wire [axis_tkeep_width-1:0]						m_axis_4_tkeep,
	output wire														m_axis_4_tlast,
	output wire 													m_axis_4_tvalid,
	input wire														m_axis_4_tready,
	output wire [axis_tuser_in_port_width-1:0]			m_axis_4_tuser_in_port, m_axis_4_tuser_in_vport,
	output wire [axis_tuser_out_port_width-1:0]			m_axis_4_tuser_out_port, m_axis_4_tuser_out_vport,
	output wire [axis_tuser_packet_length_width-1:0]	m_axis_4_tuser_packet_length,
	
		
	input wire														m_axis_5_clk,
	input wire														m_axis_5_reset,
	output wire [axis_data_width-1:0]						m_axis_5_tdata,
	output wire [axis_tkeep_width-1:0]						m_axis_5_tkeep,
	output wire														m_axis_5_tlast,
	output wire 													m_axis_5_tvalid,
	input wire														m_axis_5_tready,
	output wire [axis_tuser_in_port_width-1:0]			m_axis_5_tuser_in_port, m_axis_5_tuser_in_vport,
	output wire [axis_tuser_out_port_width-1:0]			m_axis_5_tuser_out_port, m_axis_5_tuser_out_vport,
	output wire [axis_tuser_packet_length_width-1:0]	m_axis_5_tuser_packet_length,
	
		
	input wire														m_axis_6_clk,
	input wire														m_axis_6_reset,
	output wire [axis_data_width-1:0]						m_axis_6_tdata,
	output wire [axis_tkeep_width-1:0]						m_axis_6_tkeep,
	output wire														m_axis_6_tlast,
	output wire 													m_axis_6_tvalid,
	input wire														m_axis_6_tready,
	output wire [axis_tuser_in_port_width-1:0]			m_axis_6_tuser_in_port, m_axis_6_tuser_in_vport,
	output wire [axis_tuser_out_port_width-1:0]			m_axis_6_tuser_out_port, m_axis_6_tuser_out_vport,
	output wire [axis_tuser_packet_length_width-1:0]	m_axis_6_tuser_packet_length,
	
	output wire														qdr_a_k,				// K
	output wire														qdr_a_k_n,			// K
	output wire														qdr_a_c,				// C
	output wire														qdr_a_c_n,			// Cn
	input	wire														qdr_a_cq,			// CQ
	input	wire														qdr_a_cq_n,			// CQn
	output wire [18:0]											qdr_a_sa,			// A[18:0]
	output wire														qdr_a_r_n,			// RPS
	input wire [35:0]												qdr_a_q,				// Q[35:0]
	output wire														qdr_a_w_n,			// WPS
	output wire [3:0]												qdr_a_bw_n,			// BWS[3:0]
	output wire [35:0]											qdr_a_d,				// D[35:0]
	output wire														qdr_a_dll_off_n,	// DOFFn
	
	output wire														qdr_b_k,				// K
	output wire														qdr_b_k_n,			// K
	output wire														qdr_b_c,				// C
	output wire														qdr_b_c_n,			// Cn
	input	wire														qdr_b_cq,			// CQ
	input wire														qdr_b_cq_n,			// CQn
	output wire [18:0]											qdr_b_sa,			// A[18:0]
	output wire														qdr_b_r_n,			// RPS
	input wire [35:0]												qdr_b_q,				// Q[35:0]
	output wire														qdr_b_w_n,			// WPS
	output wire [3:0]												qdr_b_bw_n,			// BWS[3:0]
	output wire [35:0]											qdr_b_d,				// D[35:0]
	output wire														qdr_b_dll_off_n,	// DOFFn

	inout wire [35:0]												chipscope_0,
	inout wire [35:0]												chipscope_1,
	inout wire [35:0]												chipscope_2
);

axis_defs::tuser_t s_axis_tuser;

logic m_axis_clk[7];
logic m_axis_reset[7];
logic [axis_data_width-1:0] m_axis_tdata[7];
logic [axis_tkeep_width-1:0] m_axis_tkeep[7];
axis_defs::tuser_t m_axis_tuser[7];
logic m_axis_tlast[7];
logic m_axis_tvalid[7];
wire m_axis_tready[7];

qdr2_sram_if qdr[2]();

output_queue #(

	.output_ports								(output_ports),
	.axis_data_width							(axis_data_width),
	
	.multicast_queue_size_256b				(2**9), // 16KB
	.store_and_forward_ports				( { 1'b1, 1'b1, 1'b1, 1'b1 } )

) output_queue_0 (
	.*
);

assign s_axis_tuser.in.port = s_axis_tuser_in_port;
assign s_axis_tuser.in.vport = s_axis_tuser_in_vport;
assign s_axis_tuser.out.port = s_axis_tuser_out_port;
assign s_axis_tuser.out.vport = s_axis_tuser_out_vport;
assign s_axis_tuser.packet_length = s_axis_tuser_packet_length;

assign m_axis_clk = '{ m_axis_0_clk, m_axis_1_clk, m_axis_2_clk, m_axis_3_clk, m_axis_4_clk, m_axis_5_clk, m_axis_6_clk };
assign m_axis_reset = '{ m_axis_0_reset, m_axis_1_reset, m_axis_2_reset, m_axis_3_reset, m_axis_4_reset, m_axis_5_reset, m_axis_6_reset };

assign m_axis_0_tdata = m_axis_tdata[0];
assign m_axis_1_tdata = m_axis_tdata[1];
assign m_axis_2_tdata = m_axis_tdata[2];
assign m_axis_3_tdata = m_axis_tdata[3];
assign m_axis_4_tdata = m_axis_tdata[4];
assign m_axis_5_tdata = m_axis_tdata[5];
assign m_axis_6_tdata = m_axis_tdata[6];


assign m_axis_0_tkeep = m_axis_tkeep[0];
assign m_axis_1_tkeep = m_axis_tkeep[1];
assign m_axis_2_tkeep = m_axis_tkeep[2];
assign m_axis_3_tkeep = m_axis_tkeep[3];
assign m_axis_4_tkeep = m_axis_tkeep[4];
assign m_axis_5_tkeep = m_axis_tkeep[5];
assign m_axis_6_tkeep = m_axis_tkeep[6];

assign m_axis_0_tuser_in_port = m_axis_tuser[0].in.port;
assign m_axis_0_tuser_in_vport = m_axis_tuser[0].in.vport;
assign m_axis_0_tuser_out_port = m_axis_tuser[0].out.port;
assign m_axis_0_tuser_out_vport = m_axis_tuser[0].out.vport;
assign m_axis_0_tuser_packet_length = m_axis_tuser[0].packet_length;

assign m_axis_1_tuser_in_port = m_axis_tuser[1].in.port;
assign m_axis_1_tuser_in_vport = m_axis_tuser[1].in.vport;
assign m_axis_1_tuser_out_port = m_axis_tuser[1].out.port;
assign m_axis_1_tuser_out_vport = m_axis_tuser[1].out.vport;
assign m_axis_1_tuser_packet_length = m_axis_tuser[1].packet_length;

assign m_axis_2_tuser_in_port = m_axis_tuser[2].in.port;
assign m_axis_2_tuser_in_vport = m_axis_tuser[2].in.vport;
assign m_axis_2_tuser_out_port = m_axis_tuser[2].out.port;
assign m_axis_2_tuser_out_vport = m_axis_tuser[2].out.vport;
assign m_axis_2_tuser_packet_length = m_axis_tuser[2].packet_length;

assign m_axis_3_tuser_in_port = m_axis_tuser[3].in.port;
assign m_axis_3_tuser_in_vport = m_axis_tuser[3].in.vport;
assign m_axis_3_tuser_out_port = m_axis_tuser[3].out.port;
assign m_axis_3_tuser_out_vport = m_axis_tuser[3].out.vport;
assign m_axis_3_tuser_packet_length = m_axis_tuser[3].packet_length;

assign m_axis_4_tuser_in_port = m_axis_tuser[4].in.port;
assign m_axis_4_tuser_in_vport = m_axis_tuser[4].in.vport;
assign m_axis_4_tuser_out_port = m_axis_tuser[4].out.port;
assign m_axis_4_tuser_out_vport = m_axis_tuser[4].out.vport;
assign m_axis_4_tuser_packet_length = m_axis_tuser[4].packet_length;

assign m_axis_5_tuser_in_port = m_axis_tuser[5].in.port;
assign m_axis_5_tuser_in_vport = m_axis_tuser[5].in.vport;
assign m_axis_5_tuser_out_port = m_axis_tuser[5].out.port;
assign m_axis_5_tuser_out_vport = m_axis_tuser[5].out.vport;
assign m_axis_5_tuser_packet_length = m_axis_tuser[5].packet_length;

assign m_axis_6_tuser_in_port = m_axis_tuser[6].in.port;
assign m_axis_6_tuser_in_vport = m_axis_tuser[6].in.vport;
assign m_axis_6_tuser_out_port = m_axis_tuser[6].out.port;
assign m_axis_6_tuser_out_vport = m_axis_tuser[6].out.vport;
assign m_axis_6_tuser_packet_length = m_axis_tuser[6].packet_length;

assign m_axis_0_tlast = m_axis_tlast[0];
assign m_axis_1_tlast = m_axis_tlast[1];
assign m_axis_2_tlast = m_axis_tlast[2];
assign m_axis_3_tlast = m_axis_tlast[3];
assign m_axis_4_tlast = m_axis_tlast[4];
assign m_axis_5_tlast = m_axis_tlast[5];
assign m_axis_6_tlast = m_axis_tlast[6];

assign m_axis_0_tvalid = m_axis_tvalid[0];
assign m_axis_1_tvalid = m_axis_tvalid[1];
assign m_axis_2_tvalid = m_axis_tvalid[2];
assign m_axis_3_tvalid = m_axis_tvalid[3];
assign m_axis_4_tvalid = m_axis_tvalid[4];
assign m_axis_5_tvalid = m_axis_tvalid[5];
assign m_axis_6_tvalid = m_axis_tvalid[6];

assign m_axis_tready = '{ m_axis_0_tready, m_axis_1_tready, m_axis_2_tready, m_axis_3_tready, m_axis_4_tready, m_axis_5_tready, m_axis_6_tready };

assign qdr_a_k = qdr[0].k;
assign qdr_a_k_n = qdr[0].k_n;
assign qdr_a_c = qdr[0].c;
assign qdr_a_c_n = qdr[0].c_n;
assign qdr[0].cq = qdr_a_cq;
assign qdr[0].cq_n = qdr_a_cq_n;
assign qdr_a_sa = qdr[0].sa;
assign qdr_a_r_n = qdr[0].r_n;
assign qdr[0].q = qdr_a_q;
assign qdr_a_w_n = qdr[0].w_n;
assign qdr_a_bw_n = qdr[0].bw_n;
assign qdr_a_d = qdr[0].d;
assign qdr_a_dll_off_n = qdr[0].dll_off_n;

assign qdr_b_k = qdr[1].k;
assign qdr_b_k_n = qdr[1].k_n;
assign qdr_b_c = qdr[1].c;
assign qdr_b_c_n = qdr[1].c_n;
assign qdr[1].cq = qdr_b_cq;
assign qdr[1].cq_n = qdr_b_cq_n;
assign qdr_b_sa = qdr[1].sa;
assign qdr_b_r_n = qdr[1].r_n;
assign qdr[1].q = qdr_b_q;
assign qdr_b_w_n = qdr[1].w_n;
assign qdr_b_bw_n = qdr[1].bw_n;
assign qdr_b_d = qdr[1].d;
assign qdr_b_dll_off_n = qdr[1].dll_off_n;


endmodule
