/*
 * UPB Output Queue testbench
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


`timescale 1ns / 1ps
`default_nettype none

module main_tb;

	logic clk100 = 0;
	
	always
		#5 clk100 = ~clk100;
		
		
	wire qdr_a_k;					// K
	wire qdr_a_k_n;				// K
	wire qdr_a_c;					// C
	wire qdr_a_c_n;				// Cn
	logic qdr_a_cq, qdr_a_cq_memory;					// CQ
	logic qdr_a_cq_n, qdr_a_cq_n_memory;				// CQn
	wire [18:0] qdr_a_sa;		// A[18:0]
	wire qdr_a_r_n;				// RPS
	logic [35:0] qdr_a_q, qdr_a_q_memory;			// Q[35:0]
	wire qdr_a_w_n;				// WPS
	wire [3:0] qdr_a_bw_n;		// BWS[3:0]
	wire [35:0] qdr_a_d;			// D[35:0]

	main uut (
		.clk100(clk100),
		.pb_n(2'b11),
		
		.qdr_a_k(qdr_a_k),
		.qdr_a_k_n(qdr_a_k_n),
		.qdr_a_c(qdr_a_c),
		.qdr_a_c_n(qdr_a_c_n),
		.qdr_a_cq(qdr_a_cq),
		.qdr_a_cq_n(qdr_a_cq_n),
		.qdr_a_sa(qdr_a_sa),
		.qdr_a_r_n(qdr_a_r_n),
		.qdr_a_q(qdr_a_q),
		.qdr_a_w_n(qdr_a_w_n),
		.qdr_a_bw_n(qdr_a_bw_n),
		.qdr_a_d(qdr_a_d)
	);
	
	CY7C1515JV18 qdr_a (
		.D(qdr_a_d),
		.Q(qdr_a_q_memory),
		.A(qdr_a_sa),
		.RPS_n(qdr_a_r_n),
		.WPS_n(qdr_a_w_n),
		.BW_n(qdr_a_bw_n),
		.K(qdr_a_k),
		.K_n(qdr_a_k_n),
		.C(qdr_a_c),
		.C_n(qdr_a_c_n),
		.CQ(qdr_a_cq_memory),
		.CQ_n(qdr_a_cq_n_memory)
	);
	
	// simulate delay on the PCB's traces and on the FPGA's IOB
	always @(*) begin
		qdr_a_cq <= #0 qdr_a_cq_memory;
		qdr_a_cq_n <= #0 qdr_a_cq_n_memory;
		
		qdr_a_q <= #6 qdr_a_q_memory;
	end

	initial begin

	end
      
endmodule
