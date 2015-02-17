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
`timescale 1ns / 1ps

module dma_input_tb;

	// Inputs
	reg CLK;
	reg axi_resetn;
	reg arbiter_m_axis_tready;
	reg [255:0] output_queue_s_axis_tdata;
	reg [31:0] output_queue_s_axis_tkeep;
	reg [2:0] output_queue_s_axis_tuser_in_port;
	reg [2:0] output_queue_s_axis_tuser_in_vport;
	reg [7:0] output_queue_s_axis_tuser_out_port;
	reg [7:0] output_queue_s_axis_tuser_out_vport;
	reg [13:0] output_queue_s_axis_tuser_packet_length;
	reg output_queue_s_axis_tvalid;
	reg output_queue_s_axis_tlast;
	reg dma_m_axis_tready;
	reg [63:0] dma_s_axis_tdata;
	reg [7:0] dma_s_axis_tkeep;
	reg [127:0] dma_s_axis_tuser;
	reg dma_s_axis_tvalid;
	reg dma_s_axis_tlast;

	// Outputs
	wire [255:0] arbiter_m_axis_tdata;
	wire [31:0] arbiter_m_axis_tkeep;
	wire [2:0] arbiter_m_axis_tuser_in_port;
	wire [2:0] arbiter_m_axis_tuser_in_vport;
	wire [7:0] arbiter_m_axis_tuser_out_port;
	wire [7:0] arbiter_m_axis_tuser_out_vport;
	wire [13:0] arbiter_m_axis_tuser_packet_length;
	wire arbiter_m_axis_tvalid;
	wire arbiter_m_axis_tlast;
	wire output_queue_s_axis_tready;
	wire [63:0] dma_m_axis_tdata;
	wire [7:0] dma_m_axis_tkeep;
	wire [127:0] dma_m_axis_tuser;
	wire dma_m_axis_tvalid;
	wire dma_m_axis_tlast;
	wire dma_s_axis_tready;

	// Instantiate the Unit Under Test (UUT)
	nf10_upb_dma_input uut (
		.CLK(CLK), 
		.axi_resetn(axi_resetn), 
		.arbiter_m_axis_tdata(arbiter_m_axis_tdata), 
		.arbiter_m_axis_tkeep(arbiter_m_axis_tkeep), 
		.arbiter_m_axis_tuser_in_port(arbiter_m_axis_tuser_in_port), 
		.arbiter_m_axis_tuser_in_vport(arbiter_m_axis_tuser_in_vport), 
		.arbiter_m_axis_tuser_out_port(arbiter_m_axis_tuser_out_port), 
		.arbiter_m_axis_tuser_out_vport(arbiter_m_axis_tuser_out_vport), 
		.arbiter_m_axis_tuser_packet_length(arbiter_m_axis_tuser_packet_length), 
		.arbiter_m_axis_tvalid(arbiter_m_axis_tvalid), 
		.arbiter_m_axis_tready(arbiter_m_axis_tready), 
		.arbiter_m_axis_tlast(arbiter_m_axis_tlast), 
		.output_queue_s_axis_tdata(output_queue_s_axis_tdata), 
		.output_queue_s_axis_tkeep(output_queue_s_axis_tkeep), 
		.output_queue_s_axis_tuser_in_port(output_queue_s_axis_tuser_in_port), 
		.output_queue_s_axis_tuser_in_vport(output_queue_s_axis_tuser_in_vport), 
		.output_queue_s_axis_tuser_out_port(output_queue_s_axis_tuser_out_port), 
		.output_queue_s_axis_tuser_out_vport(output_queue_s_axis_tuser_out_vport), 
		.output_queue_s_axis_tuser_packet_length(output_queue_s_axis_tuser_packet_length), 
		.output_queue_s_axis_tvalid(output_queue_s_axis_tvalid), 
		.output_queue_s_axis_tready(output_queue_s_axis_tready), 
		.output_queue_s_axis_tlast(output_queue_s_axis_tlast), 
		.dma_m_axis_tdata(dma_m_axis_tdata), 
		.dma_m_axis_tkeep(dma_m_axis_tkeep), 
		.dma_m_axis_tuser(dma_m_axis_tuser), 
		.dma_m_axis_tvalid(dma_m_axis_tvalid), 
		.dma_m_axis_tready(dma_m_axis_tready), 
		.dma_m_axis_tlast(dma_m_axis_tlast), 
		.dma_s_axis_tdata(dma_s_axis_tdata), 
		.dma_s_axis_tkeep(dma_s_axis_tkeep), 
		.dma_s_axis_tuser(dma_s_axis_tuser), 
		.dma_s_axis_tvalid(dma_s_axis_tvalid), 
		.dma_s_axis_tready(dma_s_axis_tready), 
		.dma_s_axis_tlast(dma_s_axis_tlast)
	);

	initial begin
		// Initialize Inputs
		CLK = 0;
		axi_resetn = 1;
		arbiter_m_axis_tready = 0;
		output_queue_s_axis_tdata = 0;
		output_queue_s_axis_tkeep = 0;
		output_queue_s_axis_tuser_in_port = 0;
		output_queue_s_axis_tuser_in_vport = 0;
		output_queue_s_axis_tuser_out_port = 0;
		output_queue_s_axis_tuser_out_vport = 0;
		output_queue_s_axis_tuser_packet_length = 0;
		output_queue_s_axis_tvalid = 0;
		output_queue_s_axis_tlast = 0;
		dma_m_axis_tready = 1; // let dma always receive
		dma_s_axis_tdata = 0;
		dma_s_axis_tkeep = 0;
		dma_s_axis_tuser = 0;
		dma_s_axis_tvalid = 0;
		dma_s_axis_tlast = 0;

		// Wait 100 ns for global reset to finish
		#100;
        
		// Test data flow from dma to arbiter
		@(posedge CLK);
		dma_s_axis_tdata = 64'h11;
		dma_s_axis_tuser = {
			115'b0,
			3'b101, // src port
			10'b0101010101 // packet size
		};
		dma_s_axis_tkeep = {(8){1'b1}};
		dma_s_axis_tlast = 0;
		dma_s_axis_tvalid = 1;
		@(posedge CLK && dma_s_axis_tready)
		dma_s_axis_tkeep = 8'h1f;
		dma_s_axis_tdata = 12;
		dma_s_axis_tlast = 1;
		@(posedge CLK && dma_s_axis_tready)
		dma_s_axis_tvalid = 0;
		@(arbiter_m_axis_tvalid); // Wait for FIFO fall through
		#4; // wait a bit longer...
		arbiter_m_axis_tready = 1;


		#200;
		
		// Test data flow from output queue to dma
		@(posedge CLK);
		output_queue_s_axis_tdata = 256'haffedead;
		output_queue_s_axis_tkeep = {3'b0, {(29){1'b1}}};
		output_queue_s_axis_tuser_in_port = 3'd5;
		output_queue_s_axis_tuser_in_vport = 3'd6;
		output_queue_s_axis_tuser_out_port = 8'd7;
		output_queue_s_axis_tuser_out_vport = 8'd8;
		output_queue_s_axis_tuser_packet_length = 14'd17;
		output_queue_s_axis_tlast = 1;
		output_queue_s_axis_tvalid = 1;
		wait(output_queue_s_axis_tready);
		@(posedge CLK) output_queue_s_axis_tvalid = 0;

	end
	
	always begin
		#1 CLK = ~CLK;
	end
      
endmodule
