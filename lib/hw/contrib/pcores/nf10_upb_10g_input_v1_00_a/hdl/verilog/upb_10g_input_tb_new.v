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

module upb_10g_input_tb_new;

	// Inputs
	reg clk156;
	reg mac_m_axis_tready;
	reg [63:0] mac_s_axis_tdata;
	reg [7:0] mac_s_axis_tkeep;
	reg [0:0] mac_s_axis_tuser;
	reg mac_s_axis_tvalid;
	reg mac_s_axis_tlast;
	reg axi_aclk;
	reg axi_resetn;
	reg arbiter_m_axis_tready;
	reg [255:0] output_queue_s_axis_tdata;
	reg [31:0] output_queue_s_axis_tkeep;
	reg [13:0] output_queue_s_axis_tuser_packet_length;
	reg [3:0] output_queue_s_axis_tuser_in_port;
	reg [3:0] output_queue_s_axis_tuser_out_port;
	reg output_queue_s_axis_tvalid;
	reg output_queue_s_axis_tlast;

	// Outputs
	wire [63:0] mac_m_axis_tdata;
	wire [7:0] mac_m_axis_tkeep;
	wire [0:0] mac_m_axis_tuser;
	wire mac_m_axis_tvalid;
	wire mac_m_axis_tlast;
	wire mac_s_axis_tready;
	wire [255:0] arbiter_m_axis_tdata;
	wire [31:0] arbiter_m_axis_tkeep;
	wire [13:0] arbiter_m_axis_tuser_packet_length;
	wire [3:0] arbiter_m_axis_tuser_in_port;
	wire [3:0] arbiter_m_axis_tuser_out_port;
	wire arbiter_m_axis_tvalid;
	wire arbiter_m_axis_tlast;
	wire output_queue_s_axis_tready;
	wire output_queue_clk;

	// Instantiate the Unit Under Test (UUT)
	nf10_upb_10g_input #(.C_MAX_PACKET_LENGTH(150)) uut (
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
		.arbiter_m_axis_tdata(arbiter_m_axis_tdata), 
		.arbiter_m_axis_tkeep(arbiter_m_axis_tkeep), 
		.arbiter_m_axis_tuser_packet_length(arbiter_m_axis_tuser_packet_length), 
		.arbiter_m_axis_tuser_in_port(arbiter_m_axis_tuser_in_port), 
		.arbiter_m_axis_tuser_out_port(arbiter_m_axis_tuser_out_port), 
		.arbiter_m_axis_tvalid(arbiter_m_axis_tvalid), 
		.arbiter_m_axis_tready(arbiter_m_axis_tready), 
		.arbiter_m_axis_tlast(arbiter_m_axis_tlast), 
		.output_queue_s_axis_tdata(output_queue_s_axis_tdata), 
		.output_queue_s_axis_tkeep(output_queue_s_axis_tkeep), 
		.output_queue_s_axis_tuser_packet_length(output_queue_s_axis_tuser_packet_length), 
		.output_queue_s_axis_tuser_in_port(output_queue_s_axis_tuser_in_port), 
		.output_queue_s_axis_tuser_out_port(output_queue_s_axis_tuser_out_port), 
		.output_queue_s_axis_tvalid(output_queue_s_axis_tvalid), 
		.output_queue_s_axis_tready(output_queue_s_axis_tready), 
		.output_queue_s_axis_tlast(output_queue_s_axis_tlast)
		//.output_queue_clk(output_queue_clk)
	);

	initial begin
		// Initialize Inputs
		clk156 = 0;
		mac_m_axis_tready = 0;
		mac_s_axis_tdata = 0;
		mac_s_axis_tkeep = 0;
		mac_s_axis_tuser = 0;
		mac_s_axis_tvalid = 0;
		mac_s_axis_tlast = 0;
		axi_aclk = 0;
		axi_resetn = 1;
		arbiter_m_axis_tready = 1;
		output_queue_s_axis_tdata = 0;
		output_queue_s_axis_tkeep = 0;
		output_queue_s_axis_tuser_packet_length = 0;
		output_queue_s_axis_tuser_in_port = 0;
		output_queue_s_axis_tuser_out_port = 0;
		output_queue_s_axis_tvalid = 0;
		output_queue_s_axis_tlast = 0;

		// Wait 100 ns for global reset to finish
		#100;
        
		// Add stimulus here
    // receive one pkt
    @(posedge clk156) begin
      mac_s_axis_tdata = 64'hDEADBEEFAFFEDEAD;
      mac_s_axis_tkeep = 8'hFF;
      mac_s_axis_tvalid = 1;
    end
    
    #89;
    @(posedge clk156) begin
      mac_s_axis_tlast = 1;
    end
    
    #17;
    @(posedge clk156) begin
      mac_s_axis_tvalid = 0;
      mac_s_axis_tlast = 0;
    end
    
    #200;
    // receive another pkt
    @(posedge clk156) begin
      mac_s_axis_tdata = 64'hDEADBEEFAFFE0002;
      mac_s_axis_tkeep = 8'hFF;
      mac_s_axis_tvalid = 1;
    end
    
    #400; // pkt is too long
    @(posedge clk156) begin
      mac_s_axis_tlast = 1;
    end
    
    #17;
    @(posedge clk156) begin
      mac_s_axis_tvalid = 0;
      mac_s_axis_tlast = 0;
    end
    
    #200;
    // receive yet another pkt
    @(posedge clk156) begin
      mac_s_axis_tdata = 64'hDEADBEEFAFFE0003;
      mac_s_axis_tkeep = 8'hFF;
      mac_s_axis_tvalid = 1;
    end
    
    #89;
    @(posedge clk156) begin
      mac_s_axis_tlast = 1;
      mac_s_axis_tkeep = 8'h0F;
    end
    
    #17;
    @(posedge clk156) begin
      mac_s_axis_tvalid = 0;
      mac_s_axis_tlast = 0;
    end   

    #200;
    // and receive yet another pkt
    @(posedge clk156) begin
      mac_s_axis_tdata = 64'hDEADBEEFAFFE0003;
      mac_s_axis_tkeep = 8'hFF;
      mac_s_axis_tvalid = 1;
    end
    
    #89;
    @(posedge clk156) begin
      mac_s_axis_tlast = 1;
      mac_s_axis_tuser = 1; // bad frame
      mac_s_axis_tkeep = 8'h0F;
    end
    
    #17;
    @(posedge clk156) begin
      mac_s_axis_tvalid = 0;
      mac_s_axis_tlast = 0;
      mac_s_axis_tuser = 0;
    end       

	end
  
  always
    #9 clk156 = ~clk156;
    
  always
    #10 axi_aclk = ~axi_aclk;
      
endmodule

