`timescale 100ns / 10ps

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

module frame_gen_and_check_tb;

	// Inputs
	reg s_axis_clk;
	reg axi_aclk;
	reg axi_resetn;
	reg m_axis_tready;
	reg [255:0] s_axis_tdata;
	reg [31:0] s_axis_tkeep;
	reg [13:0] s_axis_tuser_packet_length;
	reg [2:0] s_axis_tuser_in_port;
	reg [7:0] s_axis_tuser_out_port;
	reg [2:0] s_axis_tuser_in_vport;
	reg [7:0] s_axis_tuser_out_vport;
	reg s_axis_tvalid;
	reg s_axis_tlast;

	// Outputs
	wire [7:0] error_count;
	wire [255:0] m_axis_tdata;
	wire [31:0] m_axis_tkeep;
	wire [13:0] m_axis_tuser_packet_length;
	wire [2:0] m_axis_tuser_in_port;
	wire [7:0] m_axis_tuser_out_port;
	wire [2:0] m_axis_tuser_in_vport;
	wire [7:0] m_axis_tuser_out_vport;
	wire m_axis_tvalid;
	wire m_axis_tlast;
	wire s_axis_tready;

	// Instantiate the Unit Under Test (UUT)
	frame_gen_and_check uut (
		.error_count(error_count), 
		.s_axis_clk(s_axis_clk), 
		.axi_aclk(axi_aclk), 
		.axi_resetn(axi_resetn), 
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

  reg h = 0;
  
	initial begin
		// Initialize Inputs
		s_axis_clk = 0;
		axi_aclk = 0;
		axi_resetn = 1;
		m_axis_tready = 1;
		s_axis_tdata = 0;
		s_axis_tkeep = 0;
		s_axis_tuser_packet_length = 0;
		s_axis_tuser_in_port = 0;
		s_axis_tuser_out_port = 0;
		s_axis_tuser_in_vport = 0;
		s_axis_tuser_out_vport = 0;
		s_axis_tvalid = 0;
		s_axis_tlast = 0;
        
		// Add stimulus here
    #2;
    h = 1;
	end
  
  always begin  
    #1;
    s_axis_clk = ~s_axis_clk;
    axi_aclk = ~ axi_aclk;
  end
  
  reg [7:0] counter = 0;
  
  always @(posedge s_axis_clk) begin
    if (counter == {8{1'b1}}) begin
      counter <= 0;
      s_axis_tdata <= 256'hAFFEDEAD;
    end else begin
      counter <= counter + 1;
      s_axis_tdata <= m_axis_tdata;
    end
    
    if (h)
      s_axis_tvalid <= m_axis_tvalid;
    else
      s_axis_tvalid <= 0;
      
    s_axis_tkeep <= m_axis_tkeep;
    s_axis_tlast <= m_axis_tlast;
  end
      
endmodule

