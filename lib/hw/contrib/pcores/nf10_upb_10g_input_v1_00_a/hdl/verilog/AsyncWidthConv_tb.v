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

module AsyncWidthConv_tb;

	// Inputs
	reg clk156;
	reg [63:0] s_axis_tdata;
	reg [7:0] s_axis_tkeep;
	reg [0:0] s_axis_tuser;
	reg s_axis_tvalid;
	reg s_axis_tlast;
	wire axi_aclk = clk;
	reg axi_resetn;
	reg m_axis_tready;

	// Outputs
	wire s_axis_tready;
	wire must_read;
	wire [255:0] m_axis_tdata;
	wire [31:0] m_axis_tkeep;
	wire [0:0] m_axis_tuser;
	wire m_axis_tvalid;
	wire m_axis_tlast;
  
  reg clk;

	// Instantiate the Unit Under Test (UUT)
	AsyncWidthConverter uut (
		.clk156(clk156), 
		.s_axis_tdata(s_axis_tdata), 
		.s_axis_tkeep(s_axis_tkeep), 
		.s_axis_tuser(s_axis_tuser), 
		.s_axis_tvalid(s_axis_tvalid), 
		.s_axis_tready(s_axis_tready), 
		.s_axis_tlast(s_axis_tlast), 
		.axi_aclk(axi_aclk), 
		.axi_resetn(axi_resetn), 
		.must_read(must_read), 
		.m_axis_tdata(m_axis_tdata), 
		.m_axis_tkeep(m_axis_tkeep), 
		.m_axis_tuser(m_axis_tuser), 
		.m_axis_tvalid(m_axis_tvalid), 
		.m_axis_tready(m_axis_tready), 
		.m_axis_tlast(m_axis_tlast)
	);

	initial begin
		// Initialize Inputs
		clk156 = 0;
		s_axis_tdata = 0;
		s_axis_tkeep = 0;
		s_axis_tuser = 0;
		s_axis_tvalid = 0;
		s_axis_tlast = 0;
		clk = 0;
		axi_resetn = 1;
		m_axis_tready = 1;

		// Wait 100 ns for global reset to finish
		#100;
        
		// Add stimulus here
    @(posedge clk156)
		s_axis_tdata = 64'hDEADBEEFDEADBEEF;
		s_axis_tkeep = 8'hFF;
		s_axis_tvalid = 1;
    #252;
    
    @(posedge clk156)    
		s_axis_tlast = 1;
		s_axis_tkeep = 8'h01;
	 #18;
    
    @(posedge clk156)   
		s_axis_tvalid = 0; 
		s_axis_tlast = 0;
    
    #120;
    @(posedge clk)
    m_axis_tready = 1;
	 
	 
	 
	 
	 
	 
	 
	 
    
    #120;
    @(posedge clk156)   
		s_axis_tkeep = 8'hFF;
		s_axis_tdata = 64'h0;
		s_axis_tvalid = 1; 
    
    forever begin
      @(posedge clk156)   
        s_axis_tdata = s_axis_tdata + 1;
    end

	end
  
  always
    #9 clk156 = ~clk156;
    
  always
    #10 clk = ~clk;
      
endmodule

