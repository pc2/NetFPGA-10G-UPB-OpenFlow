`timescale 10ns / 10ps

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

module width_div_tb;

	// Inputs
	reg clk;
	reg reset;
	reg m_axis_tready;
	reg [255:0] s_axis_tdata;
	reg [31:0] s_axis_tkeep;
	reg s_axis_tvalid;
	reg [0:0] s_axis_tuser;
	reg s_axis_tlast;

	// Outputs
	wire [63:0] m_axis_tdata;
	wire [7:0] m_axis_tkeep;
	wire m_axis_tvalid;
	wire [0:0] m_axis_tuser;
	wire m_axis_tlast;
	wire s_axis_tready;

	// Instantiate the Unit Under Test (UUT)
	width_divider uut (
		.clk(clk), 
		.reset(reset), 
		.m_axis_tdata(m_axis_tdata), 
		.m_axis_tkeep(m_axis_tkeep), 
		.m_axis_tvalid(m_axis_tvalid), 
		.m_axis_tready(m_axis_tready), 
		.m_axis_tuser(m_axis_tuser), 
		.m_axis_tlast(m_axis_tlast), 
		.s_axis_tdata(s_axis_tdata), 
		.s_axis_tkeep(s_axis_tkeep), 
		.s_axis_tvalid(s_axis_tvalid), 
		.s_axis_tready(s_axis_tready), 
		.s_axis_tuser(s_axis_tuser), 
		.s_axis_tlast(s_axis_tlast)
	);

	initial begin
		// Initialize Inputs
		clk = 0;
		reset = 0;
		m_axis_tready = 0;
		s_axis_tdata = 0;
		s_axis_tkeep = 0;
		s_axis_tvalid = 0;
		s_axis_tuser = 0;
		s_axis_tlast = 0;

		// Wait 100 ns for global reset to finish
		#100;
        
		// Add stimulus here
    @(posedge clk)
		m_axis_tready = 1;
		s_axis_tdata = {2{128'hDEADBEEFDEADBEEFAFFEDEADAFFEDEAD}};
		s_axis_tkeep = 32'hFFFFFFFF;
		s_axis_tvalid = 1;
    

	end
  
  always
    #1 clk = ~clk;
      
endmodule

