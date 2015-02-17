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

module xgmac_to_axi_converter_tb;

	// Inputs
	reg reset;
	reg clk156;
	reg tx_ack;
	reg [63:0] rx_data;
	reg [7:0] rx_data_valid;
	reg rx_good_frame;
	reg rx_bad_frame;
	reg m_axis_tready;
	reg [63:0] s_axis_tdata;
	reg [7:0] s_axis_tkeep;
	reg [0:0] s_axis_tuser;
	reg s_axis_tvalid;
	reg s_axis_tlast;

	// Outputs
	wire [63:0] tx_data;
	wire [7:0] tx_data_valid;
	wire tx_start;
	wire [63:0] m_axis_tdata;
	wire [7:0] m_axis_tkeep;
	wire [0:0] m_axis_tuser;
	wire m_axis_tvalid;
	wire m_axis_tlast;
	wire s_axis_tready;

	// Instantiate the Unit Under Test (UUT)
	xgmac_to_axi_converter uut (
		.reset(reset), 
		.clk156(clk156), 
		.tx_data(tx_data), 
		.tx_data_valid(tx_data_valid), 
		.tx_start(tx_start), 
		.tx_ack(tx_ack), 
		.rx_data(rx_data), 
		.rx_data_valid(rx_data_valid), 
		.rx_good_frame(rx_good_frame), 
		.rx_bad_frame(rx_bad_frame), 
		.m_axis_tdata(m_axis_tdata), 
		.m_axis_tkeep(m_axis_tkeep), 
		.m_axis_tuser(m_axis_tuser), 
		.m_axis_tvalid(m_axis_tvalid), 
		.m_axis_tready(m_axis_tready), 
		.m_axis_tlast(m_axis_tlast), 
		.s_axis_tdata(s_axis_tdata), 
		.s_axis_tkeep(s_axis_tkeep), 
		.s_axis_tuser(s_axis_tuser), 
		.s_axis_tvalid(s_axis_tvalid), 
		.s_axis_tready(s_axis_tready), 
		.s_axis_tlast(s_axis_tlast)
	);

	initial begin
		// Initialize Inputs
		reset = 0;
		clk156 = 0;
		tx_ack = 0;
		rx_data = 0;
		rx_data_valid = 0;
		rx_good_frame = 0;
		rx_bad_frame = 0;
		m_axis_tready = 0;
		s_axis_tdata = 1000;
		s_axis_tkeep = 0;
		s_axis_tuser = 0;
		s_axis_tvalid = 0;
		s_axis_tlast = 0;

		// Wait for global reset to finish
		#101;
        
		// Add stimulus here
    // testing reception
    @(posedge clk156) begin
      rx_data_valid = 255;
      #30;
    @(posedge clk156)
      rx_data_valid = 3;
      #2;
    @(posedge clk156)
      rx_data_valid = 0;
      #12;
    @(posedge clk156)
      rx_good_frame = 1; // latest possible time
      #2;
    @(posedge clk156)
      rx_good_frame = 0;
      #100;
      // testing transmission
    @(posedge clk156)
      s_axis_tvalid = 1;
      s_axis_tkeep = 255;
      #30;
    @(posedge clk156)
      s_axis_tkeep = 3;
      s_axis_tlast = 1;
      #2;
    @(posedge clk156)
      s_axis_tvalid = 0;
      s_axis_tkeep = 0;
      s_axis_tlast = 0;
    end
    
    // testing reception
    @(posedge clk156) begin
      rx_data_valid = 255;
      #1000;
    @(posedge clk156)
      rx_data_valid = 3;
      #2;
    @(posedge clk156)
      rx_data_valid = 0;
      #12;
    @(posedge clk156)
      rx_bad_frame = 1; // latest possible time
      #2;
    @(posedge clk156)
      rx_bad_frame = 0;
      #100;
      // testing transmission
    @(posedge clk156)
      s_axis_tvalid = 1;
      s_axis_tkeep = 255;
      #30;
    @(posedge clk156)
      s_axis_tkeep = 3;
      s_axis_tlast = 1;
      #2;
    @(posedge clk156)
      s_axis_tvalid = 0;
      s_axis_tkeep = 0;
      s_axis_tlast = 0;
    end
	end
  
  always
    #1 clk156 = ~clk156;
    
  always begin #1  forever begin
    #2 if (rx_data < 1000)
      rx_data <= rx_data + 1;
    if (s_axis_tdata < 2000)
      s_axis_tdata <= s_axis_tdata + 1;
  end end
  
  always
    #1 if (tx_start) begin
    @(posedge clk156)
      #9 tx_ack = 1;
      #2 tx_ack = 0;
    end
      
endmodule

