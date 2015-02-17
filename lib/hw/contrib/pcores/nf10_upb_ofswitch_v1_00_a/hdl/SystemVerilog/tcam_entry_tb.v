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

module tcam_entry_tb;
  
	logic CLK;
	logic RST;
	
	logic [9:0] content;
	logic wdata;
	logic waddr;
	logic wen;
	
	wire match;
	
	// UUT
	upb_tcam_entry #(
		.SRL_SIZE(32),    // use 32 here to generate SRLC32E instances
		.TCAM_WIDTH(2),  // number of SRLs in parallel, so total width is log2(SRL_SIZE) * TCAM_WIDTH
		.INPUT_WIDTH(1), // input width for feeding data in
		.FORCE_MUXCY(0)   // force usage of carry chain on current Xilinx devices
	) uut (
		CLK,
		RST,
		content,
		wdata,
		waddr,
		wen,
		match
	);

	logic [31:0] srl_data [1:0];
	assign srl_data[0] = 32'haffedeaf;
	assign srl_data[1] = 32'hdeadbeef;

  initial begin
    CLK = 1;
    RST = 0;
    content = 10'b0;
    wdata = 0;
    waddr = 0;
    wen = 0;
    
    #100
    
	 // feed data into tcam entry
    for (int i = 1; i >= 0; i--) begin
		for (int j = 31; j >= 0; j--) begin
			waddr = i;
			wen = 1;
			wdata = srl_data[i][j];
			#2;
		end
	 end
	 wen = 0;
	 
	 #10;
	 
	 // look up contents
	 for (int i = 0; i < 1024; i++) begin
		content = i;
		#2
		if (match != (srl_data[0][i] && srl_data[1][i >> 5])) begin
			$display("Error!");
			$display("  Content: ", content);
			$display("  Match: ", match);
			$stop();
		end
	 end
    
	 $display("All fine :)");
    #100 $stop();
  end
  
  always begin
    #1 CLK <= ~CLK;
  end
  
endmodule
