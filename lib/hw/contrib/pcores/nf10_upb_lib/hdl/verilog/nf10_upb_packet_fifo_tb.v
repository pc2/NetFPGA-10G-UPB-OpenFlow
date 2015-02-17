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

module nf10_upb_packet_fifo_tb;

	// Inputs
	reg CLK;
	reg [31:0] DI;
	reg [7:0] MI;
	reg RDEN;
	reg WREN;
	reg COMMIT;
	reg REVERT;
	reg RST;

	// Outputs
	wire [31:0] DO;
	wire [7:0] MO;
	wire EOP;
	wire FULL;
	wire EMPTY;
	wire RDERR;
	wire WRERR;
	wire BELOW_LOW;
	wire ABOVE_HIGH;

	// Instantiate the Unit Under Test (UUT)
	nf10_upb_packet_fifo #(
		.DATA_WIDTH(32),
		.METADATA_WIDTH(8),
		.DATA_DEPTH(3), // 8 entries
		.METADATA_DEPTH(2), // 4 entries
		.LOW_THRESHOLD(3),
		.HIGH_THRESHOLD(5)
	) uut (
		.CLK(CLK), 
		.DI(DI), 
		.MI(MI), 
		.RDEN(RDEN), 
		.WREN(WREN), 
		.COMMIT(COMMIT), 
		.REVERT(REVERT), 
		.RST(RST), 
		.DO(DO), 
		.MO(MO), 
		.EOP(EOP), 
		.FULL(FULL), 
		.EMPTY(EMPTY), 
		.RDERR(RDERR), 
		.WRERR(WRERR),
		.BELOW_LOW(BELOW_LOW),
		.ABOVE_HIGH(ABOVE_HIGH)
	);

	initial begin
		// Initialize Inputs
		CLK = 1;
		DI = 0;
		MI = 0;
		RDEN = 0;
		WREN = 0;
		COMMIT = 0;
		REVERT = 0;
		RST = 0;

		// Wait 100 ns for global reset to finish
		#100;

		@(posedge CLK);
		DI <= 32'hdeadbeef;
		WREN <= 1;
		@(posedge CLK);
		WREN <= 0;
		@(posedge CLK);
		WREN <= 1;
		DI <= 32'haffedeaf;
		@(posedge CLK);
		DI <= 32'hdeadbeef;
		@(posedge CLK);
		MI <= 8'h55;
		DI <= 32'haffedeaf;
		COMMIT <= 1;
		@(posedge CLK);
		WREN <= 0;
		COMMIT <= 0;
		DI <= 0;
		MI <= 0;
		#10;
		
		@(posedge CLK);
		RDEN <= 1;
		#20;
		@(posedge CLK);
		RDEN <= 0;
		#20;
		
		@(posedge CLK);
		MI <= 55;
		WREN <= 1;
		DI <= 32'h01;
		@(posedge CLK);
		DI <= 32'h02;
		@(posedge CLK);
		DI <= 32'h03;
		@(posedge CLK);
		DI <= 32'h04;
		@(posedge CLK);
		DI <= 32'h05;
		@(posedge CLK);
		DI <= 32'h06;
		@(posedge CLK);
		DI <= 32'h07;
		@(posedge CLK);
		DI <= 32'h08;
		@(posedge CLK);
		DI <= 32'h09;
		@(posedge CLK);
		DI <= 32'h0a;
		@(posedge CLK);
		DI <= 32'h0b;
		REVERT <= 1;
		@(posedge CLK);
		WREN <= 0;
		REVERT <= 0;
		
		/* @(posedge CLK);
		RDEN <= 1; // Always read...
		DI <= 32'h01;
		MI <= 8'h01;
		COMMIT <= 1;
		WREN <= 1;
		@(posedge CLK);
		COMMIT <= 0;
		DI <= 32'h11;
		MI <= 8'hff; // should never be propagated
		@(posedge CLK);
		DI <= 32'h12;
		MI <= 8'h11;
		COMMIT <= 1;
		@(posedge CLK);
		COMMIT <= 0;
		MI <= 8'hff; // should never be propagated
		DI <= 32'h21;
		@(posedge CLK);
		DI <= 32'h22;
		@(posedge CLK);
		DI <= 32'h23;
		MI <= 8'h21;
		COMMIT <= 1;
		@(posedge CLK);
		COMMIT <= 0;
		MI <= 8'hff; // should never be propagated
		DI <= 32'hffffffff; // should never be propagated
		WREN <= 0; */
		
		#10
		@(posedge CLK);
		RST <= 1;
		@(posedge CLK);
		RST <= 0;
		
		#100 $finish();
	end
	
	always
		#1 CLK <= ~CLK;
      
endmodule

