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

module nf10_upb_packet_fifo #(
	parameter DATA_WIDTH = 256,
	parameter METADATA_WIDTH = 16,
	parameter DATA_DEPTH = 10,     // 2^n entries
	parameter METADATA_DEPTH = 10, // 2^m entries
	parameter LOW_THRESHOLD = 256,  // set BELOW_LOW when less than p entries in data bram
	parameter HIGH_THRESHOLD = 768  // set ABOVE_HIGH when more than q entries in data bram
) (
	input CLK,
	input [DATA_WIDTH-1:0] DI,
	input [METADATA_WIDTH-1:0] MI,
	input RDEN,
	input WREN,
	input COMMIT,
	input REVERT,
	input RST,
	output [DATA_WIDTH-1:0] DO,
	output [METADATA_WIDTH-1:0] MO,
	output EOP,
	output FULL,
	output reg EMPTY = 1,
	output BELOW_LOW,
	output ABOVE_HIGH,
	output reg RDERR = 0,
	output reg WRERR = 0
);

	localparam data_entries = 2 ** DATA_DEPTH;
	localparam meta_entries = 2 ** METADATA_DEPTH;

	// BRAM
	reg [DATA_WIDTH:0] data [data_entries-1:0]; // +1 bit width for EOP
	reg [METADATA_WIDTH-1:0] metadata [meta_entries-1:0];
	
	// Pointer
	reg [DATA_DEPTH-1:0] data_ptr_wr = 0;
	reg [DATA_DEPTH-1:0] data_ptr_rd = 0;
	reg [DATA_DEPTH-1:0] data_ptr_last_packet = 0;
	reg [METADATA_DEPTH-1:0] meta_ptr_wr = 0;
	reg [METADATA_DEPTH-1:0] meta_ptr_rd = 0;
	
	// Writing to FIFO
	wire [DATA_DEPTH-1:0] cur_data_entries = data_ptr_wr - data_ptr_rd;
	wire [DATA_DEPTH-1:0] data_ptr_distance = data_ptr_rd - data_ptr_wr;
	wire [METADATA_DEPTH-1:0] meta_ptr_distance = meta_ptr_rd - meta_ptr_wr;
	assign FULL = (data_ptr_distance == 1 || meta_ptr_distance == 1); // Waisting one entry here
	assign BELOW_LOW = (cur_data_entries < LOW_THRESHOLD);
	assign ABOVE_HIGH = (cur_data_entries > HIGH_THRESHOLD);
	
	always @(posedge CLK) begin
		WRERR <= 0;
		
		if ((REVERT && COMMIT) || (WREN && FULL && !REVERT))
			WRERR <= 1;
		else if (REVERT)
			data_ptr_wr <= data_ptr_last_packet;
		else if (WREN) begin
			data[data_ptr_wr] <= {COMMIT, DI};
			data_ptr_wr <= data_ptr_wr + 1;
			if (COMMIT) begin
				metadata[meta_ptr_wr] <= MI;
				data_ptr_last_packet <= data_ptr_wr + 1;
				meta_ptr_wr <= meta_ptr_wr + 1;
			end
		end
		
		if (RST) begin
			data_ptr_wr <= 0;
			data_ptr_last_packet <= 0;
			meta_ptr_wr <= 0;
		end
	end
	
	// Reading from FIFO
	reg meta_empty_d1 = 1;
	wire meta_empty = (meta_ptr_distance == 0);
	wire performing_read = (RDEN && !EMPTY); // only valid on rising CLK edge
	wire fall_through = (EMPTY && !meta_empty_d1);
	reg [DATA_WIDTH:0] data_reg = {1'b1, {(DATA_WIDTH){1'b0}}};
	reg [METADATA_WIDTH-1:0] meta_reg = 0;
	assign DO = data_reg[DATA_WIDTH-1:0];
	assign EOP = data_reg[DATA_WIDTH];
	assign MO = meta_reg;
	
	always @(posedge CLK) begin
		RDERR <= 0;
		meta_empty_d1 <= meta_empty;
		
		if (RDEN && EMPTY)
			RDERR <= 1;
		
		if (performing_read || fall_through) begin //something to do
		
			if (!EOP || !meta_empty_d1) begin //data for current packet or another packet available
				data_ptr_rd <= data_ptr_rd + 1;
				data_reg <= data[data_ptr_rd];
			end
			
			if (EOP || fall_through) begin //we need a new packet
				EMPTY <= meta_empty_d1;
				if (!meta_empty_d1) begin
					meta_ptr_rd <= meta_ptr_rd + 1;
					meta_reg <= metadata[meta_ptr_rd];
				end
			end
			
		end
		
		if (RST) begin
			data_ptr_rd <= 0;
			meta_ptr_rd <= 0;
			EMPTY <= 1;
			data_reg <= {1'b1, {(DATA_WIDTH){1'b0}}};
			meta_reg <= 0;
			meta_empty_d1 <= 1;
		end
	end
	
endmodule
