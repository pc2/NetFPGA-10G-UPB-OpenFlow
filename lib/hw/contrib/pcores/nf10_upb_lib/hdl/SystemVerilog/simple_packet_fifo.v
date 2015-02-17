/*
*    UPB Simple Packet FIFO Core
*
*    Copyright (c) 2014, 2015 Jörg Niklas
*    osjsn@niklasfamily.de
*
*    This file is part of the NetFPGA 10G UPB OpenFlow Switch project:
*
*    Project Group "On-the-Fly Networking for Big Data"
*    SFB 901 "On-The-Fly Computing"
*
*    University of Paderborn
*    Computer Engineering Group
*    Pohlweg 47 - 49
*    33098 Paderborn
*    Germany
*
*   
*    This file is free code: you can redistribute it and/or modify it under
*    the terms of the GNU Lesser General Public License version 2.1 as
*    published by the Free Software Foundation.
*
*    This file is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU Lesser General Public License for more details.
*
*    You should have received a copy of the GNU Lesser General Public License
*    along with this project.  If not, see <http://www.gnu.org/licenses/>.
*/

`default_nettype none

module simple_packet_fifo #(

	parameter data_width 								= 256,
	parameter metadata_width 							= 32,
	parameter store_and_forward						= 1,
	parameter almost_full_offset						= 6,
	parameter almost_full_offset_metadata_fifo	= almost_full_offset / 2 // 2 data beats = minimum packet size
	
) (

	input wire										wr_clk,
	input wire										wr_reset,
	input wire [data_width-1:0]				wr_data,
	input wire [metadata_width-1:0]			wr_metadata,
	input wire										wr_last,
	input wire										wr_dirty,
	input wire										wr,
	output logic									wr_full,
	output logic									wr_almost_full,
	
	input wire										rd_clk,
	input wire										rd_reset,
	output logic [data_width-1:0]				rd_data,
	output logic [metadata_width-1:0]		rd_metadata,
	output logic 									rd_last,
	input wire										rd,
	output logic 									rd_empty,
	
	output logic									error

);

localparam data_fifo_count = (data_width + 1) / 72 + ((data_width + 1) % 72 ? 1 : 0); // data_width + 1: "data" and "last"
localparam metadata_fifo_count = (metadata_width + 1) / 36 + (metadata_width % 36 ? 1 : 0); // metadata_width + 1: wr_metadata + wr_dirty

// write logic
wire [data_fifo_count-1:0][71:0] wr_data_fifo = {wr_data, wr_last};
wire [metadata_fifo_count-1:0][35:0] wr_metadata_fifo = {wr_metadata, wr_dirty};

wire [data_fifo_count-1:0] wr_full_fifo, wr_almost_full_fifo;
wire [metadata_fifo_count-1:0] wr_full_metadata_fifo, wr_almost_full_metadata_fifo;
assign wr_full = (| wr_full_fifo) | (| wr_full_metadata_fifo);
assign wr_almost_full = (| wr_almost_full_fifo) | (| wr_almost_full_metadata_fifo);

// read logic
wire [data_fifo_count-1:0][71:0] rd_data_fifo;
wire [data_fifo_count-1:0][35:0] rd_metadata_fifo;
wire rd_dirty;
assign {rd_metadata, rd_dirty} = rd_metadata_fifo;

wire rd_last_internal;
assign rd_last = rd_last_internal;
assign {rd_data, rd_last_internal} = rd_data_fifo;

wire [data_fifo_count-1:0] rd_empty_fifo;
wire [metadata_fifo_count-1:0] rd_empty_metadata_fifo;
wire fifos_are_empty = (| rd_empty_fifo) | (| rd_empty_metadata_fifo);
assign rd_empty = fifos_are_empty | rd_dirty; // if any fifo is empty or the dirty bit is set we cannot read data

// error detection
wire [data_fifo_count-1:0] wr_err_fifo, rd_err_fifo;
wire [metadata_fifo_count-1:0] wr_err_metadata_fifo, rd_err_metadata_fifo;
assign error = (| wr_err_fifo) | (| rd_err_fifo) | (| wr_err_metadata_fifo) | (| rd_err_metadata_fifo);

// read logic: destroy data if dirty bit is set
wire rd_fifos = rd | (~fifos_are_empty & rd_dirty);

// data FIFOs
generate

	for (genvar i = 0; i < data_fifo_count; i++) begin
	
		FIFO36_72 #(
			.SIM_MODE("FAST"), // Simulation: "SAFE" vs. "FAST", see "Synthesis and Simulation Design Guide" for details
			.ALMOST_FULL_OFFSET(almost_full_offset), // Sets almost full threshold
			.ALMOST_EMPTY_OFFSET(9'h080), // Sets the almost empty threshold
			.DO_REG(1), // Enable output register (0 or 1), must be 1 if EN_SYN = "FALSE"
			.EN_ECC_READ("FALSE"), // Enable ECC decoder, "TRUE" or "FALSE"
			.EN_ECC_WRITE("FALSE"), // Enable ECC encoder, "TRUE" or "FALSE"
			.EN_SYN("FALSE"), // Specifies FIFO as Asynchronous ("FALSE") or Synchronous ("TRUE")
			.FIRST_WORD_FALL_THROUGH("TRUE") // Sets the FIFO FWFT to "TRUE" or "FALSE"
			
		) data_fifo (
		
			.RST(wr_reset | rd_reset),
		
			.WRCLK(wr_clk),
			.DI(wr_data_fifo[i][63:0]),	// 64-bit data input
			.DIP(wr_data_fifo[i][71:64]),		// 8-bit parity input
			.WREN(wr),
			.FULL(wr_full_fifo[i]),
			.ALMOSTFULL(wr_almost_full_fifo[i]),
		
			.RDCLK(rd_clk),
			.DO(rd_data_fifo[i][63:0]),	// 64-bit data output
			.DOP(rd_data_fifo[i][71:64]),		// 8-bit parity data output
			.RDEN(rd_fifos),
			.EMPTY(rd_empty_fifo[i]),
			
			.RDERR(rd_err_fifo[i]),
			.WRERR(wr_err_fifo[i])
			
		);
	
	end


endgenerate

// logic for detecting the first word of the packet (signal "first")
logic wr_first = '1;

always_ff @(posedge wr_clk) begin

	if (wr_reset) begin
		wr_first <= '1;
	end else begin
		if (wr) begin
			wr_first <= wr_last; // if "last" is set, then the "first" word is on the next wr_clk cycle
		end
	end

end

// metadata FIFOs

generate

	for (genvar i = 0; i < metadata_fifo_count; i++) begin
	
		FIFO18_36 #(
			.SIM_MODE("FAST"), // Simulation: "SAFE" vs. "FAST", see "Synthesis and Simulation Design Guide" for details
			.ALMOST_FULL_OFFSET(almost_full_offset_metadata_fifo), // Sets almost full threshold
			.ALMOST_EMPTY_OFFSET(9'h080), // Sets the almost empty threshold
			.DO_REG(1), // Enable output register (0 or 1), must be 1 if EN_SYN = "FALSE"
			.EN_SYN("FALSE"), // Specifies FIFO as Asynchronous ("FALSE") or Synchronous ("TRUE")
			.FIRST_WORD_FALL_THROUGH("TRUE") // Sets the FIFO FWFT to "TRUE" or "FALSE"
			
		) metadata_fifo (
		
			.RST(wr_reset | rd_reset),
		
			.WRCLK(wr_clk),
			.DI(wr_metadata_fifo[i][31:0]),	// 32-bit data input
			.DIP(wr_metadata_fifo[i][35:32]),	// 4-bit parity input
			.WREN(store_and_forward ? wr & wr_last : wr & wr_first),
			.FULL(wr_full_metadata_fifo[i]),
			.ALMOSTFULL(wr_almost_full_metadata_fifo[i]),
		
			.RDCLK(rd_clk),
			.DO(rd_metadata_fifo[i][31:0]),		// 32-bit data output
			.DOP(rd_metadata_fifo[i][35:32]),	// 4-bit parity data output
			.RDEN(rd_fifos & rd_last_internal),	// pop metadata FIFO if ethernet packet's last word from data fifo was read
			.EMPTY(rd_empty_metadata_fifo[i]),
			
			.RDERR(rd_err_metadata_fifo[i]),
			.WRERR(wr_err_metadata_fifo[i])
			
		);
	
	end


endgenerate


endmodule
