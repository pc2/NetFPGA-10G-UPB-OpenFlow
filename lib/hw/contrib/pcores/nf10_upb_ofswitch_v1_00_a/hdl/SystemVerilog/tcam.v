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

`default_nettype none

module upb_tcam #(
	parameter SRL_SIZE = 32,    // use 32 here to generate SRLC32E instances
	parameter TCAM_WIDTH = 49,  // number of SRLs in parallel, so total width is log2(SRL_SIZE) * TCAM_WIDTH (max. 128)
	parameter DELAY_MATCH = 1,  // minimum: 1
	parameter DELAY_LOOKUP = 1,  // minimum: 1
	parameter DELAY_WRITE = 1, // minimum: 1
	parameter FORCE_MUXCY = 0,  // force usage of carry chain on current Xilinx devices
	parameter TCAM_DEPTH = 64,   // number of entries (max. 1024)
	parameter DATA_WIDTH = 16  // width of stored data (max. 32)
)(
	input wire CLK,
	input wire RST,
	
	input wire [31:0] wdata,
	input wire [31:0] waddr,
	input wire wen,
	
	input wire [31:0] raddr,
	input wire rden,
	output logic [31:0] rdata,
	
	input wire [$clog2(SRL_SIZE)*TCAM_WIDTH-1:0] content, // this is a tCam, so content is what we search for
	output logic [DATA_WIDTH-1:0] data_out, // this is the data stored behind the content
	output logic match_out,
	output logic [$clog2(TCAM_DEPTH)-1:0] match_addr
	
);

	/*
	  Address map:
	  0x000000:             module name              (readonly)
	  0x000001:             module version           (readonly)
	  0x000002:             number of TCAM entries   (readonly)
	  0x001000 - 0x001FFF:  SRL entries (4 addresses each) (writeonly)
	  0x002000 - 0x0023FF:  data entries (writeonly)
	  0x003000 - 0x003019:  active bits (1 address for 32 entries) (writeonly)
	*/
	
	
	
	// instanciate block ram for actual data
	logic [DATA_WIDTH-1:0] data [TCAM_DEPTH-1:0];
	
	// Signals for handling wen, active and match
	logic [$clog2(TCAM_DEPTH+1)-1:0] matched_entry; // one extra valid value for "no match"
	logic [TCAM_DEPTH-1:0] match, tcam_wen;
	logic [TCAM_DEPTH-1:0] active;
	
	// Signals that are delayed for improved timing
	// Forced to be synthesized to registers because SRLs won't help us much...
	logic [$clog2(SRL_SIZE)*TCAM_WIDTH-1:0] content_reg [DELAY_LOOKUP-1:0] /* synthesis syn_srlstyle = "registers" */;
	logic [TCAM_DEPTH-1:0] match_reg [DELAY_MATCH-1:0] /* synthesis syn_srlstyle = "registers" */;
	logic [31:0] wdata_reg [DELAY_WRITE-1:0] /* synthesis syn_srlstyle = "registers" */;
	logic [31:0] waddr_reg [DELAY_WRITE-1:0] /* synthesis syn_srlstyle = "registers" */;
	logic [TCAM_DEPTH-1:0] active_reg [DELAY_MATCH-1:0] /* synthesis syn_srlstyle = "registers" */;
	logic [DELAY_WRITE-1:0] wen_reg /* synthesis syn_srlstyle = "registers" */;
	
	wire [$clog2(SRL_SIZE)*TCAM_WIDTH-1:0] content_d = content_reg[DELAY_LOOKUP-1];
	wire [TCAM_DEPTH-1:0] match_d = match_reg[DELAY_MATCH-1];
	wire [31:0] wdata_d = wdata_reg[DELAY_WRITE-1];
	wire [31:0] waddr_d = waddr_reg[DELAY_WRITE-1];
	wire [TCAM_DEPTH-1:0] active_d = active_reg[DELAY_MATCH-1];
	wire wen_d = wen_reg[DELAY_WRITE-1];
	
	
	
	
	// Delaying of signals
	always_ff @(posedge CLK) begin
		content_reg[0] <= content;
		match_reg[0] <= match;
		wdata_reg[0] <= wdata;
		waddr_reg[0] <= waddr;
		active_reg[0] <= active;
		wen_reg[0] <= wen;
		
		if (RST) begin
			wen_reg[0] <= 0;
			active_reg[0] <= {(TCAM_DEPTH){1'b0}};
		end
	end
	generate
		for (genvar i = 1; i < DELAY_LOOKUP; i++) begin
			always_ff @(posedge CLK) begin
				content_reg[i] <= content_reg[i-1];
			end
		end
	endgenerate
	generate
		for (genvar i = 1; i < DELAY_MATCH; i++) begin
			always_ff @(posedge CLK) begin
				match_reg[i] <= match_reg[i-1];
				active_reg[i] <= active_reg[i-1];
				if (RST)
					active_reg[i] <= {(TCAM_DEPTH){1'b0}};
			end
		end
	endgenerate
	generate
		for (genvar i = 1; i < DELAY_WRITE; i++) begin
			always_ff @(posedge CLK) begin
				wdata_reg[i] <= wdata_reg[i-1];
				waddr_reg[i] <= waddr_reg[i-1];
				wen_reg[i] <= wen_reg[i-1];
				
				if (RST)
					wen_reg[i] <= 0;
			end
		end
	endgenerate
	
	
	
	
	// Read functionality, very very basic :)
	always_ff @(posedge CLK) begin
		if (rden) begin
			case (raddr)
				32'h0:
					rdata <= "TCAM";
				32'h1:
					rdata <= 32'd1;
				32'h2:
					rdata <= TCAM_DEPTH;
			endcase
		end
	end
	
	
	
	
	// Write functionality is divided into three different address blocks (see above)
	
	// write to block ram
	always_ff @(posedge CLK) begin
		if (wen_d && waddr_d>>12 == 20'h2) begin
			data[waddr_d[9:0]] <= wdata_d;
		end
	end
	
	// write to bitstring that marks active entries
	always_ff @(posedge CLK) begin
		if (wen_d && waddr_d>>12 == 20'h3) begin
			active[(waddr_d[7:0]+1)*32-1-:32] <= wdata_d;
		end
		
		if (RST)
			active <= {(TCAM_DEPTH){1'b0}};
	end

	// instantiate entries
	// write commends are delegated to each sigle tcam entry
	generate
		for (genvar i = 0; i < TCAM_DEPTH; i++) begin
			assign tcam_wen[i] = (wen_d && waddr_d>>12 == 20'h1 && waddr_d[11:2] == i);
			upb_tcam_entry #(
				.SRL_SIZE(SRL_SIZE),
				.TCAM_WIDTH(TCAM_WIDTH),
				.INPUT_WIDTH(32),
				.FORCE_MUXCY(FORCE_MUXCY)
			) tcam_entries (
				.CLK(CLK),
				.RST(RST),
				.content(content_d),
				.wdata(wdata_d),
				.waddr(waddr_d[1:0]),
				.wen(tcam_wen[i]),
				.match(match[i])
			);
		end
	endgenerate
	
	
	
	
	// Match generation
	// Priority encoder: choose first match (+1 because 0 means no match)
	always_comb begin
		matched_entry = 0;
		for (int i = 0; i < TCAM_DEPTH; i++) begin
			if (match_d[i] && active_d[i]) begin
				matched_entry = i+1;
				break;
			end
		end
	end
	
	// look up data
	always_ff @(posedge CLK) begin
		if (matched_entry != 0) begin
			data_out <= data[matched_entry-1];
			match_addr <= matched_entry-1;
		end
		match_out <= matched_entry != 0;
		
		if (RST)
			match_out <= 0;
	end

endmodule
