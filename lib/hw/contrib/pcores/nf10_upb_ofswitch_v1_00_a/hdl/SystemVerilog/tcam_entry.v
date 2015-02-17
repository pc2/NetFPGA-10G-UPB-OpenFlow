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

module upb_tcam_entry #(
	parameter SRL_SIZE = 32,    // use 32 here to generate SRLC32E instances
	parameter TCAM_WIDTH = 49,  // number of SRLs in parallel, so total width is log2(SRL_SIZE) * TCAM_WIDTH
	parameter INPUT_WIDTH = 32, // input width for feeding data in
	parameter FORCE_MUXCY = 0   // force usage of carry chain on current Xilinx devices
)(
	input wire CLK,
	input wire RST,
	
	input wire [$clog2(SRL_SIZE)*TCAM_WIDTH-1:0] content,
	input wire [INPUT_WIDTH-1:0] wdata,
	input wire [$clog2(TCAM_WIDTH/INPUT_WIDTH)-1:0] waddr,
	input wire wen,
	
	output logic match
);

	logic [SRL_SIZE-1:0] srl [TCAM_WIDTH-1:0] /* synthesis syn_srlstyle = "select_srl" */;
	
	generate
		for (genvar i = 0; i < TCAM_WIDTH; i++) begin // write data into SRLs
			always_ff @(posedge CLK) begin
				if (wen == 1 && i < INPUT_WIDTH*(waddr+1) && i >= INPUT_WIDTH*waddr) begin
					srl[i] <= {srl[i][SRL_SIZE-2:0], wdata[i-(waddr*INPUT_WIDTH)]};
				end
			end
		end
	endgenerate

	/* We want to use the MUXCYs in the carry chain to generate our match signal if they
	   are available. If the necessary MUXCYs are not inferred automatically we have to
	   do this on our own. */
	
	generate
		if (FORCE_MUXCY) begin
			logic [TCAM_WIDTH-1:0] mux_o;
			assign match = mux_o[TCAM_WIDTH-1];
			
			MUXCY muxcy0 (
				.O(mux_o[0]),
				.CI(1),
				.DI(0),
				.S(srl[0][content[$clog2(SRL_SIZE)-1-:$clog2(SRL_SIZE)]])
			);
			for (genvar i = 1; i < TCAM_WIDTH; i++) begin
				MUXCY muxcys (
					.O(mux_o[i]),
					.CI(mux_o[i-1]),
					.DI(0),
					.S(srl[i][content[(i+1)*$clog2(SRL_SIZE)-1-:$clog2(SRL_SIZE)]])
				);
			end
		end else begin	
			always_comb begin
				match = 1;
				for (integer i = 0; i < TCAM_WIDTH; i++) begin
					match = match & srl[i][content[(i+1)*$clog2(SRL_SIZE)-1-:$clog2(SRL_SIZE)]];
				end
			end
		end	
	endgenerate

endmodule
