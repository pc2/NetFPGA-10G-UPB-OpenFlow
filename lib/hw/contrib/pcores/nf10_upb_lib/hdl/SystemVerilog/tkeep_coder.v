/*
 * UPB AXI Stream TKEEP encoder/decoder
 *
 * Copyright (c) 2014, 2015 Jörg Niklas
 * osjsn@niklasfamily.de
 *
 * This file is part of the NetFPGA 10G UPB OpenFlow Switch project.
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
 */

`default_nettype none

package tkeep_coder;

	function int unsigned get_tkeep_encoded_width(int unsigned tkeep_width);
		
		get_tkeep_encoded_width = $clog2(tkeep_width);
		
	endfunction

endpackage

module tkeep_encoder #(

	parameter tkeep_width						= 32,
	parameter tkeep_encoded_width				= tkeep_coder::get_tkeep_encoded_width(tkeep_width)
	
) (
	
	input wire [tkeep_width-1:0]				tkeep,
	output logic [tkeep_encoded_width-1:0]	tkeep_encoded

);

always_comb begin
	tkeep_encoded <= '1; // default: all bytes valid
	for (int unsigned i = 1; i < tkeep_width; i++) begin
		if (tkeep[i] == 1'b0) begin
			tkeep_encoded <= i-1;
			break;
		end
	end
end

endmodule


module tkeep_decoder #(

	parameter tkeep_width						= 32,
	parameter tkeep_encoded_width				= tkeep_coder::get_tkeep_encoded_width(tkeep_width)
	
) (
	
	input wire [tkeep_encoded_width-1:0]	tkeep_encoded,
	output logic [tkeep_width-1:0]			tkeep

);

always_comb begin
	for (int unsigned i = 0; i < tkeep_width; i++) begin
		tkeep[i] <= (i <= tkeep_encoded ? 1'b1 : 1'b0);
	end
end

endmodule
