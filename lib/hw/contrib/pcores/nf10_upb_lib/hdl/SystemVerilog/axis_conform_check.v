/*
 * UPB AXI Stream conformance check core
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

package axis_conform_check_pkg;

	typedef struct packed {
	
		logic packet_too_small;
		logic packet_too_big;
		logic packet_tkeep_not_continuous;
		logic packet_tkeep_encoded_wrong;
		logic packet_tvalid_deasserted;
		
	} axi_errors_t;

endpackage

module axis_conform_check #(

	parameter tkeep_encoded						= 0,
	parameter axis_data_width					= 256,
	parameter axis_tkeep_width					= axis_data_width / 8,		// 32 bit
	parameter axis_tkeep_encoded_width			= $clog2(axis_tkeep_width),	// 5 bit
	parameter max_packet_size_counter			= 65535,
	parameter min_packet_size					= 33,		// in bytes
	parameter max_packet_size					= 9500		// in bytes

) (
	input wire									clk,
	input wire									reset,

	input wire [axis_tkeep_width-1:0]			axis_tkeep,
	input wire [axis_tkeep_encoded_width-1:0]	axis_tkeep_encoded,
	input wire									axis_tlast,
	input wire 									axis_tvalid,
	input wire									axis_tready,

	output reg									error,
	output axis_conform_check_pkg::axi_errors_t	error_vec
);

reg [$clog2(max_packet_size_counter+1)-1:0] packet_length;
reg [axis_tkeep_encoded_width:0] axis_tkeep_encoded_internal; // one bit wider than axis_tkeep_encoded (we count from 0 to 32)

generate

	if (tkeep_encoded) begin

		assign axis_tkeep_encoded_internal = axis_tkeep_encoded + 1;

	end else begin

		always_comb begin
			axis_tkeep_encoded_internal = 0;
			for (int i = 0; i < axis_tkeep_width; i++) begin
				if (axis_tkeep[i] == 1'b1)
					axis_tkeep_encoded_internal++;
			end
		end
	end

endgenerate

assign error = |error_vec;

logic last_axis_tvalid, last_axis_tready, last_reset;

always_ff @(posedge clk) begin
	
	error_vec <= '0;

	last_axis_tvalid <= axis_tvalid;
	last_axis_tready <= axis_tready;
	last_reset <= reset;

	if (reset) begin

		packet_length <= 0;

	end else begin

		if (axis_tvalid & axis_tready) begin
		
			if (~axis_tlast) begin

				if (packet_length < max_packet_size_counter - axis_tkeep_width)
					packet_length <= packet_length + axis_tkeep_encoded_internal;

				if (axis_tkeep_encoded_internal != axis_tkeep_width)
					error_vec.packet_tkeep_not_continuous <= 1'b1;

			end else begin

				if (packet_length + axis_tkeep_encoded_internal < min_packet_size) begin
					error_vec.packet_too_small <= 1'b1;
				end
			
				if (packet_length + axis_tkeep_encoded_internal > max_packet_size) begin
					error_vec.packet_too_big <= 1'b1;
				end

				packet_length <= 0;
			end

			if (tkeep_encoded == 0) begin

				for (int i = 0; i < axis_tkeep_width; i++) begin
					if (axis_tkeep[i] == 1'b0) begin // found a zero => only zeros may follow
						for (int j = i+1; j < axis_tkeep_width; j++) begin
							if (axis_tkeep[j] == 1'b1)
								error_vec.packet_tkeep_encoded_wrong <= 1'b1;
						end
						break;
					end
				end

			end

			if (~last_reset & ~axis_tvalid & last_axis_tvalid & ~last_axis_tready) begin
				error_vec.packet_tvalid_deasserted <= 1'b1;
			end

		end
	end

end

endmodule

