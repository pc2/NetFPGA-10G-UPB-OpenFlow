/*
 * UPB SECDED Core
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

package secded;

	// returns number of needed parity bits for a given memory width
	function int unsigned get_parity_bits_from_memory_width(int unsigned memory_bits); // there is an extra parity bit (overall parity) in the memory_bits...
	
		get_parity_bits_from_memory_width = $clog2(memory_bits) + 1; // ...so no clog2(input_bits+1) here! But: Additional + 1 for the overall parity bit itself!
	
	endfunction

endpackage

module secded_generator #(

	parameter output_bits					= 288,									
	parameter hamming_code_parity_bits	= secded::get_parity_bits_from_memory_width(output_bits) - 1, // only the parity bits for the hamming code (not including the overall parity bit)
	parameter input_bits						= output_bits - secded::get_parity_bits_from_memory_width(output_bits)

) (

	input wire [input_bits-1:0]		data_in,
	output logic [output_bits-1:0]	data_out

);

	logic [output_bits-1:0] data_out_internal;
	assign data_out = data_out_internal;

	always_comb begin
	
		// 1st step: forward the data bits to the new output position
		automatic int unsigned input_bit = 0;
		automatic int unsigned parity_bit = 1;
		
		for (int unsigned i = 1; i < output_bits; i++) begin // the last output bit is reserved for the overall parity; [0,286] is used for parity calculation
		
				if (i == parity_bit) begin
					parity_bit = parity_bit << 1;
				end else begin
					data_out_internal[i-1] = data_in[input_bit++];
				end
		end
		
		// 2nd step: calculate the parity bits
		parity_bit = 1;
		
		for (int unsigned i = 1; i < output_bits; i++) begin // the last output bit is reserved for the overall parity; [0,286] is used for parity calculation
		
				if (i == parity_bit) begin
				
					// i is a parity bit position (only 1 bit set in i)
				
					automatic bit found_first = 0;
					automatic bit xor_value;
				
					for (int unsigned j = i+1; j < output_bits; j++) begin
					
						if (j & parity_bit) begin
						
							if (found_first) begin
								xor_value = xor_value ^ data_out_internal[j-1];
							end else begin
								xor_value = data_out_internal[j-1];
								found_first = 1;
							end
						end
						
					end
					
					data_out_internal[i-1] = xor_value;
				
					parity_bit = parity_bit << 1;
				end
		end
		
		// 3rd step: calculate the overall parity bit
		begin
			automatic bit xor_value = data_out_internal[0];
			
			for (int unsigned i = 1; i < output_bits-1; i++) begin // the last output bit is reserved for the overall parity; [0,286] is used for parity calculation
				xor_value = xor_value ^ data_out_internal[i];
			end
			
			data_out_internal[output_bits-1] = xor_value;
		end
	
	end

endmodule

module secded_checker #(

	parameter input_bits						= 288,
	parameter hamming_code_parity_bits	= secded::get_parity_bits_from_memory_width(input_bits) - 1, // only the parity bits for the hamming code (not including the overall parity bit), 					// ...so no clog2(input_bits+1) here!
	parameter output_bits					= input_bits - secded::get_parity_bits_from_memory_width(input_bits)
	
) (

	input wire [input_bits-1:0]						data_in,
	output logic [output_bits-1:0]					data_out,
	
	output logic [hamming_code_parity_bits-1:0]	error_syndrome,
	output logic											overall_parity_error
	


);

	always_comb begin
		
		// 1nd step: check the parity bits (calculate the error syndrome) and forward data bits to the output
		
		automatic int unsigned parity_bit = 1, error_syndrome_pos = 0, output_bit = 0;
		
		for (int unsigned i = 1; i < input_bits; i++) begin // the last input bit is reserved for the overall parity; [0,286] is used for parity calculation
		
				if (i == parity_bit) begin
				
					// i is a parity bit position (only 1 bit set in i)
				
					automatic bit xor_value = data_in[i-1];
				
					for (int unsigned j = i+1; j < input_bits; j++) begin
					
						if (j & parity_bit) begin

							xor_value = xor_value ^ data_in[j-1];
								
						end
						
					end
					
					error_syndrome[error_syndrome_pos++] = xor_value;
				
					parity_bit = parity_bit << 1;
					
				end else begin
				
					// i is a data bit position => forward to data_out
					
					data_out[output_bit++] = data_in[i-1];
				
				end
		end
		
		// 2nd step: check the overall parity bit
		begin
			automatic bit xor_value = data_in[0];
			
			for (int unsigned i = 1; i < input_bits; i++) begin
				xor_value = xor_value ^ data_in[i];
			end
			
			overall_parity_error = xor_value;

		end
	
	end

endmodule

module secded_corrector #(

	parameter data_bits							= 278,
	parameter hamming_code_parity_bits		= 9

) (

	input wire [data_bits-1:0]							data_in,
	output logic [data_bits-1:0]						data_out,
		
	input wire [hamming_code_parity_bits-1:0]		error_syndrome,
	input wire												overall_parity_error,
	
	output logic											corrected_error,
	output logic											uncorrectable_error

);

always_comb begin
		
	corrected_error = 1'b0;
	uncorrectable_error = 1'b0;
	
	data_out = data_in;
	
	if (error_syndrome != '0 || overall_parity_error != 1'b0) begin
	
		// there is an error
	
		if (overall_parity_error == 1'b1) begin
		
			// single bit error
			
			if (error_syndrome == '0) begin
			
				// overall parity bit error
				corrected_error = 1'b1;
			
			end else begin
			
				// correctable single bit error
				
				automatic int unsigned parity_bit = 1;
				automatic int unsigned output_bit = 0;
				
				corrected_error = 1'b1;
		
				for (int unsigned i = 1; i <= data_bits + hamming_code_parity_bits; i++) begin
				
					if (i == parity_bit) begin
					
						if (i == error_syndrome) begin
						
							// erroneous bit is a parity bit
							break;
						end
						
						parity_bit = parity_bit << 1;
						
					end else begin
				
						// i is a data bit position
						
						if (i == error_syndrome) begin
						
							// found the erroneous bit in the data
							data_out[output_bit] = ~data_in[output_bit]; // invert the bit
							
							break;
						
						end
						
						output_bit++;
					
					end
				
				end
			
			end
			
		end else begin
		
			// uncorrectable error (overall parity is 0 but error_syndrome shows error (!= 0))
			uncorrectable_error = 1'b1;
		
		end
		
	end

end

	
endmodule
