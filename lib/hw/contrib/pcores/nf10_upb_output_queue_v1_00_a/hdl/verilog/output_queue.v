/*
 * UPB Output Queue core
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

//`define use_chipscope_output_queue

`default_nettype none

module output_queue #(
	
	parameter output_ports 					= 7,
	
	parameter axis_data_width				= 256,
	parameter axis_tkeep_width				= axis_data_width / 8, // 32 bit
	
	parameter single_mem_data_width		= 36,
	parameter mem_addr_width				= 19,
	parameter IODELAY_GRP					= "IODELAY_GRP_QDR2",
	parameter read_delay_cycles			= 5,
	parameter qdr2_pll_lock_cycles		= 1024,
	
	parameter multicast_queue_size_256b	= 2**9, // 16KB

	parameter [mem_addr_width-1:0] memsize_per_port = 2**mem_addr_width / output_ports,
	parameter [output_ports-1:0][mem_addr_width-1:0] queue_sizes_256b = { output_ports{ memsize_per_port } }, // QuestaSim & Synplify workaround...
	parameter [output_ports-1:0] store_and_forward_ports = { 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1}, // the 4 lower ports are store & forward (10G MAC), all other ports can deassert the 'valid' during packet transfer
	
	parameter simulation_speed_up			= 0

) (

	input	wire									clk,
	input	wire									reset,
	
	input	wire									clk2x,
	input	wire									clk2x90,
	
	input wire [axis_data_width-1:0]		s_axis_tdata,
	input wire [axis_tkeep_width-1:0]	s_axis_tkeep,
	input axis_defs::tuser_t				s_axis_tuser,
	input wire									s_axis_tlast,
	input wire 									s_axis_tvalid,
	output logic								s_axis_tready,

	input logic									m_axis_clk[output_ports],
	input logic									m_axis_reset[output_ports],
	output logic [axis_data_width-1:0]	m_axis_tdata[output_ports],
	output logic [axis_tkeep_width-1:0]	m_axis_tkeep[output_ports],
	output axis_defs::tuser_t				m_axis_tuser[output_ports],
	output logic								m_axis_tlast[output_ports],
	output logic 								m_axis_tvalid[output_ports],
	input wire									m_axis_tready[output_ports],
	
	qdr2_sram_if.master						qdr[2],
	
	inout wire [35:0]							chipscope_0, chipscope_1, chipscope_2
	
);

logic s_axis_tready_internal;
assign s_axis_tready = s_axis_tready_internal;

/*
*	Encode/decode tkeep to save space in the buffers
*/

// encode incoming tkeep
localparam tkeep_encoded_width = tkeep_coder::get_tkeep_encoded_width(axis_tkeep_width); // 5 bits

logic [tkeep_encoded_width-1:0] s_axis_tkeep_encoded;

tkeep_encoder #(.tkeep_width(axis_tkeep_width)) tkeep_enc0 (
	.tkeep(s_axis_tkeep),
	.tkeep_encoded(s_axis_tkeep_encoded)
);

/*
* Output (Async)FIFOs
*/

localparam minimum_packet_size_cycles = 64*8 / axis_data_width + (64*8 % axis_data_width ? 1 : 0);

typedef struct packed {										// 287 bit

	logic [axis_data_width-1:0] tdata;					// 256 bit
	logic [axis_tkeep_width-2:0] tkeep_reduced;	// 31 bit

} output_data_t;

output_data_t [output_ports-1:0] output_fifo_data_in;

logic [output_ports-1:0] output_fifo_tlast_in;

typedef struct packed {										// 8 bit

	axis_defs::out_vport_t out_vport;					// 8 bit

} output_metadata_t;

output_metadata_t [output_ports-1:0] output_fifo_metadata_in;

logic [output_ports-1:0] output_fifo_wr, output_fifo_almost_full = '0, output_fifo_almost_full_early;

generate

	for (genvar i = 0; i < output_ports; i++) begin
	
		localparam almost_full_offset = 10; // TODO: Check! It should be 7...
		
		wire output_fifo_rd, output_fifo_empty;
		output_data_t output_fifo_data_out;
		output_metadata_t output_fifo_metadata_out;

		simple_packet_fifo #(
		
			.data_width									($bits(output_data_t)),
			.metadata_width							($bits(output_metadata_t)),
			.store_and_forward						(store_and_forward_ports[i]),
			.almost_full_offset						(almost_full_offset),
			.almost_full_offset_metadata_fifo	(almost_full_offset / minimum_packet_size_cycles)

		) output_fifo (
		
			.wr_clk				(clk),
			.wr_reset			(reset),
			.wr_data				(output_fifo_data_in[i]),
			.wr_metadata		(output_fifo_metadata_in[i]),
			.wr_last				(output_fifo_tlast_in[i]),
			.wr_dirty			(1'b0),
			.wr					(output_fifo_wr[i]),
			.wr_full				(),
			.wr_almost_full	(output_fifo_almost_full_early[i]),
			
			.rd_clk				(m_axis_clk[i]),
			.rd_reset			(m_axis_reset[i]),
			.rd_data				(output_fifo_data_out),
			.rd_metadata		(output_fifo_metadata_out),
			.rd_last				(m_axis_tlast[i]),
			.rd					(output_fifo_rd),
			.rd_empty			(output_fifo_empty),
			
			.error				()
		);
		
		assign m_axis_tdata[i] = output_fifo_data_out.tdata;
		assign m_axis_tkeep[i] = { output_fifo_data_out.tkeep_reduced, 1'b1 };
		
		// the output cores don't need most information from tuser
		assign m_axis_tuser[i].in.port = '0;
		assign m_axis_tuser[i].in.vport = '0;
		assign m_axis_tuser[i].out.port = '0;
		assign m_axis_tuser[i].out.vport = output_fifo_metadata_out.out_vport;
		assign m_axis_tuser[i].packet_length = '0;
		
		assign m_axis_tvalid[i] = ~output_fifo_empty;
		
		assign output_fifo_rd = ~output_fifo_empty & m_axis_tready[i];

	end
	
endgenerate

always_ff @(posedge clk) begin

	if (reset) begin
		output_fifo_almost_full <= '0;
	end else begin
		output_fifo_almost_full <= output_fifo_almost_full_early;
	end

end


/*
*	SRAM Memory logic
*/
localparam whole_mem_data_width = single_mem_data_width * 8; // 288 bit
localparam parity_bits = secded::get_parity_bits_from_memory_width(whole_mem_data_width); // 10 bit
localparam mem_data_width = whole_mem_data_width - parity_bits; // 278 bit


logic wr = 1'b0, rd = 1'b0;
logic [mem_addr_width-1:0] emem_r_addr, emem_w_addr;

typedef struct packed {										// 270 bit

	logic [axis_data_width-1:0] tdata;					// 256 bit
	logic [tkeep_encoded_width-1:0] tkeep_encoded;	// 5 bit
	logic tlast;												// 1 bit
	axis_defs::out_vport_t out_vport;					// 8 bit

} stream_data_t;

stream_data_t emem_r_data, emem_w_data;

assign emem_w_data = '{			

	tdata: s_axis_tdata,
	tkeep_encoded: s_axis_tkeep_encoded,
	tlast: s_axis_tlast,
	out_vport: s_axis_tuser.out.vport
	
};

wire emem_error, emem_ready;
wire [parity_bits-2:0] ecc_error_syndrome; // -2: $get_bits_from_memory_width returns the parity bits including the overall parity
wire ecc_overall_parity_error;

// SECDED generator/checker
logic [single_mem_data_width*8-1:0] emem_r_data_with_ecc, emem_w_data_with_ecc, emem_w_data_with_ecc_ff;

/*
secded_generator #(

	.output_bits				(whole_mem_data_width)
	
) secded_generator_0 (

	.data_in						(emem_w_data),
	.data_out					(emem_w_data_with_ecc)

);
*/

assign emem_w_data_with_ecc = emem_w_data;

always_ff @( posedge clk )
	emem_w_data_with_ecc_ff <= emem_w_data_with_ecc;

/*
secded_checker #(

	.input_bits					(whole_mem_data_width)
	
) secded_checker_0 (
	
	.data_in						(emem_r_data_with_ecc),
	.data_out					(emem_r_data),
	
	.error_syndrome			(ecc_error_syndrome),
	.overall_parity_error	(ecc_overall_parity_error)
	
);
*/

assign emem_r_data = emem_r_data_with_ecc;
assign ecc_error_syndrome = '0;
assign ecc_overall_parity_error = '0;

wire ready_sram_a, ready_sram_b;
assign emem_ready = ready_sram_a & ready_sram_b;
wire error_sram_a, error_sram_b;
assign emem_error = error_sram_a | error_sram_b;

qdr2_sram #(

	.mem_data_width(single_mem_data_width),
	.mem_addr_width(mem_addr_width),
	.IODELAY_GRP(IODELAY_GRP),
	.read_delay_cycles(read_delay_cycles),
	.qdr2_pll_lock_cycles(qdr2_pll_lock_cycles),
	.simulation_speed_up(simulation_speed_up)
	
	
) qdr2_sram_a (

	.clk,
	.clk2x,
	.clk2x90,
	.reset,
	
	.qdr				(qdr[0]),
	
	.ready			(ready_sram_a),
	.error			(error_sram_a),
	
	.rd				(rd),
	.r_addr			(emem_r_addr),
	.r_valid			(),
	.r_data			(emem_r_data_with_ecc[single_mem_data_width * 4 - 1 : 0]),
	
	.wr				(wr),
	.w_addr			(emem_w_addr),
	.w_data			(emem_w_data_with_ecc_ff[single_mem_data_width * 4 - 1 : 0]),
	
	.chipscope		(chipscope_1)
);

qdr2_sram #(

	.mem_data_width(single_mem_data_width),
	.mem_addr_width(mem_addr_width),
	.IODELAY_GRP(IODELAY_GRP),
	.read_delay_cycles(read_delay_cycles),
	.qdr2_pll_lock_cycles(qdr2_pll_lock_cycles),
	.simulation_speed_up(simulation_speed_up)
	
) qdr2_sram_b (

	.clk,
	.clk2x,
	.clk2x90,
	.reset,
	
	.qdr				(qdr[1]),
	
	.ready			(ready_sram_b),
	.error			(error_sram_b),
	
	.rd				(rd),
	.r_addr			(emem_r_addr),
	.r_valid			(),
	.r_data			(emem_r_data_with_ecc[single_mem_data_width * 8 - 1 : single_mem_data_width * 4]),
	
	.wr				(wr),
	.w_addr			(emem_w_addr),
	.w_data			(emem_w_data_with_ecc_ff[single_mem_data_width * 8 - 1 : single_mem_data_width * 4]),
	
	.chipscope		(chipscope_2)
);

/*
*	Input logic
*/

assign s_axis_tready_internal = 1'b1; // we are always ready !!!

// Offsets of the output queues
function int unsigned get_queue_offset(int unsigned queue);
	
	get_queue_offset = 0;
	for (int unsigned i = 0; i < queue; i++) begin
		get_queue_offset += queue_sizes_256b[i];
	end
	
endfunction




// Counters for the queues
logic [output_ports-1:0] emem_enqueue = '0, emem_dequeue = '0;
logic [output_ports-1:0] emem_queue_not_empty;
logic [output_ports-1:0] emem_queue_could_store_current_packet = '1; // at power on there is alwas enough freespace in external memory for one packet
logic [output_ports-1:0][mem_addr_width-1:0] emem_queue_wr_addr, emem_queue_rd_addr;

localparam minimum_freespace_for_one_packet = 16384; // for lower fpga utilization: use 2^n

generate

	for (genvar i = 0; i < output_ports; i++) begin
	
		localparam queue_counter_width = $clog2(get_queue_offset(i) + queue_sizes_256b[i]); // maximum address would be get_queue_offset(i) + queue_sizes_256b[i] - 1
		
		logic [queue_counter_width-1:0]
			emem_queue_wr_addr_internal = get_queue_offset(i),
			emem_queue_rd_addr_internal = get_queue_offset(i)
		;
		
		localparam queue_level_counter_width = $clog2(queue_sizes_256b[i] + 1);
		logic [queue_level_counter_width-1:0] queue_level = '0;
		
		// delay the "emem_enqueue" information for imem_fallthrough_latency cycles
		
		localparam imem_fallthrough_latency = 1; // the emem_queue_not_empty has to be delayed to match the imem fifo latency (minimum 1, try with simulation)
	
		logic [imem_fallthrough_latency-1:0] emem_enqueue_delayed = '0;
		
		always_ff @(posedge clk) begin
			
			emem_enqueue_delayed[imem_fallthrough_latency-1] <= emem_enqueue[i];
			
			for (int i = 0; i < imem_fallthrough_latency-1; i++) begin
				emem_enqueue_delayed[i] <= emem_enqueue_delayed[i+1];
			end
				
			if (reset)
				emem_enqueue_delayed <= '0;
				
		end
		
		always_ff @(posedge clk) begin
		
			if (emem_enqueue[i]) begin
			
				if (emem_queue_wr_addr_internal == get_queue_offset(i) + queue_sizes_256b[i] - 1) begin // maximum position reached?
					
					emem_queue_wr_addr_internal <= get_queue_offset(i); // wrap around
					
				end else begin
					
					emem_queue_wr_addr_internal <= emem_queue_wr_addr_internal + 1;
					
				end
				
			end
			
			if (emem_dequeue[i]) begin
				
				if (emem_queue_rd_addr_internal == get_queue_offset(i) + queue_sizes_256b[i] - 1) begin // maximum position reached?
					
					emem_queue_rd_addr_internal <= get_queue_offset(i); // wrap around
					
				end else begin
					
					emem_queue_rd_addr_internal <= emem_queue_rd_addr_internal + 1;
					
				end
				
			end
			
			if (emem_enqueue_delayed[0] ^ emem_dequeue[i]) begin // enqueue and dequeue at the same time: no level change
				
				if (emem_enqueue_delayed[0]) begin
					queue_level <= queue_level + 1;
				end else begin
					queue_level <= queue_level - 1;
				end
				
			end
			
			if (s_axis_tvalid & s_axis_tready_internal & s_axis_tlast) begin
				// at the end of every packet "emem_queue_could_store_current_packet" gets updated for the next packet
				emem_queue_could_store_current_packet[i] <= queue_sizes_256b[i] - queue_level >= minimum_freespace_for_one_packet / (axis_data_width / 8) + imem_fallthrough_latency;
			end
			
			if (reset) begin
			
				emem_queue_could_store_current_packet[i] <= '1; // after a reset there is always enough freespace for at least one packet
				queue_level <= 0;
				emem_queue_wr_addr_internal <= get_queue_offset(i);
				emem_queue_rd_addr_internal <= get_queue_offset(i);
			
			end
			
		end
		
		// assign queue status signals
		always_comb begin
			emem_queue_not_empty[i] = queue_level != '0;
			emem_queue_wr_addr[i] = emem_queue_wr_addr_internal;
			emem_queue_rd_addr[i] = emem_queue_rd_addr_internal;
		end
	
	end

endgenerate

// address line connections to the external memory (+ rd and wr)
always_comb begin
	rd = 1'b1;//emem_dequeue != '0; // we always read...
end

always_ff @( posedge clk ) begin

	for (int unsigned i = 0; i < output_ports; i++) begin
	
		if (emem_enqueue == 2**i) begin
			emem_w_addr <= emem_queue_wr_addr[i];
		end
		
		if (emem_dequeue == 2**i) begin
			emem_r_addr <= emem_queue_rd_addr[i];
		end
		
	end
	
	wr <= emem_enqueue != '0;
end

// internal memory queues (for multicasts)

logic [output_ports-1:0][mem_addr_width-1:0] input_word_count; // counts the words that got forwarded ether to emem or imem

logic [output_ports-1:0] imem_enqueue = 1'b0, imem_dequeue = 1'b0, imem_commit = 1'b0, imem_revert = 1'b0;
wire [output_ports-1:0] imem_queue_not_empty, imem_queue_full, imem_tlast;

typedef struct packed {										// 280 bit
	
	logic [axis_data_width-1:0] tdata;					// 256 bit
	logic [tkeep_encoded_width-1:0] tkeep_encoded;	// 5 bit
	logic [mem_addr_width-1:0]	word_pos;				// 19 bit

} imem_data_t;

imem_data_t [output_ports-1:0] imem_r_data;
axis_defs::out_vport_t [output_ports-1:0] imem_r_data_vport;

generate

	for (genvar i = 0; i < output_ports; i++) begin
	
		imem_data_t imem_w_data;

		assign imem_w_data = '{
			tdata: s_axis_tdata,
			tkeep_encoded: s_axis_tkeep_encoded,
			word_pos: input_word_count[i]
		};
		
		wire queue_empty;
		
		simple_packet_fifo #(
		
			.data_width($bits(imem_data_t)),
			.metadata_width($bits(s_axis_tuser.out.vport)),
			.store_and_forward(1'b1),
			.almost_full_offset(8)
			
		) simple_packet_fifo_0 (
			
			.wr_clk(clk),
			.wr_reset(reset),
			.wr_data(imem_w_data),
			.wr_metadata(s_axis_tuser.out.vport),
			.wr_last(imem_commit[i]),
			.wr_dirty(imem_revert[i]),
			.wr(imem_enqueue[i]),
			.wr_full(),
			.wr_almost_full(imem_queue_full[i]),
			
			.rd_clk(clk),
			.rd_reset(reset),
			.rd_data(imem_r_data[i]),
			.rd_metadata(imem_r_data_vport[i]),
			.rd_last(imem_tlast[i]),
			.rd(imem_dequeue[i]),
			.rd_empty(queue_empty),
			
			.error()
		);
		
		assign imem_queue_not_empty[i] = !queue_empty;

	end
	
endgenerate

// detect if the current packet on the slave stream interface is a multicast packet
logic multicast_packet;

always_comb begin

	automatic bit found_one_destination = '0;
	
	multicast_packet = '0;
	
	for (int unsigned i = 0; i < output_ports; i++) begin
	
		if (s_axis_tuser.out.port[i]) begin
		
			if (found_one_destination) begin
				multicast_packet = '1;
				break;
			end
		
			found_one_destination = '1;
		end
	end
end

// generate a start-of-frame
logic s_axis_tfirst = '1;

always_ff @(posedge clk) begin

	if (s_axis_tvalid & s_axis_tready_internal) begin
		s_axis_tfirst <= s_axis_tlast;
	end
	
	if (reset)
		s_axis_tfirst <= '1;

end

// synchronize emem_ready with packet frame timing
logic emem_ready_for_current_packet, emem_ready_for_current_packet_ff = '0;

always_ff @(posedge clk) begin

	if (reset) begin
	
		emem_ready_for_current_packet_ff <= '0;
		
	end else begin
	
		if (s_axis_tfirst && emem_ready) // emem_ready does not reset after once set
			emem_ready_for_current_packet_ff <= '1;
	end
		

end

always_comb begin

	if (reset) begin
		
		emem_ready_for_current_packet = '0;
		
	end else begin

		emem_ready_for_current_packet = emem_ready_for_current_packet_ff;
		
		if (s_axis_tfirst && emem_ready)
			emem_ready_for_current_packet = '1;
	end
		
end

// the main packet distributor

logic [output_ports-1:0] this_packet_should_go_into_imem;

always_comb begin

	emem_enqueue = '0;
	this_packet_should_go_into_imem = '0;

	if (!reset) begin
	
		automatic bit process_in_imem = '0;
		
		if (multicast_packet) begin
		
			process_in_imem = '1;
			
		end else begin
		
			// unicast: calculate outgoing port
			automatic int unsigned out_port = 0;
			for (int unsigned i = 0; i < output_ports; i++) begin
			
				if (s_axis_tuser.out.port[i] == 1'b1) begin
				
					out_port = i;
					break;
				end
			end
		
			if (emem_ready_for_current_packet & emem_queue_could_store_current_packet[out_port]) begin
			
				if (s_axis_tvalid & s_axis_tready_internal) begin
				
					// store data on every valid data cycle
					emem_enqueue = s_axis_tuser.out.port;
				end
			
			end else begin
			
				process_in_imem = '1;
				
			end
		
		end
		
		if (process_in_imem) begin
			this_packet_should_go_into_imem = s_axis_tuser.out.port;
		end
		
	end
	
end

// additional distribution logic for the packet fifos

logic [output_ports-1:0] last_imem_packet_was_dropped = '0;

generate

	for (genvar i = 0; i < output_ports; i++) begin
	
		logic drop_packet_ff = '0, packet_was_dropped_from_the_beginning = '0;
		
		always_comb begin
		
			imem_enqueue[i] = '0;
			imem_commit[i] = '0;
			imem_revert[i] = '0;
			
			if (reset) begin
			
			end else begin
			
				if (s_axis_tvalid & s_axis_tready_internal & this_packet_should_go_into_imem[i]) begin
				
					if (!imem_queue_full[i] && !drop_packet_ff) begin
					
						imem_enqueue[i] = '1;
						
						if (s_axis_tlast) begin
							imem_commit[i] = '1;
						end
						
					end else begin
					
						if (s_axis_tlast) begin
						
							if (!s_axis_tfirst && !packet_was_dropped_from_the_beginning)
								imem_enqueue[i] = '1; // only enqueue if can can not simply forget about the packet
								
							imem_commit[i] = '1;
							imem_revert[i] = '1;
						end
					end

				end
			end
		end
		
		always_ff @(posedge clk) begin

			if (reset) begin
			
				drop_packet_ff <= '0;
				packet_was_dropped_from_the_beginning <= '0;
				last_imem_packet_was_dropped[i] <= '0;
			
			end else begin
			
				if (s_axis_tvalid & s_axis_tready_internal & this_packet_should_go_into_imem[i]) begin
				
					if (s_axis_tlast)
						packet_was_dropped_from_the_beginning <= '0;
				
					if (imem_queue_full[i]) begin
				
						drop_packet_ff <= '1;
						
						if (s_axis_tfirst)
							packet_was_dropped_from_the_beginning <= '1;
							
					end
					
					if (s_axis_tlast) begin
					
						drop_packet_ff <= '0;
					end
				end
				
				if (imem_commit[i]) begin
					last_imem_packet_was_dropped[i] <= imem_revert[i];
				end
				
			end
		end
	end
	
endgenerate

/*
*		Counters for the input words (counts all data that was input either into the imem or the emem for each port)
*/

// returns the needed bits for a word counter matching the queue size (so the counters associated to a queue can possibly be taller that the external memory address width)
function int unsigned get_queue_word_counter_width(int unsigned queue);

	get_queue_word_counter_width = $clog2(queue_sizes_256b[queue] > multicast_queue_size_256b ? queue_sizes_256b[queue] : multicast_queue_size_256b); // questasim 10.1d workaround: $max(,) causes "Parameter value must be constant."

endfunction

generate

	for (genvar port = 0; port < output_ports; port++) begin
	
		logic [get_queue_word_counter_width(port)-1:0] input_word_count_internal = '0, input_word_count_revert_internal = '0; // (reduced) width counters for the input word count

		always_ff @(posedge clk) begin
		
			if (imem_enqueue[port]) begin
			
				input_word_count_internal <= input_word_count_internal + 1;
				
				if (imem_commit[port]) begin // "tlast"
				
					if (imem_revert[port]) begin // "dirty"
						input_word_count_internal <= input_word_count_revert_internal; //...also revert the counter
					end else begin
						input_word_count_revert_internal <= input_word_count_internal + 1; // save the next counter if we have to roll back later
					end
				
				end 
				
			end
			
			if (emem_enqueue[port]) begin // this word is forwarded to the emem
				input_word_count_internal <= input_word_count_internal + 1;
			end
			
			if (reset) begin
				input_word_count_internal <= '0;
				input_word_count_revert_internal <= '0;
			end
		
		end
		
		assign input_word_count[port] = input_word_count_internal; // attach the counter to the possibly wider "external" input_word_count

	end
	
endgenerate

/*
*		Counters for the output words
*/

logic inc_output_word_counter[output_ports];
logic [output_ports-1:0][mem_addr_width-1:0] output_word_count; // counts the words that came out from imem or emem

generate

	for (genvar port = 0; port < output_ports; port++) begin
	
		logic [get_queue_word_counter_width(port)-1:0] output_word_count_internal = '0; // (reduced) width counters for the output word count
		
		always_ff @(posedge clk) begin
			
			if (inc_output_word_counter[port]) begin
				output_word_count_internal <= output_word_count_internal + 1;
			end
			
		end
		
		assign output_word_count[port] = output_word_count_internal; // attach the counter to the possibly wider "external" output_word_count
	end
endgenerate

/*
*		Output logic
*/

// check if the next word has to come from the imem (PacketFIFO)
logic use_imem_for_output[output_ports];

generate

	for (genvar port = 0; port < output_ports; port++) begin
	
		always_comb begin
		
			use_imem_for_output[port] = 1'b0;
		
			if (imem_queue_not_empty[port] && imem_r_data[port].word_pos == output_word_count[port]) begin // the PacketFIFO (imem) is not empty and has the next following data word for the output port
			
				use_imem_for_output[port] = 1'b1;
			end
		
		end
	end
	
endgenerate

logic port_process_emem_data[output_ports]; // driven from the emem output arbiter
logic emem_processing_locked_pipeline_conflict[output_ports]; // if there is a OUT_FW_IMEM cycle in the pipeline we have to wait until this is processed before we add more emem cycles

localparam effective_read_delay_cycles = read_delay_cycles + 1;

generate

	for (genvar port = 0; port < output_ports; port++) begin
	
		typedef enum {
			
			OUT_FW_IDLE = 0,
			OUT_FW_IMEM = 1,
			OUT_FW_EMEM = 2
			
		} out_fw_pipeline_t;

		out_fw_pipeline_t out_fw_pipeline[effective_read_delay_cycles] = '{default: OUT_FW_IDLE};
		out_fw_pipeline_t out_fw_pipeline_idle_for_test[effective_read_delay_cycles] = '{default: OUT_FW_IDLE};
		
		always_ff @(posedge clk) begin
		
			out_fw_pipeline[0] <= OUT_FW_IDLE;

			// shift pipeline
			for (int i = 0; i <= effective_read_delay_cycles-2; i++) begin
				out_fw_pipeline[i+1] <= out_fw_pipeline[i];
			end
			
			if (!output_fifo_almost_full[port]) begin
			
				if (use_imem_for_output[port]) begin
					
					// modify pipeline at the highest possible position to forward the imem data
					for (int i = 0; i < effective_read_delay_cycles; i++) begin
					
						if (out_fw_pipeline[i] != OUT_FW_IDLE) begin // find the first pipeline slot which is in use
						
							out_fw_pipeline[i] <= OUT_FW_IMEM; // tricky: i (and not i-1) because the pipeline gets shifted a few lines above
							break;
							
						end
					end
					
					if (out_fw_pipeline == out_fw_pipeline_idle_for_test) begin // the whole pipeline is idle
						
							//...directly forwarding from imem (see comb. logic below)
					end
					
				end else begin
				
					if (port_process_emem_data[port]) begin
					
						if (output_fifo_almost_full[port])
							$display("Error: Output FIFO almost full");
					
						out_fw_pipeline[0] <= OUT_FW_EMEM;
					end
				
				end
			end
			
			if (reset) begin
				out_fw_pipeline <= '{default: OUT_FW_IDLE};
			end
			
		end
		
		logic immediately_use_imem_for_forward; // circumvent the pipeline to directly forward from imem
		
		always_comb begin
		
			immediately_use_imem_for_forward = 1'b0;
			emem_dequeue[port] = 1'b0;
			inc_output_word_counter[port] = 1'b0;
			
			if (!output_fifo_almost_full[port]) begin
			
				if (use_imem_for_output[port]) begin
				
				
					// modify pipeline at the highest possible position to forward the imem data
					for (int i = 0; i < effective_read_delay_cycles; i++) begin
					
						if (out_fw_pipeline[i] != OUT_FW_IDLE) begin // we changed the pipeline in the always_ff before => update the word counter
						
							inc_output_word_counter[port] = 1'b1;
							break;
							
						end
					end
					
					if (out_fw_pipeline == out_fw_pipeline_idle_for_test) begin // the whole pipeline is idle
						
							immediately_use_imem_for_forward = 1'b1; //...directly forwarding from imem
							inc_output_word_counter[port] = 1'b1;
					end
					
				end else begin
				
					if (port_process_emem_data[port]) begin
					
						emem_dequeue[port] = 1'b1;
						inc_output_word_counter[port] = 1'b1;
					end
				
				end
					
			end
			
			// a decision was made: move data from the imem to the output fifo NOW
			imem_dequeue[port] = 1'b0;
		
			if (immediately_use_imem_for_forward || out_fw_pipeline[effective_read_delay_cycles-1] == OUT_FW_IMEM) begin
				
				imem_dequeue[port] = 1'b1;
			end
		
		end
		
		// locking of emem when an OUT_FW_IMEM cycle is in the pipeline
		always_comb begin
		
			emem_processing_locked_pipeline_conflict[port] = 1'b0;
			
			for (int i = 0; i <= effective_read_delay_cycles-1; i++) begin
				if (out_fw_pipeline[i] == OUT_FW_IMEM)
					emem_processing_locked_pipeline_conflict[port] = 1'b1;
			end
			
			if (immediately_use_imem_for_forward)
				emem_processing_locked_pipeline_conflict[port] = 1'b1;
			
		
		end
		
		// output multiplexer with register
		
		stream_data_t output_data_ff;
		logic [parity_bits-2:0] ecc_error_syndrome_delayed;
		logic ecc_overall_parity_error_delayed;
		
		
		always_ff @(posedge clk) begin
		
			output_fifo_wr[port] <= '0;
		
			if (immediately_use_imem_for_forward || out_fw_pipeline[effective_read_delay_cycles-1] == OUT_FW_IMEM) begin
			
				output_data_ff.tdata <= imem_r_data[port].tdata;
				output_data_ff.tkeep_encoded <= imem_r_data[port].tkeep_encoded;
				output_data_ff.tlast <= imem_tlast[port];
				output_data_ff.out_vport <= imem_r_data_vport[port];
				
				// no error correction with imem
				ecc_error_syndrome_delayed <= '0;
				ecc_overall_parity_error_delayed <= '0;
				
				output_fifo_wr[port] <= 1'b1;
			
			end else begin
			
				if (out_fw_pipeline[effective_read_delay_cycles-1] == OUT_FW_EMEM) begin
				
					output_data_ff <= emem_r_data;
					
					ecc_error_syndrome_delayed <= ecc_error_syndrome;
					ecc_overall_parity_error_delayed <= ecc_overall_parity_error;
					
					output_fifo_wr[port] <= 1'b1;
				
				end
			
			end
			
			if (reset) begin
				output_fifo_wr[port] <= 1'b0;
			end
		
		end
		
		// SECDED ECC corrector
		
		stream_data_t output_data_corrected; 
		
		/*
		secded_corrector #(
		
			.data_bits(mem_data_width), // 278 bit
			.hamming_code_parity_bits(parity_bits-1) // 9 bit
			
		) secded_corrector_0 (
		
			.data_in(output_data_ff),
			.data_out(output_data_corrected),
			
			.error_syndrome(ecc_error_syndrome_delayed),
			.overall_parity_error(ecc_overall_parity_error_delayed),
			
			.corrected_error(),
			.uncorrectable_error()
		
		);
		*/
		
		assign output_data_corrected = output_data_ff;
		
		// decode the tkeep at this point
		logic [axis_tkeep_width-1:0] tkeep_decoded;
		
		tkeep_decoder #(.tkeep_width(axis_tkeep_width)) tkeep_enc0 (
			.tkeep_encoded(output_data_corrected.tkeep_encoded),
			.tkeep(tkeep_decoded)
		);
		
		// attach to output fifo
		assign output_fifo_data_in[port] = '{ tdata: output_data_corrected.tdata, tkeep_reduced: tkeep_decoded[axis_tkeep_width-1:1] };
		assign output_fifo_tlast_in[port] = output_data_corrected.tlast;
		assign output_fifo_metadata_in[port] = '{ out_vport: output_data_corrected.out_vport };
		
	end
endgenerate

/*
*		Output arbiter
*/
logic [$clog2(output_ports)-1:0] current_output_port = '0, last_output_port = '0;

always_comb begin

	current_output_port = '0; // theoretically unnecessary
	port_process_emem_data = '{ default: '0 };
	
	for (int unsigned i = 0; i < output_ports; i++) begin
		if (i == last_output_port) begin
		
			for (int unsigned j = 0; j < output_ports; j++) begin
			
				current_output_port = (i + j + 1) % output_ports;
				
				if (
						!output_fifo_almost_full[current_output_port]
					&&	!use_imem_for_output[current_output_port]
					&&	emem_queue_not_empty[current_output_port]
					&& !emem_processing_locked_pipeline_conflict[current_output_port]
				) begin

					port_process_emem_data[current_output_port] = 1'b1;
				
					break; // the arbiter decided for an output port - stop here
				end
			
			end
			
			break; // the first for loop is just used to find the last output port
		end
	end
end

always_ff @(posedge clk) begin
	
	last_output_port <= current_output_port;
	
	if (reset)
		last_output_port <= '0;
	
end


endmodule


