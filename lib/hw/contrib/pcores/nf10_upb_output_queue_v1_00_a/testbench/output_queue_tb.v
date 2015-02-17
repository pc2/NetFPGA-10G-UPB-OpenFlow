/*
 * UPB Output Queue testbench
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

`timescale 1ns / 1ps
`default_nettype none

module output_queue_tb #(
	
	parameter output_ports					= 2,
	parameter axis_data_width				= 256,
	parameter axis_tkeep_width				= axis_data_width / 8 // 32 bit
	
);


	logic clk = '0, clk2x = '0;
	
	always begin
		#1.666 begin 
			clk = ~clk;
			clk2x = ~clk2x;
		end
		#1.666 begin
			clk2x = ~clk2x;
		end
	end
	
	logic clk2x90 = '0;
	always @(*) begin
		clk2x90 = #0.833 clk2x;
	end
	
	logic clk156 = '0;
	always begin
		#3.2 clk156 = ~clk156;
	end
	
	logic reset = 0;

	logic [axis_data_width-1:0] s_axis_tdata;
	logic [axis_tkeep_width-1:0] s_axis_tkeep;
	axis_defs::tuser_t s_axis_tuser;
	logic s_axis_tlast;
	logic s_axis_tvalid = '0;
	wire s_axis_tready;

	wire m_axis_clk[output_ports];
	assign m_axis_clk = '{ default: clk156 };
	logic m_axis_reset[output_ports] = '{ default: '0};
	wire [axis_data_width-1:0] m_axis_tdata[output_ports];
	wire [axis_tkeep_width-1:0] m_axis_tkeep[output_ports];
	axis_defs::tuser_t m_axis_tuser[output_ports];
	wire m_axis_tlast[output_ports];
	wire m_axis_tvalid[output_ports];
	logic m_axis_tready[output_ports] = '{ default:'1 };
	
	qdr2_sram_if qdr[2]();
	
	output_queue #(

		.output_ports								(output_ports),
		
		.multicast_queue_size_256b				(2**9), // 16KB
		.queue_sizes_256b							( { 19'd1000, 19'd1000 } ),
		
		.store_and_forward_ports				( { 1'b1, 1'b0 } ),
		
		.simulation_speed_up						(1)

	) output_queue_0 (
	
		.*,
		
		.chipscope_0(),
		.chipscope_1(),
		.chipscope_2()
		
	);
	
	logic qdr_a_cq_memory, qdr_a_cq_n_memory;
	logic [35:0] qdr_a_q_memory;
	
	CY7C1515JV18 qdr_a (
		.D(qdr[0].d),
		.Q(qdr_a_q_memory),
		.A(qdr[0].sa),
		.RPS_n(qdr[0].r_n),
		.WPS_n(qdr[0].w_n),
		.BW_n(qdr[0].bw_n),
		.K(qdr[0].k),
		.K_n(qdr[0].k_n),
		.C(qdr[0].c),
		.C_n(qdr[0].c_n),
		.CQ(qdr_a_cq_memory),
		.CQ_n(qdr_a_cq_n_memory)
	);
	
	// simulate delay on the PCB's traces and on the FPGA's IOB
	always @(*) begin
		qdr[0].cq = #0 qdr_a_cq_memory;
		qdr[0].cq_n = #0 qdr_a_cq_n_memory;
		
		qdr[0].q = #6 qdr_a_q_memory;
	end
	
	logic qdr_b_cq_memory, qdr_b_cq_n_memory;
	logic [35:0] qdr_b_q_memory;
	
	CY7C1515JV18 qdr_b (
		.D(qdr[1].d),
		.Q(qdr_b_q_memory),
		.A(qdr[1].sa),
		.RPS_n(qdr[1].r_n),
		.WPS_n(qdr[1].w_n),
		.BW_n(qdr[1].bw_n),
		.K(qdr[1].k),
		.K_n(qdr[1].k_n),
		.C(qdr[1].c),
		.C_n(qdr[1].c_n),
		.CQ(qdr_b_cq_memory),
		.CQ_n(qdr_b_cq_n_memory)
	);
	
	// simulate delay on the PCB's traces and on the FPGA's IOB
	always @(*) begin
		qdr[1].cq = #0 qdr_b_cq_memory;
		qdr[1].cq_n = #0 qdr_b_cq_n_memory;
		
		qdr[1].q = #6 qdr_b_q_memory;
	end
	
	class mailbox_t;
	
		int unsigned length_bytes;
		bit [7:0] dest_vport;
		
	endclass
	
	mailbox #(mailbox_t) check_packet_mb[output_ports];


	task automatic send_packet(
	
		input int unsigned length_bytes,
		input bit [7:0] dest_port, dest_vport
	
	);
	
		automatic bit [7:0] data = 0;
		
		automatic mailbox_t mb_data;
		
		mb_data = new();
		mb_data.length_bytes = length_bytes;
		mb_data.dest_vport = dest_vport;
		for (int unsigned i = 0; i < output_ports; i++) begin
			if (dest_port[i] == 1'b1)
				check_packet_mb[i].put(mb_data);
		end
		
		while (length_bytes) begin
		
			for (int unsigned i = 0; i < 32; i++) begin
				
				if (length_bytes) begin
				
					s_axis_tdata[i*8+7-:8] <= data++;
					s_axis_tkeep[i] <= 1'b1;
					
					length_bytes--;
				
				end else begin
				
					s_axis_tdata[i*8+7-:8] <= 8'hx;
					s_axis_tkeep[i] <= 1'b0;
				
				end
				
			end
			
			s_axis_tvalid <= 1'b1;
			s_axis_tlast <= length_bytes ? 1'b0 : 1'b1;
			s_axis_tuser <= '0;
			s_axis_tuser.packet_length <= length_bytes;
			s_axis_tuser.out.port <= dest_port;
			s_axis_tuser.out.vport <= dest_vport;
				
			@(posedge clk);
			
			while (s_axis_tready != 1)
				@(posedge clk); // wait further clock cycles if tready is not set
				
			
		end
		
		s_axis_tvalid <= 1'b0;
		s_axis_tdata <= 256'hx;
		s_axis_tkeep <= 32'hx;
		
	endtask
	
	task automatic receive_packet(
	
		input int unsigned dest_port,
		ref int unsigned length_bytes,
		ref axis_defs::tuser_t tuser,
		ref bit error
		
	);
	
		automatic bit [7:0] data = '0;
		
		length_bytes = 0;
		error = '0;
		
		do begin
		
			@(posedge m_axis_clk[dest_port]);
			
			if (m_axis_tvalid[dest_port] && m_axis_tready[dest_port]) begin
			
				if (length_bytes == 0) begin
				
					tuser = m_axis_tuser[dest_port];
					
				end else begin
				
					if (tuser != m_axis_tuser[dest_port]) begin
						$error("Port %d: TUSER Error: TUSER is not constant", dest_port);
						error = 1'b1;
					end
				
				end
			
				if (m_axis_tkeep[dest_port] != '1 && m_axis_tlast[dest_port] != 1'b1) begin
					$error("Port %d: TKEEP Error: TKEEP may only have 0's in the last data beat", dest_port);
					error = 1'b1;
				end
					
				begin
					automatic bit found_zero = 1'b0;
					for (int unsigned i = 0; i < 32; i++) begin
						
						if (m_axis_tkeep[dest_port][i] == 1'b0) begin
						
								found_zero = 1'b1;
						end
						
						if (m_axis_tkeep[dest_port][i] == 1'b1) begin
						
							if (found_zero) begin
							
								$error("Port %d: TKEEP Error: Only the upper bits may have contiguous 0's", dest_port);
								error = 1'b1;
								break;
							end
							
						end
					end
				end
				
				if (m_axis_tkeep[dest_port] == '0) begin
					$error("Port %d: TKEEP Error: Data cycles with no tkeep bits set are not allowed", dest_port);
					error = 1'b1;
				end
				
				for (int unsigned i = 0; i < 32; i++) begin
					
					if (m_axis_tkeep[dest_port][i] == 1'b1) begin
					
						length_bytes++;
					
						if (m_axis_tdata[dest_port][i*8+7-:8] != data) begin
						
							$error("Port %d: TDATA Error: Data does not match. Expected 0x%02x, got 0x%02x", dest_port, data, m_axis_tdata[dest_port][i*8+7-:8]);
							error = 1'b1;
						
						end
						
						data++;
					end
				
				end
			
			end
			
		end while (!(m_axis_tvalid[dest_port] && m_axis_tready[dest_port] && m_axis_tlast[dest_port]));
		
	endtask
	
	struct {
	
		int unsigned good_packets, bad_packets, dropped_packets, largest_block_dropped_packets;
	
	} statistics[output_ports] = '{ default: '{ default: '0 } };
	
	initial begin
	
		for (int port=0; port < output_ports; port++) begin
			check_packet_mb[port] = new();
		end
	
		fork
			begin
				automatic bit [7:0] dest_vport = 0;
			
				#13000 // wait until the SRAMs are initialized and ready
				@(posedge clk);
				
				forever begin
				
					send_packet($urandom_range(64, 8000), $urandom_range(0, 3), dest_vport++);

					//static int unsigned packet_size = 32 * 2;
					//send_packet(packet_size, 8'b01, dest_vport++);
					//send_packet(packet_size, 8'b10, dest_vport++);
					//send_packet(packet_size, 8'b11, dest_vport++);
				end
			end
			
			// lower the throughput at the destination ports
			forever begin
				m_axis_tready <= '{ default: '1};
				@(posedge clk156);
				m_axis_tready <= '{ default: '0};
				@(posedge clk156);
				@(posedge clk156);
				@(posedge clk156);
			end
			
			// check packets for the ports
			begin
				for (int i=0; i < output_ports; i++) begin
				
					automatic int port = i;
				
					fork
					
						forever begin
						
							automatic int unsigned length_bytes;
							automatic axis_defs::tuser_t tuser;
							automatic bit error;
							automatic int unsigned block_dropped_packets = 0;
							
							receive_packet(port, length_bytes, tuser, error);
							
							forever begin
							
								automatic mailbox_t mb_data;
							
								if (check_packet_mb[port].try_get(mb_data)) begin
								
									if (	mb_data.length_bytes == length_bytes
										&&	mb_data.dest_vport == tuser.out.vport) begin
										
											if (error)
												statistics[port].bad_packets++;
											else
												statistics[port].good_packets++;
										
											//$display("Port %d: Packet received", port);
											break;
										
									end else begin
									
										block_dropped_packets++;
									
										if (error)
												statistics[port].bad_packets++;
											else
												statistics[port].dropped_packets++;
									
										//$display("Port %d: Dropped packet with %d bytes and out.vport %d (Details: Received a packet with %d bytes and out.vport %d)", port, mb_data.length_bytes, mb_data.dest_vport, length_bytes, tuser.out.vport);
									
									end
								
								end else begin
								
									$error("Port %d: Received a packet which was not sent before (Details: Received a packet with %d bytes and out.vport %d)", port, length_bytes, tuser.out.vport);
									statistics[port].bad_packets++;
									
									break;
								
								end
							end
							
							if (statistics[port].largest_block_dropped_packets < block_dropped_packets)
								statistics[port].largest_block_dropped_packets = block_dropped_packets;
							
						end
					
					join_none
				
				end
			end
			
			// output statistics
			begin
				int unsigned run = 0;
				forever begin
				
					case (run++)
					
						0: #20000;
						1: #10000;
						2: #10000;
						3: #10000;
						default: #500000;
					
					endcase
					
					for (int port=0; port < output_ports; port++) begin
						$display("Port %d packet statistics: %d received, %d dropped (largest block of dropped packets %d), %d bad", port, statistics[port].good_packets, statistics[port].dropped_packets, statistics[port].largest_block_dropped_packets, statistics[port].bad_packets);
					end
				
				end
			end
			
		join
	
	end


endmodule
