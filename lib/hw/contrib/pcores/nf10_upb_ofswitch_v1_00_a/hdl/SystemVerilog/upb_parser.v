/*
 * UPB OpenFlow 1.0 Parser core
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

`include "tuple_t.v"

package eth_types;

	function logic [15:0] htons (input logic [15:0] in);
		return { in[7:0], in[15:8] };
	endfunction
	
	function logic [31:0] htonl (input logic [31:0] in);
		return { in[7:0], in[15:8], in[23:16], in[31:24] };
	endfunction
	
	function logic [47:0] htonl48 (input logic [47:0] in);
		return { in[7:0], in[15:8], in[23:16], in[31:24], in[39:32], in[47:40] };
	endfunction

	typedef enum logic [15:0] {
		
		et_ipv4		= htons(16'h0800),
		et_arp		= htons(16'h0806),
		et_vlan		= htons(16'h8100)
		
	} ethertype_t;

	typedef struct packed {

		ethertype_t ethertype;
		logic [47:0] src_mac;
		logic [47:0] dst_mac;

	} ethernet_t;

	typedef struct packed {
	
		ethertype_t ethertype;

		struct packed {
		
			// the fields inside here are converted to little-endian
			logic [7:0] vid_lo;
			logic [2:0] pcp;
			logic dei;
			logic [11:8] vid_hi;
		} tci;
		
	} vlan_t;
	
	typedef enum logic [7:0] {
		
		ip_prot_icmp = 8'd1,
		ip_prot_tcp = 8'd6,
		ip_prot_udp = 8'd17
	
	} ip_proto_t;
	
	typedef struct packed {
	
		logic [31:0] dst_addr;
		logic [31:0] src_addr;
		logic [15:0] header_chksum;
		ip_proto_t protocol;
		logic [7:0] ttl;
		logic [15:0] flags_fragment_offset;
		logic [15:0] id;
		logic [15:0] total_len;
		struct packed { // DiffServ fields are used here
			logic [5:0] dscp;
			logic [1:0] ecn; // Explicit Congestion Notification: Unused with OpenFlow 1.0
		} tos;
		logic [3:0] version;
		logic [3:0] hdr_len;
	
	} ipv4_t;
	
	typedef struct packed {
	
		logic [31:0] tpa;		// Target protocol address
		logic [47:0] tha;		// Target hardware address
		logic [31:0] spa;		// Sender protocol address
		logic [47:0] sha;		// Sender hardware address
		logic [15:0] oper;	// Operation
		logic [7:0] plen;		// Protocol address length
		logic [7:0] hlen;		// Hardware address length
		logic [15:0] ptype;	// Protocol type
		logic [15:0] htype;	// Hardware type
	
	} arp_t;
		
	typedef struct packed {
		
		// rest omitted for OF 1.0 parser
		logic [7:0] code;
		logic [7:0] icmp_type;
	
	} icmp_t;
	
	typedef struct packed {
		
		// rest omitted for OF 1.0 parser
		logic [15:0] dst_port;
		logic [15:0] src_port;
	
	} tcp_t;
	
	typedef struct packed {
		
		// rest omitted for OF 1.0 parser
		logic [15:0] dst_port;
		logic [15:0] src_port;
	
	} udp_t;

endpackage

module upb_parser #(

	parameter axis_data_width				= 256,
	parameter axis_tkeep_width				= axis_data_width / 8

) (

	input	wire									clk,
	input	wire									reset,
	
	input wire [axis_data_width-1:0]		s_axis_tdata,
	input wire [axis_tkeep_width-1:0]	s_axis_tkeep,
	input axis_defs::tuser_t				s_axis_tuser,
	input wire									s_axis_tlast,
	input wire 									s_axis_tvalid,
	output logic								s_axis_tready,
	
	output logic								valid,
	output logic								error,
	output tuple_t								tuple

);

assign s_axis_tready = 1'b1; // we are always ready

function int max(input int a, b);
	return a > b ? a : b;
endfunction

function int div_round_up(input int a, b);
	return a / b + (a % b ? 1 : 0);
endfunction

/*
* Determine the maximum number of bits needed for parsing
*/

localparam ip_max_options = 10;
localparam ip_header_size_bits = (5 + ip_max_options) * 32;

localparam bits_of_interest = 

		$bits(eth_types::ethernet_t)									// Ethernet header
	+	$bits(eth_types::vlan_t)										// + (optional) VLAN tag
	+	max(max(max(														// + maximum of...
			$bits(eth_types::arp_t),									// ...ARP
			ip_header_size_bits + $bits(eth_types::tcp_t)		// ...IP + TCP
		),
			ip_header_size_bits + $bits(eth_types::udp_t)		// ...IP + UDP
		),
			ip_header_size_bits + $bits(eth_types::icmp_t)		// ...IP + ICMP
		)
;

/*
* Extract the "bits of interest" from the ethernet stream and copy them into "extracted_packet"
* The result is ready when "extracted_packet_ready" is set (only valid for one cycle)
*/

localparam packet_copy_steps = div_round_up(bits_of_interest, axis_data_width);

typedef struct packed {

	logic [packet_copy_steps * axis_data_width - 1 : 0] data; // packet is larger than "bits_of_interest" (unused bits will be optimized away)
	logic [$clog2(packet_copy_steps * axis_data_width + 1)-1:0] size_bits;
	logic ready;
	
} packet_t;

packet_t extracted_packet = '{ ready: '0, default: 'x};
axis_defs::tuser_t extracted_tuser;

logic [$clog2(packet_copy_steps)-1:0] packet_copy_step = '0;
logic stop_copy = '0;

always_ff @(posedge clk) begin

	extracted_packet.ready <= 0;

	if (s_axis_tvalid) begin
	
		if (!stop_copy) begin
	
			automatic logic [$clog2(axis_tkeep_width+1)-1:0] bytes_kept = 0;
			
			// count all bytes which are kept in this data cycle (tkeep...)
			for (int i = 0; i < axis_tkeep_width; i++) begin
				if (s_axis_tkeep[i] == 1)
					bytes_kept++;
			end
			
			extracted_packet.size_bits <= (packet_copy_step ? extracted_packet.size_bits : 0) + bytes_kept * 8;
			
			for (int i = 0; i < packet_copy_steps; i++) begin
				if (packet_copy_step == i) begin
					extracted_packet.data[axis_data_width*i +: axis_data_width] <= s_axis_tdata;
				end
			end
			
			extracted_tuser <= s_axis_tuser; // tuser is constant during the packet
			
			if (packet_copy_step == packet_copy_steps-1) begin
				// all "interesting" words are copies
				stop_copy <= 1;
				extracted_packet.ready <= 1;
			
			end else begin
				packet_copy_step <= packet_copy_step + 1;
			end
			
			if (s_axis_tlast) begin
			
				// this is the last data word of the packet, copying can not continue
				extracted_packet.ready <= 1;
			end
		end
		
		// reset registers for the next packet
		if (s_axis_tlast) begin
			stop_copy <= 0;
			packet_copy_step <= 0;
		end
		
	end

	if (reset) begin
	
		stop_copy <= 0;
		packet_copy_step <= 0;
		extracted_packet.ready <= 0;
	end
end
	
/*
* The parser...
*/

typedef struct packed {
	tuple_t tuple;
	logic error;
	logic parse_vlan, parse_arp, parse_ipv4, parse_icmp, parse_udp, parse_tcp;
	logic [3:0] ip_option_fields_to_remove;
} parser_t;

/*
* 1. Parse MAC header
*/

parser_t tuple_eth_parsed = '{error: '0, default: 'x};
packet_t packet_eth_removed = '{ready: '0, default: 'x};

always_ff @(posedge clk) begin

	automatic eth_types::ethernet_t eth;
	eth = extracted_packet.data[0 +: $bits(eth_types::ethernet_t)];

	tuple_eth_parsed <= '0; // default: all unset values in the result tuple are '0'
	
	tuple_eth_parsed.tuple.port <= extracted_tuser.in.port;
	tuple_eth_parsed.tuple.vport <= extracted_tuser.in.vport;
	
	// check for errors
	if ($bits(eth_types::ethernet_t) > extracted_packet.size_bits)
		tuple_eth_parsed.error <= 1;
	
	tuple_eth_parsed.tuple.dmac <= eth.dst_mac;
	tuple_eth_parsed.tuple.smac <= eth.src_mac;
	tuple_eth_parsed.tuple.typ <= eth.ethertype; // this may also be the ethertype for vlan (will be changed in the next stage)
	
	// info for next stages
	case (eth.ethertype)
	
		eth_types::et_vlan: tuple_eth_parsed.parse_vlan <= 1;
		eth_types::et_arp: tuple_eth_parsed.parse_arp <= 1;
		eth_types::et_ipv4: tuple_eth_parsed.parse_ipv4 <= 1;
	endcase

	packet_eth_removed.data <= extracted_packet.data >> $bits(eth_types::ethernet_t);
	packet_eth_removed.size_bits <= extracted_packet.size_bits - $bits(eth_types::ethernet_t);
	packet_eth_removed.ready <= extracted_packet.ready;
	
	if (reset) begin
		packet_eth_removed.ready <= 0;
	end
end

/*
* 2. Parse VLAN header
*/

parser_t tuple_vlan_parsed = '{error: '0, default: 'x};
packet_t packet_vlan_removed = '{ ready: '0, default: 'x};

always_ff @(posedge clk) begin

	tuple_vlan_parsed <= tuple_eth_parsed;
	packet_vlan_removed <= packet_eth_removed;
	
	if (tuple_eth_parsed.parse_vlan) begin
		
		automatic eth_types::vlan_t vlan;
		vlan = packet_eth_removed.data[0 +: $bits(eth_types::vlan_t)];
		
		if ($bits(eth_types::vlan_t) > packet_eth_removed.size_bits)
			tuple_vlan_parsed.error <= 1;
		
		tuple_vlan_parsed.tuple.vid <= {vlan.tci.vid_hi, vlan.tci.vid_lo};
		tuple_vlan_parsed.tuple.pcp <= vlan.tci.pcp;
		tuple_vlan_parsed.tuple.typ <= vlan.ethertype;
		
		// info for next stages
		case (vlan.ethertype)
			eth_types::et_arp: tuple_vlan_parsed.parse_arp <= 1;
			eth_types::et_ipv4: tuple_vlan_parsed.parse_ipv4 <= 1;
		endcase
		
		packet_vlan_removed.data <= packet_eth_removed.data >> $bits(eth_types::vlan_t);
		packet_vlan_removed.size_bits <= packet_eth_removed.size_bits - $bits(eth_types::vlan_t);
		
	end
	
	if (reset) begin
		packet_vlan_removed.ready <= 0;
	end

end

/*
* 3. Parse ARP or IP header
*/

parser_t tuple_arp_ip_parsed = '{error: '0, default: 'x};
packet_t packet_arp_ip_removed = '{ ready: '0, default: 'x};

always_ff @(posedge clk) begin

	tuple_arp_ip_parsed <= tuple_vlan_parsed;
	packet_arp_ip_removed <= packet_vlan_removed;
	
	if (tuple_vlan_parsed.parse_arp) begin
	
		automatic eth_types::arp_t arp;
		arp = packet_vlan_removed.data[0 +: $bits(eth_types::arp_t)];
		
		if ($bits(eth_types::arp_t) > packet_vlan_removed.size_bits)
			tuple_arp_ip_parsed.error <= 1;

		tuple_arp_ip_parsed.tuple.sip <= arp.spa;
		tuple_arp_ip_parsed.tuple.dip <= arp.tpa;
		tuple_arp_ip_parsed.tuple.prot <= arp.oper[15:8]; // only lower 8 bits from opcode are used (Ethernet byte order!)
		
		// parsing is done here - no update of packet_arp_ip_removed neccessary
	end
	
	if (tuple_vlan_parsed.parse_ipv4) begin
	
		automatic eth_types::ipv4_t ipv4;
		ipv4 = packet_vlan_removed.data[0 +: $bits(eth_types::ipv4_t)];

		if ($bits(eth_types::ipv4_t) > packet_vlan_removed.size_bits)
			tuple_arp_ip_parsed.error <= 1;

		if (
				ipv4.version != 4
			|| ipv4.hdr_len < 5
		)
			tuple_arp_ip_parsed.error <= 1;
			
		tuple_arp_ip_parsed.ip_option_fields_to_remove <= ipv4.hdr_len - 5;

		tuple_arp_ip_parsed.tuple.sip <= ipv4.src_addr;
		tuple_arp_ip_parsed.tuple.dip <= ipv4.dst_addr;
		tuple_arp_ip_parsed.tuple.prot <= ipv4.protocol;
		tuple_arp_ip_parsed.tuple.tos <= ipv4.tos.dscp;
		
		case (ipv4.protocol)
		
			eth_types::ip_prot_icmp: begin
				tuple_arp_ip_parsed.parse_icmp <= 1;
				end
				
			eth_types::ip_prot_tcp: begin
				tuple_arp_ip_parsed.parse_tcp <= 1;
				end
				
			eth_types::ip_prot_udp: begin
				tuple_arp_ip_parsed.parse_udp <= 1;
				end
		
		endcase
		
		packet_arp_ip_removed.data <= packet_vlan_removed.data >> $bits(eth_types::ipv4_t);
		packet_arp_ip_removed.size_bits <= packet_vlan_removed.size_bits - $bits(eth_types::ipv4_t);
	end
	
	if (reset) begin
		packet_arp_ip_removed.ready <= 0;
	end
end

/*
* 4. Remove IP option fields in 2 cycles
*/

parser_t tuple_ip_parsed[2] = '{ '{ error: '0, default: 'x}, '{ error: '0, default: 'x}};
packet_t packet_ip_removed[2] = '{ '{ ready: '0, default: 'x}, '{ ready: '0, default: 'x}};

always_ff @(posedge clk) begin

	automatic int remove_bits_cycle_0 = tuple_arp_ip_parsed.ip_option_fields_to_remove[3:2] * 4 * 32; // process bits with significance 8,4
	automatic int remove_bits_cycle_1 = tuple_ip_parsed[1].ip_option_fields_to_remove[1:0] * 32; // process bits with significance 2,1
	
	tuple_ip_parsed[1] <= tuple_arp_ip_parsed;
	tuple_ip_parsed[0] <= tuple_ip_parsed[1];
	packet_ip_removed[1] <= packet_arp_ip_removed;
	packet_ip_removed[0] <= packet_ip_removed[1];
	
	if (tuple_arp_ip_parsed.ip_option_fields_to_remove * 32 > packet_arp_ip_removed.size_bits)
		tuple_ip_parsed[1].error <= 1;
	
	packet_ip_removed[1].data <= packet_arp_ip_removed.data >> remove_bits_cycle_0;
	packet_ip_removed[0].data <= packet_ip_removed[1].data >> remove_bits_cycle_1;

	packet_ip_removed[1].size_bits <= packet_arp_ip_removed.size_bits - remove_bits_cycle_0;
	packet_ip_removed[0].size_bits <= packet_ip_removed[1].size_bits - remove_bits_cycle_1;
	
	if (reset) begin
		packet_ip_removed[0].ready <= 0;
		packet_ip_removed[1].ready <= 0;
	end
end

/*
* 5. Parse ICMP, TCP, UDP
*/

parser_t tuple_icmp_tcp_udp_parsed = '{error: '0, default: 'x};
packet_t packet_icmp_tcp_udp_parsed = '{ ready: '0, default: 'x};

always_ff @(posedge clk) begin

	tuple_icmp_tcp_udp_parsed <= tuple_ip_parsed[0];
	packet_icmp_tcp_udp_parsed <= packet_ip_removed[0];
	
	if (tuple_ip_parsed[0].parse_icmp) begin
	
		automatic eth_types::icmp_t icmp;
		icmp = packet_ip_removed[0].data[0 +: $bits(eth_types::icmp_t)];
		
		if ($bits(eth_types::icmp_t) > packet_ip_removed[0].size_bits)
			tuple_icmp_tcp_udp_parsed.error <= 1;

		tuple_icmp_tcp_udp_parsed.tuple.tsp[15:8] <= icmp.icmp_type; // network byte order !
		tuple_icmp_tcp_udp_parsed.tuple.tdp[15:8] <= icmp.code;
		
		// parsing is done here - no update of packet_icmp_tcp_udp_parsed neccessary
	end
	
	if (tuple_ip_parsed[0].parse_tcp) begin
	
		automatic eth_types::tcp_t tcp;
		tcp = packet_ip_removed[0].data[0 +: $bits(eth_types::tcp_t)];
		
		if ($bits(eth_types::tcp_t) > packet_ip_removed[0].size_bits)
			tuple_icmp_tcp_udp_parsed.error <= 1;

		tuple_icmp_tcp_udp_parsed.tuple.tsp <= tcp.src_port;
		tuple_icmp_tcp_udp_parsed.tuple.tdp <= tcp.dst_port;
		
		// parsing is done here - no update of packet_icmp_tcp_udp_parsed neccessary
	end
	
	if (tuple_ip_parsed[0].parse_udp) begin
	
		automatic eth_types::udp_t udp;
		udp = packet_ip_removed[0].data[0 +: $bits(eth_types::udp_t)];
		
		if ($bits(eth_types::udp_t) > packet_ip_removed[0].size_bits)
			tuple_icmp_tcp_udp_parsed.error <= 1;

		tuple_icmp_tcp_udp_parsed.tuple.tsp <= udp.src_port;
		tuple_icmp_tcp_udp_parsed.tuple.tdp <= udp.dst_port;
		
		// parsing is done here - no update of packet_icmp_tcp_udp_parsed neccessary
	end
	
	if (reset) begin
		packet_icmp_tcp_udp_parsed.ready <= 0;
	end
	
end

assign tuple = tuple_icmp_tcp_udp_parsed.tuple;
assign error = tuple_icmp_tcp_udp_parsed.error;
assign valid = packet_icmp_tcp_udp_parsed.ready;

endmodule
