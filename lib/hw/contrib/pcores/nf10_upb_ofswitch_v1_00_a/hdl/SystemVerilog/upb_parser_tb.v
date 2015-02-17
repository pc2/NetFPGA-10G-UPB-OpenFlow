/*
 * UPB OpenFlow 1.0 Parser testbench
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
`include "tuple_t.v"

module upb_parser_tb #(
	parameter axis_data_width				= 256,
	parameter axis_tkeep_width				= axis_data_width / 8 // 32 bit
);

	typedef struct {
	
		bit packet[];
		axis_defs::tuser_t tuser;
		tuple_t tuple;
		bit error;
	
	} test_data_t;
	
	test_data_t packets[] = '{
	
	'{ // ARP packet (valid)
	
		packet: { << {{ << byte { // the packet is shortened to the minimum length to just be OF 1.0 parseable
			'h8eabbf392981002590a9b08c08060001080006040001002590a9b08c0abac8178eabbf3929810abac801
		}}}},
		
		tuser: '{in: '{port: 2, vport: 4}},
		
		tuple: '{
		
			port: 2, vport: 4,
			dmac: eth_types::htonl48(48'h8eabbf392981), smac: eth_types::htonl48(48'h002590a9b08c),
			typ: eth_types::htons(16'h0806),
			vid: 0, pcp: 0,
			sip: eth_types::htonl(32'h0abac817), dip: eth_types::htonl(32'h0abac801),
			prot: 8'h01, // ARP opcode: request (1)
			tos: 0,
			tsp: 0, tdp: 0,
			
			default: '0 // only "valid" is unset
		},
		
		error: 0
	},
	
	'{ // ARP packet (invalid - 1 byte too short)
	
		packet: { << {{ << byte { // the packet is shortened to the minimum length to just be OF 1.0 parseable
			'h8eabbf392981002590a9b08c08060001080006040001002590a9b08c0abac8178eabbf3929810abac8
		}}}},
		
		tuser: '{in: '{port: 2, vport: 4}},
		
		tuple: '{
		
			port: 2, vport: 4,
			dmac: eth_types::htonl48(48'h8eabbf392981), smac: eth_types::htonl48(48'h002590a9b08c),
			typ: eth_types::htons(16'h0806),
			vid: 0, pcp: 0,
			sip: eth_types::htonl(32'h0abac817), dip: eth_types::htonl(32'h0abac801),
			prot: 8'h01, // ARP opcode: request (1)
			tos: 0,
			tsp: 0, tdp: 0,
			
			default: '0 // only "valid" is unset
		},
		
		error: 1
	},
	
	'{ // UDP packet
	
		packet: { << {{ << byte { // the packet is shortened to the minimum length to just be OF 1.0 parseable
			'h8eabbf392981002590a9b08c080045000032971040004011c61e0abac8170aba0001e3340045
		}}}},
		
		tuser: '{in: '{port: 5, vport: 3}},
		
		tuple: '{
		
			port: 5, vport: 3,
			dmac: eth_types::htonl48(48'h8eabbf392981), smac: eth_types::htonl48(48'h002590a9b08c),
			typ: eth_types::htons(16'h0800),
			vid: 0, pcp: 0,
			sip: eth_types::htonl(32'h0abac817), dip: eth_types::htonl(32'h0aba0001),
			prot: 8'h11, // UDP
			tos: 0,
			tsp: eth_types::htons(16'he334), tdp: eth_types::htons(16'h0045),
			
			default: '0 // only "valid" is unset
		},
		
		error: 0
	},
	
	'{ // UDP packet - too short (1 byte missing)
	
		packet: { << {{ << byte { // the packet is shortened to the minimum length to just be OF 1.0 parseable
			'h8eabbf392981002590a9b08c080045000032971040004011c61e0abac8170aba0001e33400
		}}}},
		
		tuser: '{in: '{port: 5, vport: 3}},
		
		tuple: '{
		
			port: 5, vport: 3,
			dmac: eth_types::htonl48(48'h8eabbf392981), smac: eth_types::htonl48(48'h002590a9b08c),
			typ: eth_types::htons(16'h0800),
			vid: 0, pcp: 0,
			sip: eth_types::htonl(32'h0abac817), dip: eth_types::htonl(32'h0aba0001),
			prot: 8'h11, // UDP
			tos: 0,
			tsp: eth_types::htons(16'he334), tdp: eth_types::htons(16'h0045),
			
			default: '0 // only "valid" is unset
		},
		
		error: 1
	},
	
	'{ // ICMP packet
	
		packet: { << {{ << byte { // the packet is shortened to the minimum length to just be OF 1.0 parseable
			'h8eabbf392981002590a9b08c080045000054971440004001c6080abac8170aba00010800
		}}}},
		
		tuser: '{in: '{port: 5, vport: 3}},
		
		tuple: '{
		
			port: 5, vport: 3,
			dmac: eth_types::htonl48(48'h8eabbf392981), smac: eth_types::htonl48(48'h002590a9b08c),
			typ: eth_types::htons(16'h0800),
			vid: 0, pcp: 0,
			sip: eth_types::htonl(32'h0abac817), dip: eth_types::htonl(32'h0aba0001),
			prot: 8'h01, // ICMP
			tos: 0,
			tsp: eth_types::htons(16'h0008), tdp: eth_types::htons(16'h0000),
			
			default: '0 // only "valid" is unset
		},
		
		error: 0
	},
	
	'{ // ICMP packet - too short
	
		packet: { << {{ << byte { // the packet is shortened to the minimum length to just be OF 1.0 parseable
			'h8eabbf392981002590a9b08c080045000054971440004001c6080abac8170aba000108
		}}}},
		
		tuser: '{in: '{port: 5, vport: 3}},
		
		tuple: '{
		
			port: 5, vport: 3,
			dmac: eth_types::htonl48(48'h8eabbf392981), smac: eth_types::htonl48(48'h002590a9b08c),
			typ: eth_types::htons(16'h0800),
			vid: 0, pcp: 0,
			sip: eth_types::htonl(32'h0abac817), dip: eth_types::htonl(32'h0aba0001),
			prot: 8'h01, // ICMP
			tos: 0,
			tsp: eth_types::htons(16'h0008), tdp: eth_types::htons(16'h0000),
			
			default: '0 // only "valid" is unset
		},
		
		error: 1
	},
	
	'{ // TCP packet
	
		packet: { << {{ << byte { // the packet is shortened to the minimum length to just be OF 1.0 parseable
			'h8eabbf392981002590a9b08c08004500003c294c4000400633e40abac8170aba0001cc700016
		}}}},
		
		tuser: '{in: '{port: 5, vport: 3}},
		
		tuple: '{
		
			port: 5, vport: 3,
			dmac: eth_types::htonl48(48'h8eabbf392981), smac: eth_types::htonl48(48'h002590a9b08c),
			typ: eth_types::htons(16'h0800),
			vid: 0, pcp: 0,
			sip: eth_types::htonl(32'h0abac817), dip: eth_types::htonl(32'h0aba0001),
			prot: 8'h06, // TCP
			tos: 0,
			tsp: eth_types::htons(16'hcc70), tdp: eth_types::htons(16'h0016),
			
			default: '0 // only "valid" is unset
		},
		
		error: 0
	},
	
	'{ // TCP packet - too short
	
		packet: { << {{ << byte { // the packet is shortened to the minimum length to just be OF 1.0 parseable
			'h8eabbf392981002590a9b08c08004500003c294c4000400633e40abac8170aba0001cc7000
		}}}},
		
		tuser: '{in: '{port: 5, vport: 3}},
		
		tuple: '{
		
			port: 5, vport: 3,
			dmac: eth_types::htonl48(48'h8eabbf392981), smac: eth_types::htonl48(48'h002590a9b08c),
			typ: eth_types::htons(16'h0800),
			vid: 0, pcp: 0,
			sip: eth_types::htonl(32'h0abac817), dip: eth_types::htonl(32'h0aba0001),
			prot: 8'h06, // TCP
			tos: 0,
			tsp: eth_types::htons(16'hcc70), tdp: eth_types::htons(16'h0016),
			
			default: '0 // only "valid" is unset
		},
		
		error: 1
	},
	
	'{ // TCP packet with 1 option word
	
		packet: { << {{ << byte { // the packet is shortened to the minimum length to just be OF 1.0 parseable
			'h8eabbf392981002590a9b08c08004600003c294c4000400633e40abac8170aba000112345678cc700016
		}}}},
		
		tuser: '{in: '{port: 5, vport: 3}},
		
		tuple: '{
		
			port: 5, vport: 3,
			dmac: eth_types::htonl48(48'h8eabbf392981), smac: eth_types::htonl48(48'h002590a9b08c),
			typ: eth_types::htons(16'h0800),
			vid: 0, pcp: 0,
			sip: eth_types::htonl(32'h0abac817), dip: eth_types::htonl(32'h0aba0001),
			prot: 8'h06, // TCP
			tos: 0,
			tsp: eth_types::htons(16'hcc70), tdp: eth_types::htons(16'h0016),
			
			default: '0 // only "valid" is unset
		},
		
		error: 0
	},
	
	'{ // TCP packet with 1 option word - 8 bytes too small
	
		packet: { << {{ << byte { // the packet is shortened to the minimum length to just be OF 1.0 parseable
			'h8eabbf392981002590a9b08c08004600003c294c4000400633e40abac8170aba0001
		}}}},
		
		tuser: '{in: '{port: 5, vport: 3}},
		
		tuple: '{
		
			port: 5, vport: 3,
			dmac: eth_types::htonl48(48'h8eabbf392981), smac: eth_types::htonl48(48'h002590a9b08c),
			typ: eth_types::htons(16'h0800),
			vid: 0, pcp: 0,
			sip: eth_types::htonl(32'h0abac817), dip: eth_types::htonl(32'h0aba0001),
			prot: 8'h06, // TCP
			tos: 0,
			tsp: eth_types::htons(16'hcc70), tdp: eth_types::htons(16'h0016),
			
			default: '0 // only "valid" is unset
		},
		
		error: 1
	},
	
	'{ // TCP packet with 5 option words
	
		packet: { << {{ << byte { // the packet is shortened to the minimum length to just be OF 1.0 parseable
			'h8eabbf392981002590a9b08c08004a00003c294c4000400633e40abac8170aba00011234567812345678123456781234567812345678cc700016
		}}}},
		
		tuser: '{in: '{port: 5, vport: 3}},
		
		tuple: '{
		
			port: 5, vport: 3,
			dmac: eth_types::htonl48(48'h8eabbf392981), smac: eth_types::htonl48(48'h002590a9b08c),
			typ: eth_types::htons(16'h0800),
			vid: 0, pcp: 0,
			sip: eth_types::htonl(32'h0abac817), dip: eth_types::htonl(32'h0aba0001),
			prot: 8'h06, // TCP
			tos: 0,
			tsp: eth_types::htons(16'hcc70), tdp: eth_types::htons(16'h0016),
			
			default: '0 // only "valid" is unset
		},
		
		error: 0
	},
	
	'{ // TCP packet with 10 option words
	
		packet: { << {{ << byte { // the packet is shortened to the minimum length to just be OF 1.0 parseable
			'h8eabbf392981002590a9b08c08004f00003c294c4000400633e40abac8170aba000112345678123456781234567812345678123456781234567812345678123456781234567812345678cc700016
		}}}},
		
		tuser: '{in: '{port: 5, vport: 3}},
		
		tuple: '{
		
			port: 5, vport: 3,
			dmac: eth_types::htonl48(48'h8eabbf392981), smac: eth_types::htonl48(48'h002590a9b08c),
			typ: eth_types::htons(16'h0800),
			vid: 0, pcp: 0,
			sip: eth_types::htonl(32'h0abac817), dip: eth_types::htonl(32'h0aba0001),
			prot: 8'h06, // TCP
			tos: 0,
			tsp: eth_types::htons(16'hcc70), tdp: eth_types::htons(16'h0016),
			
			default: '0 // only "valid" is unset
		},
		
		error: 0
	},
	
	'{ // TCP packet with VLAN
	
		packet: { << {{ << byte { // the packet is shortened to the minimum length to just be OF 1.0 parseable
			'h8eabbf392981002590a9b08c810000c808004500003c9d0640004006c0290abac8170aba0001cc760016
		}}}},
		
		tuser: '{in: '{port: 5, vport: 3}},
		
		tuple: '{
		
			port: 5, vport: 3,
			dmac: eth_types::htonl48(48'h8eabbf392981), smac: eth_types::htonl48(48'h002590a9b08c),
			typ: eth_types::htons(16'h0800),
			vid: 'd200 /* not in network byte order! */, pcp: 0,
			sip: eth_types::htonl(32'h0abac817), dip: eth_types::htonl(32'h0aba0001),
			prot: 8'h06, // TCP
			tos: 0,
			tsp: eth_types::htons(16'hcc76), tdp: eth_types::htons(16'h0016),
			
			default: '0 // only "valid" is unset
		},
		
		error: 0
	}
	
	};
	
	/*
	vlog +incdir+/home/jsn/repos/upb_parser/pgotfnetworking-netfpga/lib/hw/contrib/pcores/nf10_upb_ofswitch_v1_00_a/hdl/SystemVerilog -L mtiAvm -L mtiOvm -L mtiUvm -L mtiUPF /home/jsn/repos/upb_parser/pgotfnetworking-netfpga/lib/hw/contrib/pcores/nf10_upb_ofswitch_v1_00_a/hdl/SystemVerilog/upb_parser_tb.v ; vlog +incdir+/home/jsn/repos/upb_parser/pgotfnetworking-netfpga/lib/hw/contrib/pcores/nf10_upb_ofswitch_v1_00_a/hdl/SystemVerilog -L mtiAvm -L mtiOvm -L mtiUvm -L mtiUPF /home/jsn/repos/upb_parser/pgotfnetworking-netfpga/lib/hw/contrib/pcores/nf10_upb_ofswitch_v1_00_a/hdl/SystemVerilog/upb_parser.v ; restart -f ; run 1 us
	
	*/

	logic clk = '0;
	
	always begin
		#3.333 begin 
			clk = ~clk;
		end
	end
	
	logic reset = 0;
	
	logic [axis_data_width-1:0] m_axis_tdata;
	logic [axis_tkeep_width-1:0] m_axis_tkeep;
	axis_defs::tuser_t m_axis_tuser;
	logic m_axis_tlast;
	logic m_axis_tvalid = 0;
	wire m_axis_tready;
	
	wire valid, error;
	tuple_t tuple;
	
	upb_parser #(
		
		.axis_data_width		(axis_data_width)
		
	) upb_parser_0 (
		
		.clk(clk),
		.reset(reset),
		
		.s_axis_tdata(m_axis_tdata),
		.s_axis_tkeep(m_axis_tkeep),
		.s_axis_tuser(m_axis_tuser),
		.s_axis_tlast(m_axis_tlast),
		.s_axis_tvalid(m_axis_tvalid),
		.s_axis_tready(m_axis_tready),
		
		.valid(valid),
		.error(error),
		.tuple(tuple)
	);
	
	class mailbox_t;
		test_data_t test_data;
	endclass
	
	mailbox #(mailbox_t) test_data_mb;
	
	task automatic send_packet(
		input test_data_t test_data
	);

		int packet_size_bytes = $size(test_data.packet) / 8;
		int sent_packet_byte = 0;
		
		automatic mailbox_t mb_data;
		mb_data = new();
		mb_data.test_data = test_data;
		test_data_mb.put(mb_data);
		
		forever begin
			m_axis_tlast <= 0;
			m_axis_tkeep <= 0;
			m_axis_tvalid <= 1;
			m_axis_tuser <= test_data.tuser;
			m_axis_tdata <= 'x;
		
			for (int i = 0; i < axis_tkeep_width; i++) begin
			
				if (sent_packet_byte < packet_size_bytes) begin
					for (int j = 0; j < 8; j++) begin
						m_axis_tdata[i * 8 + j] <= test_data.packet[sent_packet_byte * 8 + j];
					end
					m_axis_tkeep[i] <= 1'b1;
					sent_packet_byte++;
				end
			end
			
			if (sent_packet_byte == packet_size_bytes) begin
				m_axis_tlast <= 1;
			end
			
			@(posedge clk);
		
			while (m_axis_tready != 1)
				@(posedge clk); // wait further clock cycles if tready is not set
			
			if (sent_packet_byte == packet_size_bytes) begin
				break;
			end
		end
			
		m_axis_tvalid <= 1'b0;
		m_axis_tdata <= 'x;
		m_axis_tlast <= 'x;
		m_axis_tkeep <= 'x;
		m_axis_tuser <= 'x;

	
	endtask
	
	initial begin
	
		test_data_mb = new();
		
		@(posedge clk);
	
		fork
		
			foreach (packets[n]) begin
				send_packet(packets[n]);
			end
			
			forever begin

				static int packet_count = 0;
				
				automatic mailbox_t mb_data;
				test_data_mb.get(mb_data);
			
				packet_count++;
				
				while (!valid)
					@(posedge clk);
				
				if (error != mb_data.test_data.error) begin
				
					$display("Error: Different error bits for packet %d. Test data: %d, parser: %d", packet_count, mb_data.test_data.error, error);
					
				end else begin
				
					if (error) begin
					
						$display("Parser detected invalid packet %d (correct behaviour)", packet_count);
					
					end else begin
					
						if (tuple == mb_data.test_data.tuple) begin
							$display("Received correct tuple for packet %d", packet_count);
						end else begin
							$display("Error: Tuple for packet %d does not match:", packet_count);
							
							if (tuple.port != mb_data.test_data.tuple.port)
								$display("tuple.port is %d but should be %d", tuple.port, mb_data.test_data.tuple.port);
							if (tuple.vport != mb_data.test_data.tuple.vport)
								$display("tuple.vport is %d but should be %d", tuple.vport, mb_data.test_data.tuple.vport);
							if (tuple.dmac != mb_data.test_data.tuple.dmac)
								$display("tuple.dmac is %d but should be %d", tuple.dmac, mb_data.test_data.tuple.dmac);
							if (tuple.smac != mb_data.test_data.tuple.smac)
								$display("tuple.smac is %d but should be %d", tuple.smac, mb_data.test_data.tuple.smac);
							if (tuple.typ != mb_data.test_data.tuple.typ)
								$display("tuple.typ is %d but should be %d", tuple.typ, mb_data.test_data.tuple.typ);
							if (tuple.vid != mb_data.test_data.tuple.vid)
								$display("tuple.vid is %d but should be %d", tuple.vid, mb_data.test_data.tuple.vid);
							if (tuple.pcp != mb_data.test_data.tuple.pcp)
								$display("tuple.pcp is %d but should be %d", tuple.pcp, mb_data.test_data.tuple.pcp);
							if (tuple.sip != mb_data.test_data.tuple.sip)
								$display("tuple.sip is %d but should be %d", tuple.sip, mb_data.test_data.tuple.sip);
							if (tuple.dip != mb_data.test_data.tuple.dip)
								$display("tuple.dip is %d but should be %d", tuple.dip, mb_data.test_data.tuple.dip);
							if (tuple.prot != mb_data.test_data.tuple.prot)
								$display("tuple.prot is %d but should be %d", tuple.prot, mb_data.test_data.tuple.prot);
							if (tuple.tos != mb_data.test_data.tuple.tos)
								$display("tuple.tos is %d but should be %d", tuple.tos, mb_data.test_data.tuple.tos);
							if (tuple.tsp != mb_data.test_data.tuple.tsp)
								$display("tuple.tsp is %d but should be %d", tuple.tsp, mb_data.test_data.tuple.tsp);
							if (tuple.tdp != mb_data.test_data.tuple.tdp)
								$display("tuple.tdp is %d but should be %d", tuple.tdp, mb_data.test_data.tuple.tdp);
						end
					end
				end
				
				@(posedge clk);
			end
		
		join
	
	end
	

endmodule
