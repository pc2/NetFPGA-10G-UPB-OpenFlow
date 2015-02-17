/*
 * UPB AXI Stream interface
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

package axis_defs;

	localparam default_axis_data_width = 256;
	localparam max_packet_length = 16383;
	localparam packet_length_width = $clog2(max_packet_length);
	
	// incoming port definition
	typedef logic [2:0] in_rport_t;
	typedef logic [2:0] in_vport_t;
	
	typedef struct packed {
	
		in_rport_t port;			// binary encoded incoming (hardware) port
		in_vport_t vport;			// binary encoded incoming virtual port
		
	} in_port_t;
	
	// outgoing port definition
	typedef logic [7:0] out_rport_t;
	typedef logic [7:0] out_vport_t;
	
	typedef struct packed {
	
		out_rport_t port;			// bit field encoded outgoing (hardware) ports
		out_vport_t vport;		// bit field encoded outgoing virtual port
		
	} out_port_t;
	
	// definition of the tuser signal
	typedef struct packed {
	
		in_port_t in;
		out_port_t out;
		
		logic [packet_length_width-1:0] packet_length;
		
	} tuser_t;
	
	localparam tuser_size = $bits(tuser_t);

endpackage

interface axis_if;

	import axis_defs::*;
	
	logic [default_axis_data_width-1:0] tdata;
	logic [default_axis_data_width/8-1:0] tkeep;
	logic tvalid, tready, tlast;
	tuser_t tuser;
	
	modport master (
		
		output tdata, tkeep, tvalid, tlast, tuser,
		input tready
	
	);
	
	modport slave (
	
		input tdata, tkeep, tvalid, tlast, tuser,
		output tready
		
	);

endinterface
