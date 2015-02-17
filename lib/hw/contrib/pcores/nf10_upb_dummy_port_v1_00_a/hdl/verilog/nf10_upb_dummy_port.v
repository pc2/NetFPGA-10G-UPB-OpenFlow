/*
 * UPB Dummy Port core
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

module nf10_upb_dummy_port #(

	parameter peripheral_port_width = 32,
	parameter external_port_width = 32
	
) (
	
	input wire [(peripheral_port_width-1):0] peripheral_connection_I,
	output wire [(peripheral_port_width-1):0] peripheral_connection_O,
	input wire [(peripheral_port_width-1):0] peripheral_connection_T,

	input wire [(external_port_width-1):0] external_connection_I,
	output wire [(external_port_width-1):0] external_connection_O,
	output wire [(external_port_width-1):0] external_connection_T

);

assign external_connection_O = peripheral_connection_I;
assign external_connection_T = peripheral_connection_T;
assign peripheral_connection_O = external_connection_I;

endmodule

