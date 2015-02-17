/*
 * UPB AXI-4 Stream ChipScope core
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

module nf10_upb_axi_stream_chipscope #(

	parameter registered_bus = 1,
	parameter registered_chipscope = 0,
	parameter axis_data_width = 256,
	parameter axis_tkeep_width = 32,
	parameter axis_tuser_in_port_width = 3,
	parameter axis_tuser_out_port_width = 8,
	parameter axis_tuser_packet_length_width = 14

) (

	input wire clk,

	input wire reset,
	input wire axis_tvalid,
	input wire axis_tready,
	input wire [(axis_data_width-1):0] axis_tdata,
	input wire [(axis_tkeep_width-1):0] axis_tkeep,
	input wire axis_tlast,
	input wire [(axis_tuser_packet_length_width-1):0] axis_tuser_packet_length,
	input wire [(axis_tuser_in_port_width-1):0] axis_tuser_in_port,
	input wire [(axis_tuser_in_port_width-1):0] axis_tuser_in_vport,
	input wire [(axis_tuser_out_port_width-1):0] axis_tuser_out_port,
	input wire [(axis_tuser_out_port_width-1):0] axis_tuser_out_vport,

	inout wire [35:0] chipscope_control
);

reg reset_reg;
reg axis_tvalid_reg;
reg axis_tready_reg;
reg [(axis_data_width-1):0] axis_tdata_reg;
reg [(axis_tkeep_width-1):0] axis_tkeep_reg;
reg axis_tlast_reg;
reg [(axis_tuser_packet_length_width-1):0] axis_tuser_packet_length_reg;
reg [(axis_tuser_in_port_width-1):0] axis_tuser_in_port_reg;
reg [(axis_tuser_in_port_width-1):0] axis_tuser_in_vport_reg;
reg [(axis_tuser_out_port_width-1):0] axis_tuser_out_port_reg;
reg [(axis_tuser_out_port_width-1):0] axis_tuser_out_vport_reg;

generate
	if (registered_bus == 1) begin
		always_ff @(posedge clk) begin
			reset_reg <= reset;
			axis_tvalid_reg <= axis_tvalid;
			axis_tready_reg <= axis_tready;
			axis_tdata_reg <= axis_tdata;
			axis_tkeep_reg <= axis_tkeep;
			axis_tlast_reg <= axis_tlast;
			axis_tuser_packet_length_reg <= axis_tuser_packet_length;
			axis_tuser_in_port_reg <= axis_tuser_in_port;
			axis_tuser_in_vport_reg <= axis_tuser_in_vport;
			axis_tuser_out_port_reg <= axis_tuser_out_port;
			axis_tuser_out_vport_reg <= axis_tuser_out_vport;
		end
	end else begin
		always_comb begin
			reset_reg <= reset;
			axis_tvalid_reg <= axis_tvalid;
			axis_tready_reg <= axis_tready;
			axis_tdata_reg <= axis_tdata;
			axis_tkeep_reg <= axis_tkeep;
			axis_tlast_reg <= axis_tlast;
			axis_tuser_packet_length_reg <= axis_tuser_packet_length;
			axis_tuser_in_port_reg <= axis_tuser_in_port;
			axis_tuser_in_vport_reg <= axis_tuser_in_vport;
			axis_tuser_out_port_reg <= axis_tuser_out_port;
			axis_tuser_out_vport_reg <= axis_tuser_out_vport;
		end
	end
endgenerate

wire axis_error;
axis_conform_check_pkg::axi_errors_t axis_errors;

axis_conform_check axis_conform_check_0 (

	.clk(clk),
	.reset(reset_reg),
	.axis_tkeep(axis_tkeep_reg),
	.axis_tlast(axis_tlast_reg),
	.axis_tvalid(axis_tvalid_reg),
	.axis_tready(axis_tready_reg),

	.error(axis_error), // 1 bit error signal
	.error_vec(axis_errors) // 5 bit error code output
);

wire [511:0] data = {
	axis_errors,
	axis_tuser_out_vport_reg,
	axis_tuser_out_port_reg,
	axis_tuser_in_vport_reg,
	axis_tuser_in_port_reg,
	axis_tuser_packet_length_reg,
	axis_tlast_reg,
	axis_tkeep_reg,
	axis_tdata_reg,
	axis_tready_reg,
	axis_tvalid_reg,
	reset_reg
};

wire [15:0] trig = {
	axis_error,
	axis_tready_reg,
	axis_tvalid_reg,
	reset_reg
};

reg [511:0] data_reg;
reg [15:0] trig_reg;

generate
	if (registered_chipscope == 1) begin
		always @(posedge clk) begin
			data_reg <= data;
			trig_reg <= trig;
		end
	end else begin
		always @(*) begin
			data_reg <= data;
			trig_reg <= trig;
		end
	end
endgenerate

chipscope_ila_512 axi_stream_ila_inst (
	.CONTROL(chipscope_control),
	.CLK(clk),
	.DATA(data_reg),
	.TRIG0(trig_reg)
);

endmodule

module chipscope_ila_512 (
	inout [35:0] CONTROL,
	input CLK,
	input [511:0] DATA,
	input [15:0] TRIG0
);
endmodule

