/*
 * UPB AXI-4 Lite ChipScope core
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

module nf10_upb_axi_lite_chipscope #(

	parameter C_AXI_ADDR_WIDTH = 32,
	parameter C_AXI_DATA_WIDTH = 32,

	parameter USE_FF_BEFORE_CHIPSCOPE = 1

) (

	input wire axi_aclk,
	input wire axi_aresetn,

	input wire [(C_AXI_DATA_WIDTH-1):0] axi_araddr,
	input wire axi_arvalid,
	input wire axi_arready,

	input wire [(C_AXI_DATA_WIDTH-1):0] axi_rdata,
	input wire [1:0] axi_rresp,
	input wire axi_rvalid,
	input wire axi_rready,


	input wire [C_AXI_ADDR_WIDTH-1:0] axi_awaddr,
	input wire axi_awvalid,
	input wire axi_awready,

	input wire [(C_AXI_DATA_WIDTH-1):0] axi_wdata,
	input wire [((C_AXI_DATA_WIDTH/8)-1):0] axi_wstrb,
	input wire axi_wvalid,
	input wire axi_wready,

	input wire [1:0] axi_bresp,
	input wire axi_bvalid,
	input wire axi_bready,

	inout wire [35:0] chipscope_control
);

wire [255:0] data = {
	axi_aresetn,

	axi_araddr,
	axi_arvalid,
	axi_arready,

	axi_rdata,
	axi_rresp,
	axi_rvalid,
	axi_rready,

	axi_awaddr,
	axi_awvalid,
	axi_awready,

	axi_wdata,
	axi_wstrb,
	axi_wvalid,
	axi_wready,

	axi_bresp,
	axi_bvalid,
	axi_bready
};

wire [15:0] trig = {

	axi_aresetn,
	
	axi_arvalid,
	axi_arready,

	axi_rvalid,
	axi_rready,

	axi_awvalid,
	axi_awready,
	
	axi_wvalid,
	axi_wready,

	axi_bvalid,
	axi_bready

};

reg [255:0] data_reg;
reg [15:0] trig_reg;

generate
	if (USE_FF_BEFORE_CHIPSCOPE == 1) begin
		always @(posedge axi_aclk) begin
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

chipscope_ila_256 axi_lite_ila_inst (
	.CONTROL(chipscope_control),
	.CLK(axi_aclk),
	.DATA(data_reg),
	.TRIG0(trig_reg)
);

endmodule

module chipscope_ila_256 (
	inout [35:0] CONTROL,
	input CLK,
	input [255:0] DATA,
	input [15:0] TRIG0
);
endmodule


