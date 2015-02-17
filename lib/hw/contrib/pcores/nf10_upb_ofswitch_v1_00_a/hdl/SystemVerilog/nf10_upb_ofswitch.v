/*
 * Copyright (c) 2014, 2015 Felix Wallaschek
 * felix@elektronenversand.de
 *
 * This file is part of the NetFPGA 10G UPB OpenFlow Switch project:
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

`include "parameters.v"
`include "tuple_t.v"
/*
 * The UPB OpenFlow Switch
 *
 *
 * Please note: When adding another lookup engine or increasing the size
 * of one, make sure to set the the match address offsets to 
 * their correct values. Also make sure that all adresses plus their 
 * respective offsets do fit into a bit vector of length C_MATCH_ADDR_WIDTH.
 * C_MATCH_ADDR_WIDTH is defined in parameters.v
 *
 * Example:
 * Static Lookup Engine with 8 entries
 * TCAM Lookup with 64 possible entries
 * CAM Lookup with 2048 possible entries
 * VPort Lookup with 8 possible entries
 *
 * Offset for match_addr outputs:
 * Static Lookup: 0
 * TCAM Lookup: 8
 * CAM Lookup: 8 + 64 = 72
 * VPort Lookup: 72 + 2048 = 2120
 * Overall address space: 0 - 2128
 * Minimal size of C_MATCH_ADDR_WIDTH: clog2(2128) = 12
 *
 * AXI addresses are built as follows:
 *    4 bit adress of module
 *   28 bit internal address
 * 
 * The statistics address for a flow is build as follows:
 * 0xC0000000 | ((MATCH_ADDR + OFFSET) << 4)
 * Where MATCH_ADDR is the address of the flow entry in its respective
 * Lookup module and OFFSET the offset for that module.
 * The bits [4:3] define the type of statistic that is read:
 * 00: number of packets matched by that flow
 * 01: number of bytes matched by that flow
 * 10: timestamp of last match by that flow
 * 11: undefined
 *
 * To read the current timestamp of the card read address:
 * 0xC0000000 | ((2**(C_MATCH_ADDR_WIDTH)+1) << 4)
 * (For C_MATCH_ADDR_WIDTH=12: 0xC0010010)
 */
module nf10_upb_ofswitch #(
    //Change these Values in the Synplify .prj file!
    // Also you might want to have a look at the parameters.v file.
	parameter C_AXI_BASE_ADDR_TCAM = 32'hFFFFFFFF,
	parameter C_AXI_HIGH_ADDR_TCAM = 32'h00000000,
	parameter C_AXI_BASE_ADDR_CAM = 32'hFFFFFFFF,
	parameter C_AXI_HIGH_ADDR_CAM = 32'h00000000,
	parameter C_AXI_BASE_ADDR_STATS = 32'hFFFFFFFF,
	parameter C_AXI_HIGH_ADDR_STATS = 32'h00000000,
	parameter C_DMA_PORT = 0,
	parameter C_DMA_FIRST_EXTERNAL_PORT = 0,
	parameter C_DMA_LAST_EXTERNAL_PORT = 0,
	parameter C_BRIDGED_ETH_A_VPORT = 0,
	parameter C_BRIDGED_ETH_B_VPORT = 0,
	parameter TCAM_DEPTH = 0,
    parameter CAM_DEPTH = 0
    ) (
    input clk,
    input reset,
    
    input [C_AXIS_TDATA_WIDTH-1:0] s_axis_tdata,
    input [C_AXIS_TKEEP_WIDTH-1:0] s_axis_tkeep,
    input s_axis_tvalid,
    input s_axis_tlast,
    input [C_IN_PORT_WIDTH-1:0] s_axis_tuser_in_port,
    input [C_IN_PORT_WIDTH-1:0] s_axis_tuser_in_vport,
    input [C_OUT_PORT_WIDTH-1:0] s_axis_tuser_out_port,
    input [C_OUT_PORT_WIDTH-1:0] s_axis_tuser_out_vport,
    input [C_PACKET_LENGTH_WIDTH-1:0] s_axis_tuser_packet_length,
    output s_axis_tready,
    
    
    output reg [C_AXIS_TDATA_WIDTH-1:0] m_axis_tdata,
    output reg [C_AXIS_TKEEP_WIDTH-1:0] m_axis_tkeep,
    output reg m_axis_tvalid,
    output reg m_axis_tlast,
    output reg [C_IN_PORT_WIDTH-1:0] m_axis_tuser_in_port,
    output reg [C_IN_PORT_WIDTH-1:0] m_axis_tuser_in_vport,
    output reg [C_OUT_PORT_WIDTH-1:0] m_axis_tuser_out_port,
    output reg [C_OUT_PORT_WIDTH-1:0] m_axis_tuser_out_vport,
    output reg [C_PACKET_LENGTH_WIDTH-1:0] m_axis_tuser_packet_length,
    input m_axis_tready,
    
    input s_axi_stats_aclk,
    input s_axi_stats_aresetn,
    input s_axi_stats_arvalid,
    input [31:0] s_axi_stats_araddr,
    input [2:0] s_axi_stats_arprot,
    input s_axi_stats_rready,
    output s_axi_stats_arready, 
    output s_axi_stats_rvalid, 
    output [31:0] s_axi_stats_rdata,
    output [1:0] s_axi_stats_rresp,
    
    input s_axi_tcam_aclk,
    input s_axi_tcam_aresetn,
    input s_axi_tcam_awvalid,
    output reg s_axi_tcam_awready,
    input [31:0] s_axi_tcam_awaddr,
    input [2:0] s_axi_tcam_awprot,
    input s_axi_tcam_wvalid,
    output s_axi_tcam_wready,
    input [31:0] s_axi_tcam_wdata,
    input [3:0] s_axi_tcam_wstrb,
    output s_axi_tcam_bvalid,
    input s_axi_tcam_bready,
    output reg [1:0] s_axi_tcam_bresp,
    input s_axi_tcam_arvalid,
    output reg s_axi_tcam_arready,
    input [31:0] s_axi_tcam_araddr,
    input [2:0] s_axi_tcam_arprot,
    output reg s_axi_tcam_rvalid,
    input s_axi_tcam_rready,
    output reg [31:0] s_axi_tcam_rdata,
    output reg [1:0] s_axi_tcam_rresp,
    
    input s_axi_cam_aclk,
    input s_axi_cam_aresetn,
    input s_axi_cam_awvalid,
    output reg s_axi_cam_awready,
    input [31:0] s_axi_cam_awaddr,
    input [2:0] s_axi_cam_awprot,
    input s_axi_cam_wvalid,
    output s_axi_cam_wready,
    input [31:0] s_axi_cam_wdata,
    input [3:0] s_axi_cam_wstrb,
    output s_axi_cam_bvalid,
    input s_axi_cam_bready,
    output reg [1:0] s_axi_cam_bresp,
    input s_axi_cam_arvalid,
    output reg s_axi_cam_arready,
    input [31:0] s_axi_cam_araddr,
    input [2:0] s_axi_cam_arprot,
    output reg s_axi_cam_rvalid,
    input s_axi_cam_rready,
    output reg [31:0] s_axi_cam_rdata,
    output reg [1:0] s_axi_cam_rresp
);

logic valid;

logic fifo_empty;
wire action_0_match;
wire action_0_valid;
wire[1:0] action_0_type;
wire[C_OUT_PORT_WIDTH-1:0] action_0_port;
wire[C_OUT_PORT_WIDTH-1:0] action_0_vport;
wire[C_MATCH_ADDR_WIDTH-1:0] action_0_match_addr;
wire[C_MATCH_ADDR_WIDTH-1:0] action_0_match_addr_pre_offst;
wire action_0_match_d;
wire action_0_valid_d;
wire[1:0] action_0_type_d;
wire[C_OUT_PORT_WIDTH-1:0] action_0_port_d;
wire[C_OUT_PORT_WIDTH-1:0] action_0_vport_d;
wire[C_MATCH_ADDR_WIDTH-1:0] action_0_match_addr_d;
wire action_1_match;
wire action_1_valid;
wire[1:0] action_1_type;
wire[C_OUT_PORT_WIDTH-1:0] action_1_port;
wire[C_OUT_PORT_WIDTH-1:0] action_1_vport;
wire[C_MATCH_ADDR_WIDTH-1:0] action_1_match_addr;
wire[C_MATCH_ADDR_WIDTH-1:0] action_1_match_addr_pre_offst;
wire action_2_match;
wire action_2_valid;
wire[1:0] action_2_type;
wire[C_OUT_PORT_WIDTH-1:0] action_2_port;
wire[C_OUT_PORT_WIDTH-1:0] action_2_vport;
wire[C_MATCH_ADDR_WIDTH-1:0] action_2_match_addr;
wire[C_MATCH_ADDR_WIDTH-1:0] action_2_match_addr_pre_offst;
wire action_2_match_d;
wire action_2_valid_d;
wire[1:0] action_2_type_d;
wire[C_OUT_PORT_WIDTH-1:0] action_2_port_d;
wire[C_OUT_PORT_WIDTH-1:0] action_2_vport_d;
wire[C_MATCH_ADDR_WIDTH-1:0] action_2_match_addr_d;
wire action_3_match;
wire action_3_valid;
wire[1:0] action_3_type;
wire[C_OUT_PORT_WIDTH-1:0] action_3_port;
wire[C_OUT_PORT_WIDTH-1:0] action_3_vport;
wire[C_MATCH_ADDR_WIDTH-1:0] action_3_match_addr;
wire[C_MATCH_ADDR_WIDTH-1:0] action_3_match_addr_pre_offst;
wire action_3_match_d;
wire action_3_valid_d;
wire[1:0] action_3_type_d;
wire[C_OUT_PORT_WIDTH-1:0] action_3_port_d;
wire[C_OUT_PORT_WIDTH-1:0] action_3_vport_d;
wire[C_MATCH_ADDR_WIDTH-1:0] action_3_match_addr_d;
wire action_selected_match;
wire action_selected_valid;
wire[1:0] action_selected_type;
wire[C_OUT_PORT_WIDTH-1:0] action_selected_port;
wire[C_OUT_PORT_WIDTH-1:0] action_selected_vport;
wire[C_MATCH_ADDR_WIDTH-1:0] action_selected_match_addr;


tuple_t parser_out;



wire[C_IN_PORT_WIDTH-1:0] fifo_out_tuser_in_port;
wire[C_IN_PORT_WIDTH-1:0] fifo_out_tuser_in_vport;
wire[C_AXIS_TDATA_WIDTH-1:0] fifo_out_tdata;
wire[C_PACKET_LENGTH_WIDTH-1:0] fifo_out_tuser_packet_length;
wire[C_AXIS_TKEEP_WIDTH-1:0] fifo_out_tkeep;
wire fifo_out_tlast;
wire fifo_out_tvalid;
wire fifo_out_tready;

/*
 *  FIFO to store packets until we have decided what to do with them
 */
fifo fif(
    .clk(clk),
    .din({s_axis_tuser_in_port,s_axis_tuser_in_vport,s_axis_tdata,s_axis_tuser_packet_length,s_axis_tkeep,s_axis_tlast,s_axis_tvalid}),
    .dout({fifo_out_tuser_in_port,fifo_out_tuser_in_vport,fifo_out_tdata,fifo_out_tuser_packet_length,fifo_out_tkeep,fifo_out_tlast,fifo_out_tvalid}),
    .rd_en(fifo_out_tready || ~fifo_out_tvalid),
    .wr_en(s_axis_tvalid && s_axis_tready),
    .empty(fifo_empty),
    .rst(reset)
);

/*
 *  Parser to parse AXI-Stream into OpenFlow tuples
 */

wire tuple_valid, tuple_error;
tuple_t tuple;

axis_defs::tuser_t s_axis_tuser = '{ 
	in:				'{port: s_axis_tuser_in_port, vport: s_axis_tuser_in_vport},
	out:			'{port: s_axis_tuser_out_port, vport: s_axis_tuser_out_vport},
	packet_length:	s_axis_tuser_packet_length
};

upb_parser upb_parser_0 (
    .clk(clk),
    .reset(reset),
    .s_axis_tdata(s_axis_tdata),
    .s_axis_tkeep(s_axis_tkeep),
	.s_axis_tuser(s_axis_tuser),
    .s_axis_tlast(s_axis_tlast),
    .s_axis_tvalid(s_axis_tvalid),
    .s_axis_tready(s_axis_tready),
	.valid(tuple_valid),
	.error(tuple_error),
    .tuple(tuple)
);

// TODO: Forward and process tuple_error; on error: Drop packet
// TODO: Get valid out of the tuple

always_comb begin
	parser_out = tuple;
	parser_out.valid = tuple_valid;
end

/*
 *  Static Lookup Module
 */
static_lookup #(
	.C_DMA_PORT(C_DMA_PORT),
	.C_DMA_FIRST_EXTERNAL_PORT(C_DMA_FIRST_EXTERNAL_PORT),
	.C_DMA_LAST_EXTERNAL_PORT(C_DMA_LAST_EXTERNAL_PORT),
	.C_BRIDGED_ETH_A_VPORT(C_BRIDGED_ETH_A_VPORT),
	.C_BRIDGED_ETH_B_VPORT(C_BRIDGED_ETH_B_VPORT)
) lookup (
    .clk(clk),
    .reset(reset),
    .tuple_in(parser_out),
    .action_match(action_0_match),
    .action_match_addr(action_0_match_addr_pre_offst),
    .action_valid(action_0_valid),
    .action_port(action_0_port),
    .action_type(action_0_type),
    .action_vport(action_0_vport)
);
assign action_0_match_addr = action_0_match_addr_pre_offst + 0;

action_delay #(7,C_OUT_PORT_WIDTH*2+2+C_MATCH_ADDR_WIDTH+2) act_delay ( // Static Lookup is much faster than tcam. Delay the signal here to synchronize match signals.
    .clk(clk),
    .reset(reset),
    .inp({action_0_type,action_0_valid,action_0_match,action_0_port,action_0_vport,action_0_match_addr}),
    .outp({action_0_type_d,action_0_valid_d,action_0_match_d,action_0_port_d,action_0_vport_d,action_0_match_addr_d})
);

/*
 *  TCAM Lookup Module
 */
upb_tcam_lookup #(
	.C_AXI_BASE_ADDR(C_AXI_BASE_ADDR_TCAM),
	.C_AXI_HIGH_ADDR(C_AXI_HIGH_ADDR_TCAM),
	.TCAM_DEPTH(TCAM_DEPTH)
) tcam (
    .CLK(clk),
    .RST(reset),
    .tuple(parser_out),
    .action_match(action_1_match),
    .action_valid(action_1_valid),
    .action_match_addr(action_1_match_addr_pre_offst),
    .action_port(action_1_port),
    .action_vport(action_1_vport),
    .action_type(action_1_type),
    
    .s_axi_aclk(s_axi_tcam_aclk),
    .s_axi_aresetn(s_axi_tcam_aresetn),
    .s_axi_awvalid(s_axi_tcam_awvalid),
    .s_axi_awready(s_axi_tcam_awready),
    .s_axi_awaddr(s_axi_tcam_awaddr),
    .s_axi_awprot(s_axi_tcam_awprot),
    .s_axi_wvalid(s_axi_tcam_wvalid),
    .s_axi_wready(s_axi_tcam_wready),
    .s_axi_wdata(s_axi_tcam_wdata),
    .s_axi_wstrb(s_axi_tcam_wstrb),
    .s_axi_bvalid(s_axi_tcam_bvalid),
    .s_axi_bready(s_axi_tcam_bready),
    .s_axi_bresp(s_axi_tcam_bresp),
    .s_axi_arvalid(s_axi_tcam_arvalid),
    .s_axi_arready(s_axi_tcam_arready),
    .s_axi_araddr(s_axi_tcam_araddr),
    .s_axi_arprot(s_axi_tcam_arprot),
    .s_axi_rvalid(s_axi_tcam_rvalid),
    .s_axi_rready(s_axi_tcam_rready),
    .s_axi_rdata(s_axi_tcam_rdata),
    .s_axi_rresp(s_axi_tcam_rresp)
);
assign action_1_match_addr = action_1_match_addr_pre_offst + C_NUM_INPUTS; // Static Lookup engine has C_NUM_INPUTS addresses.

/*
 *  CAM Lookup Module
 */
upb_bram_cam_lookup #(
    .C_AXI_BASE_ADDR(C_AXI_BASE_ADDR_CAM),
    .C_AXI_HIGH_ADDR(C_AXI_HIGH_ADDR_CAM),
    .CAM_DEPTH(CAM_DEPTH)
) cam(
    .CLK(clk),
    .RST(reset),
    .tuple(parser_out),
    .action_match(action_2_match),
    .action_valid(action_2_valid),
    .action_match_addr(action_2_match_addr_pre_offst),
    .action_port(action_2_port),
    .action_vport(action_2_vport),
    .action_type(action_2_type),
    
    .s_axi_aclk(s_axi_cam_aclk),
    .s_axi_aresetn(s_axi_cam_aresetn),
    .s_axi_awvalid(s_axi_cam_awvalid),
    .s_axi_awready(s_axi_cam_awready),
    .s_axi_awaddr(s_axi_cam_awaddr),
    .s_axi_awprot(s_axi_cam_awprot),
    .s_axi_wvalid(s_axi_cam_wvalid),
    .s_axi_wready(s_axi_cam_wready),
    .s_axi_wdata(s_axi_cam_wdata),
    .s_axi_wstrb(s_axi_cam_wstrb),
    .s_axi_bvalid(s_axi_cam_bvalid),
    .s_axi_bready(s_axi_cam_bready),
    .s_axi_bresp(s_axi_cam_bresp),
    .s_axi_arvalid(s_axi_cam_arvalid),
    .s_axi_arready(s_axi_cam_arready),
    .s_axi_araddr(s_axi_cam_araddr),
    .s_axi_arprot(s_axi_cam_arprot),
    .s_axi_rvalid(s_axi_cam_rvalid),
    .s_axi_rready(s_axi_cam_rready),
    .s_axi_rdata(s_axi_cam_rdata),
    .s_axi_rresp(s_axi_cam_rresp)
);
assign action_2_match_addr = action_2_match_addr_pre_offst + C_NUM_INPUTS + TCAM_DEPTH; // Static Lookup engine has C_NUM_INPUTS addresses.

action_delay #(7,C_OUT_PORT_WIDTH*2+2+2+C_MATCH_ADDR_WIDTH) cam_delay ( // CAM Lookup is much faster than tcam. Delay the signal here to synchronize match signals.
    .clk(clk),
    .reset(reset),
    .inp({action_2_type,action_2_valid,action_2_match,action_2_port,action_2_vport,action_2_match_addr}),
    .outp({action_2_type_d,action_2_valid_d,action_2_match_d,action_2_port_d,action_2_vport_d,action_2_match_addr_d})
);

/*
 *  VPort Lookup Module
 */
vport_lookup #(
	.C_DMA_PORT(C_DMA_PORT),
	.C_DMA_FIRST_EXTERNAL_PORT(C_DMA_FIRST_EXTERNAL_PORT),
	.C_DMA_LAST_EXTERNAL_PORT(C_DMA_LAST_EXTERNAL_PORT),
	.C_BRIDGED_ETH_A_VPORT(C_BRIDGED_ETH_A_VPORT),
	.C_BRIDGED_ETH_B_VPORT(C_BRIDGED_ETH_B_VPORT)
) vlookup (
    .clk(clk),
    .reset(reset),
    .tuple_in(parser_out),
    .action_match(action_3_match),
    .action_match_addr(action_3_match_addr_pre_offst),
    .action_valid(action_3_valid),
    .action_port(action_3_port),
    .action_type(action_3_type),
    .action_vport(action_3_vport)
);
assign action_3_match_addr = action_3_match_addr_pre_offst +  C_NUM_INPUTS + TCAM_DEPTH + CAM_DEPTH;

action_delay #(7,C_OUT_PORT_WIDTH*2+2+C_MATCH_ADDR_WIDTH+2) vport_delay ( // Vport Lookup is much faster than tcam. Delay the signal here to synchronize match signals.
    .clk(clk),
    .reset(reset),
    .inp({action_3_type,action_3_valid,action_3_match,action_3_port,action_3_vport,action_3_match_addr}),
    .outp({action_3_type_d,action_3_valid_d,action_3_match_d,action_3_port_d,action_3_vport_d,action_3_match_addr_d})
);

/*
 *  Select which action to use.
 * Note: all valid signals have to be synchronized and have to be set 
 * for exactly one cycle!
 */
selector #(.C_NUM_INPUTS(4)) selector(
    .clk(clk),
    .reset(reset),
    // connect ports with increasing priority
    .port({action_0_port_d,action_1_port,action_2_port_d,action_3_port_d}),
    .vport({action_0_vport_d,action_1_vport,action_2_vport_d,action_3_vport_d}),
    .atype({action_0_type_d,action_1_type,action_2_type_d,action_3_type_d}),
    .match({action_0_match_d,action_1_match,action_2_match_d,action_3_match_d}),
    .match_addr({action_0_match_addr_d,action_1_match_addr,action_2_match_addr_d,action_3_match_addr_d}),
    .valid({action_0_valid_d,action_1_valid,action_2_valid_d,action_3_valid_d}),
    .action_selected_match(action_selected_match),
    .action_selected_valid(action_selected_valid),
    .action_selected_port(action_selected_port),
    .action_selected_vport(action_selected_vport),
    .action_selected_type(action_selected_type),
    .action_selected_match_addr(action_selected_match_addr)
);

/*
 *  Apply selected action to packet in FIFO
 */
processor #(
	.C_AXI_BASE_ADDR(C_AXI_BASE_ADDR_STATS),
	.C_AXI_HIGH_ADDR(C_AXI_HIGH_ADDR_STATS),
	.C_DMA_PORT(C_DMA_PORT),
	.C_DMA_FIRST_EXTERNAL_PORT(C_DMA_FIRST_EXTERNAL_PORT),
	.C_DMA_LAST_EXTERNAL_PORT(C_DMA_LAST_EXTERNAL_PORT),
	.C_BRIDGED_ETH_A_VPORT(C_BRIDGED_ETH_A_VPORT),
	.C_BRIDGED_ETH_B_VPORT(C_BRIDGED_ETH_B_VPORT)

) processor(
    .clk(clk),
    .reset(reset),
    .action_match(action_selected_match),
    .action_valid(action_selected_valid),
    .action_port(action_selected_port),
    .action_vport(action_selected_vport),
    .action_type(action_selected_type),
    .action_match_addr(action_selected_match_addr),
    
    .s_axis_tdata(fifo_out_tdata),
    .s_axis_tready(fifo_out_tready),
    .s_axis_tvalid(fifo_out_tvalid),
    .s_axis_tuser_packet_length(fifo_out_tuser_packet_length),
    .s_axis_tuser_in_port(fifo_out_tuser_in_port),
    .s_axis_tuser_in_vport(fifo_out_tuser_in_vport),
    .s_axis_tlast(fifo_out_tlast),
    .s_axis_tkeep(fifo_out_tkeep),
    
    .m_axis_tdata(m_axis_tdata),
    .m_axis_tready(m_axis_tready),
    .m_axis_tkeep(m_axis_tkeep),
    .m_axis_tvalid(m_axis_tvalid),
    .m_axis_tlast(m_axis_tlast),
    .m_axis_tuser_out_port(m_axis_tuser_out_port),
    .m_axis_tuser_in_port(m_axis_tuser_in_port),
    .m_axis_tuser_in_vport(m_axis_tuser_in_vport),
    .m_axis_tuser_out_vport(m_axis_tuser_out_vport),
    .m_axis_tuser_packet_length(m_axis_tuser_packet_length),
    
    .s_axi_aclk(s_axi_stats_aclk),
    .s_axi_aresetn(s_axi_stats_aresetn),
    .s_axi_arvalid(s_axi_stats_arvalid),
    .s_axi_araddr(s_axi_stats_araddr),
    .s_axi_arprot(s_axi_stats_arprot),
    .s_axi_rready(s_axi_stats_rready),
    .s_axi_arready(s_axi_stats_arready), 
    .s_axi_rvalid(s_axi_stats_rvalid), 
    .s_axi_rdata(s_axi_stats_rdata),
    .s_axi_rresp(s_axi_stats_rresp)
);

endmodule
