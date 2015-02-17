/*
 * Copyright (c) 2014, 2015 Thomas Löcke
 * tloecke@mail.uni-paderborn.de
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
 * 
 */
`default_nettype none

module nf10_upb_interconnect_wrapper #( 
    parameter C_PORT_NUMBER = 0,
    parameter C_INPORT_WIDTH = 3,
    parameter C_OUTPORT_WIDTH = 8,
    parameter C_PACKET_LENGTH_WIDTH = 14,
    parameter C_MAX_PACKET_LENGTH = 10000, // in byte
    parameter C_AXIS_DATA_WIDTH = 256,
    parameter SIM_GTXRESET_SPEEDUP = 1,
    parameter [31:0] C_AXI_BASE_ADDR = 32'h00000000,
    parameter [31:0] C_AXI_HIGH_ADDR = 32'hFFFFFFFF
)
(
    // input wire arbiter side //
    input wire axi_aclk,
    input wire axi_resetn,    
    
    // Master Stream Ports
    output wire [C_AXIS_DATA_WIDTH-1:0] arbiter_m_axis_tdata,
    output wire [(C_AXIS_DATA_WIDTH/8)-1:0] arbiter_m_axis_tkeep, 
    output wire [C_PACKET_LENGTH_WIDTH-1:0] arbiter_m_axis_tuser_packet_length,
    output reg [C_INPORT_WIDTH-1:0] arbiter_m_axis_tuser_in_port = C_PORT_NUMBER,
    output reg [C_OUTPORT_WIDTH-1:0] arbiter_m_axis_tuser_out_port = 0,
    output reg [C_INPORT_WIDTH-1:0] arbiter_m_axis_tuser_in_vport = 0,
    output reg [C_OUTPORT_WIDTH-1:0] arbiter_m_axis_tuser_out_vport = 0,
    output wire arbiter_m_axis_tvalid,
    input  wire arbiter_m_axis_tready,
    output wire arbiter_m_axis_tlast,
	 
    // output queue side //
    output wire output_queue_clk,
    
    // Slave Stream Ports
    input wire [C_AXIS_DATA_WIDTH-1:0] output_queue_s_axis_tdata,
    input wire [(C_AXIS_DATA_WIDTH/8)-1:0] output_queue_s_axis_tkeep,
    input wire [C_PACKET_LENGTH_WIDTH-1:0] output_queue_s_axis_tuser_packet_length,
    input wire [C_INPORT_WIDTH-1:0] output_queue_s_axis_tuser_in_port,
    input wire [C_OUTPORT_WIDTH-1:0] output_queue_s_axis_tuser_out_port,
    input wire [C_INPORT_WIDTH-1:0] output_queue_s_axis_tuser_in_vport,
    input wire [C_OUTPORT_WIDTH-1:0] output_queue_s_axis_tuser_out_vport,
    input wire output_queue_s_axis_tvalid,
    output wire output_queue_s_axis_tready,
    input wire output_queue_s_axis_tlast,

    input wire RESET, 
    input wire INIT_CLK,
    input wire GT_RESET_IN,
    input wire GTXD8_P,
    input wire GTXD8_N,
    input wire [0:9] RXP,
    input wire [0:9] RXN,
    output wire [0:9] TXP,
    output wire [0:9] TXN,

    // AXI4Lite for reading statistics
    input wire s_axi_aclk,
    input wire s_axi_aresetn,
    input wire s_axi_arvalid,
    output reg s_axi_arready,
    input wire [31:0] s_axi_araddr,
    input wire [2:0] s_axi_arprot,
    output reg s_axi_rvalid,
    input wire s_axi_rready,
    output reg [31:0] s_axi_rdata,
    output reg [1:0] s_axi_rresp
    );

// Instantiate the module
nf10_upb_interconnect #(

	.C_PORT_NUMBER(C_PORT_NUMBER),
	.C_INPORT_WIDTH(C_INPORT_WIDTH),
	.C_OUTPORT_WIDTH(C_OUTPORT_WIDTH),
	.C_PACKET_LENGTH_WIDTH(C_PACKET_LENGTH_WIDTH),
	.C_MAX_PACKET_LENGTH(C_MAX_PACKET_LENGTH),
	.C_AXIS_DATA_WIDTH(C_AXIS_DATA_WIDTH),
	.SIM_GTXRESET_SPEEDUP(SIM_GTXRESET_SPEEDUP),
	.C_AXI_BASE_ADDR(C_AXI_BASE_ADDR),
	.C_AXI_HIGH_ADDR(C_AXI_HIGH_ADDR) 

) interconnect (
    .axi_aclk(axi_aclk), 
    .axi_resetn(axi_resetn), 
    .arbiter_m_axis_tdata(arbiter_m_axis_tdata), 
    .arbiter_m_axis_tkeep(arbiter_m_axis_tkeep), 
    .arbiter_m_axis_tuser_packet_length(arbiter_m_axis_tuser_packet_length), 
    .arbiter_m_axis_tuser_in_port(arbiter_m_axis_tuser_in_port), 
    .arbiter_m_axis_tuser_out_port(arbiter_m_axis_tuser_out_port), 
    .arbiter_m_axis_tuser_in_vport(arbiter_m_axis_tuser_in_vport), 
    .arbiter_m_axis_tuser_out_vport(arbiter_m_axis_tuser_out_vport), 
    .arbiter_m_axis_tvalid(arbiter_m_axis_tvalid), 
    .arbiter_m_axis_tready(arbiter_m_axis_tready), 
    .arbiter_m_axis_tlast(arbiter_m_axis_tlast), 
    .output_queue_clk(output_queue_clk), 
    .output_queue_s_axis_tdata(output_queue_s_axis_tdata), 
    .output_queue_s_axis_tkeep(output_queue_s_axis_tkeep), 
    .output_queue_s_axis_tuser_packet_length(output_queue_s_axis_tuser_packet_length), 
    .output_queue_s_axis_tuser_in_port(output_queue_s_axis_tuser_in_port), 
    .output_queue_s_axis_tuser_out_port(output_queue_s_axis_tuser_out_port), 
    .output_queue_s_axis_tuser_in_vport(output_queue_s_axis_tuser_in_vport), 
    .output_queue_s_axis_tuser_out_vport(output_queue_s_axis_tuser_out_vport), 
    .output_queue_s_axis_tvalid(output_queue_s_axis_tvalid), 
    .output_queue_s_axis_tready(output_queue_s_axis_tready), 
    .output_queue_s_axis_tlast(output_queue_s_axis_tlast), 
    .RESET(RESET), 
    .INIT_CLK(INIT_CLK), 
    .GT_RESET_IN(GT_RESET_IN), 
    .GTXD8_P(GTXD8_P), 
    .GTXD8_N(GTXD8_N), 
    .RXP(RXP), 
    .RXN(RXN), 
    .TXP(TXP), 
    .TXN(TXN),
    .s_axi_aclk(s_axi_aclk),
    .s_axi_aresetn(s_axi_aresetn),
    .s_axi_arvalid(s_axi_arvalid),
    .s_axi_arready(s_axi_arready),
    .s_axi_araddr(s_axi_araddr),
    .s_axi_arprot(s_axi_arprot),
    .s_axi_rvalid(s_axi_rvalid),
    .s_axi_rready(s_axi_rready),
    .s_axi_rdata(s_axi_rdata),
    .s_axi_rresp(s_axi_rresp)
    );
  
endmodule


module nf10_upb_interconnect #( 
    parameter C_PORT_NUMBER = 0,
    parameter C_INPORT_WIDTH = 3,
    parameter C_OUTPORT_WIDTH = 8,
    parameter C_PACKET_LENGTH_WIDTH = 14,
    parameter C_MAX_PACKET_LENGTH = 10000, // in byte
    parameter C_AXIS_DATA_WIDTH = 256,
    parameter SIM_GTXRESET_SPEEDUP = 1,
    parameter [31:0] C_AXI_BASE_ADDR = 32'h00000000,
    parameter [31:0] C_AXI_HIGH_ADDR = 32'hFFFFFFFF
)
(
    // input wire arbiter side //
    input wire axi_aclk,
    input wire axi_resetn,    
    
    // Master Stream Ports
    output wire [C_AXIS_DATA_WIDTH-1:0] arbiter_m_axis_tdata,
    output wire [(C_AXIS_DATA_WIDTH/8)-1:0] arbiter_m_axis_tkeep, 
    output wire [C_PACKET_LENGTH_WIDTH-1:0] arbiter_m_axis_tuser_packet_length,
    output reg [C_INPORT_WIDTH-1:0] arbiter_m_axis_tuser_in_port = C_PORT_NUMBER,
    output reg [C_OUTPORT_WIDTH-1:0] arbiter_m_axis_tuser_out_port = 0,
    output reg [C_INPORT_WIDTH-1:0] arbiter_m_axis_tuser_in_vport = 0,
    output reg [C_OUTPORT_WIDTH-1:0] arbiter_m_axis_tuser_out_vport = 0,
    output wire arbiter_m_axis_tvalid,
    input  wire arbiter_m_axis_tready,
    output wire arbiter_m_axis_tlast,
	 
    // output queue side //
    output wire output_queue_clk,
    
    // Slave Stream Ports
    input wire [C_AXIS_DATA_WIDTH-1:0] output_queue_s_axis_tdata,
    input wire [(C_AXIS_DATA_WIDTH/8)-1:0] output_queue_s_axis_tkeep,
    input wire [C_PACKET_LENGTH_WIDTH-1:0] output_queue_s_axis_tuser_packet_length,
    input wire [C_INPORT_WIDTH-1:0] output_queue_s_axis_tuser_in_port,
    input wire [C_OUTPORT_WIDTH-1:0] output_queue_s_axis_tuser_out_port,
    input wire [C_INPORT_WIDTH-1:0] output_queue_s_axis_tuser_in_vport,
    input wire [C_OUTPORT_WIDTH-1:0] output_queue_s_axis_tuser_out_vport,
    input wire output_queue_s_axis_tvalid,
    output wire output_queue_s_axis_tready,
    input wire output_queue_s_axis_tlast,

    input wire RESET, 
    input wire INIT_CLK,
    input wire GT_RESET_IN,
    input wire GTXD8_P,
    input wire GTXD8_N,
    input wire [0:9] RXP,
    input wire [0:9] RXN,
    output wire [0:9] TXP,
    output wire [0:9] TXN,

   // AXI4Lite for reading statistics
    input wire s_axi_aclk,
    input wire s_axi_aresetn,
    input wire s_axi_arvalid,
    output reg s_axi_arready,
    input wire [31:0] s_axi_araddr,
    input wire [2:0] s_axi_arprot,
    output reg s_axi_rvalid,
    input wire s_axi_rready,
    output reg [31:0] s_axi_rdata,
    output reg [1:0] s_axi_rresp
);
endmodule
