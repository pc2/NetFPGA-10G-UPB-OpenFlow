/*
 * Copyright (c) 2014, 2015 Michael Lass
 * bevan@bi-co.net
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
`include "parameters.v"
`include "tuple_t.v"

module upb_tcam_lookup #(
	parameter DELAY_MATCH = 3, // minimum: 1
	parameter DELAY_LOOKUP = 3,  // minimum: 1
	parameter DELAY_WRITE = 3, // minimum: 1
	parameter FORCE_MUXCY = 0,  // force usage of carry chain on current Xilinx devices
	parameter TCAM_DEPTH = 32,   // number of entries (max. 1024)
	parameter [31:0] C_AXI_BASE_ADDR = 32'h00000000,
	parameter [31:0] C_AXI_HIGH_ADDR = 32'hFFFFFFFF
)(
	input wire CLK,
	input wire RST,
	
	input tuple_t tuple,
	output reg action_match, // no way to define addr_width :(
    output reg action_valid,
    output reg [1:0] action_type,
    output reg [C_OUT_PORT_WIDTH-1:0] action_port,
    output reg [C_OUT_PORT_WIDTH-1:0] action_vport,
    output reg [C_MATCH_ADDR_WIDTH-1:0] action_match_addr,
	
    
    input wire s_axi_aclk,
    input wire s_axi_aresetn,
    input wire s_axi_awvalid,
    output reg s_axi_awready,
    input wire [31:0] s_axi_awaddr,
    input wire [2:0] s_axi_awprot,
    input wire s_axi_wvalid,
    output reg s_axi_wready,
    input wire [31:0] s_axi_wdata,
    input wire [3:0] s_axi_wstrb,
    output reg s_axi_bvalid,
    input wire s_axi_bready,
    output reg [1:0] s_axi_bresp,
    input wire s_axi_arvalid,
    output reg s_axi_arready,
    input wire [31:0] s_axi_araddr,
    input wire [2:0] s_axi_arprot,
    output reg s_axi_rvalid,
    input wire s_axi_rready,
    output reg [31:0] s_axi_rdata,
    output reg [1:0] s_axi_rresp

);
	
	// instanciate TCAM
	
	logic [31:0] tcam_wdata;
	logic [31:0] tcam_waddr;
	logic tcam_wen;
	logic [31:0] tcam_raddr;
	logic tcam_rden;
	logic [31:0] tcam_rdata;
	logic [244:0] tcam_content;
	logic [17:0] tcam_data_out;
	logic tcam_match_out;
	logic [$clog2(TCAM_DEPTH)-1:0] tcam_match_addr;

	upb_tcam #(
		.SRL_SIZE(32),
		.TCAM_WIDTH(49),
		.DELAY_LOOKUP(DELAY_LOOKUP),
		.DELAY_MATCH(DELAY_MATCH),
		.DELAY_WRITE(DELAY_WRITE),
		.FORCE_MUXCY(FORCE_MUXCY),
		.TCAM_DEPTH(TCAM_DEPTH),
		.DATA_WIDTH(18)
	) tcam (
		.CLK(CLK),
		.RST(RST),
		.wdata(tcam_wdata),
		.waddr(tcam_waddr),
		.wen(tcam_wen),
		.raddr(tcam_raddr),
		.rden(tcam_rden),
		.rdata(tcam_rdata),
		.content(tcam_content),
		.data_out(tcam_data_out),
		.match_out(tcam_match_out),
		.match_addr(tcam_match_addr)
	);
	
	
	
	// map tuple interface
	assign tcam_content = {tuple.port, tuple.vport, tuple.dmac, tuple.smac, tuple.typ, tuple.vid, tuple.pcp, tuple.sip, tuple.dip, tuple.prot, tuple.tos, tuple.tsp, tuple.tdp};
	
	
	
	// map action interface, delay valid signal by DELAY_LOOKUP+DELAY_MATCH+1 clk cycles (one extra for BRAM lookup)
	logic [DELAY_LOOKUP+DELAY_MATCH:0] valid_reg = 0; // might be implemented as shift register - should not harm timing
	
	always_ff @(posedge CLK) begin
		valid_reg <= {valid_reg[DELAY_LOOKUP+DELAY_MATCH-1:0], tuple.valid};
		
		if (RST)
			valid_reg <= {(DELAY_LOOKUP+DELAY_MATCH+1){1'b0}};
	end
	
	assign action_valid = valid_reg[DELAY_LOOKUP+DELAY_MATCH];
	assign action_match = tcam_match_out;
	assign action_match_addr = tcam_match_addr;
	assign action_port = tcam_data_out[7:0];
	assign action_vport = tcam_data_out[15:8];
    assign action_type = tcam_data_out[17:16];
	
	
	
	
	// AXI4Lite: read
	typedef enum {RD_STATE_WAITING, RD_STATE_PROCESSING, RD_STATE_ANSWERING} rd_states;
	rd_states rd_state = RD_STATE_WAITING, rd_state_next;
	logic [31:0] raddr_next;
	wire s_axi_araddr_in_range = (s_axi_araddr & (C_AXI_BASE_ADDR ~^ C_AXI_HIGH_ADDR)) == C_AXI_BASE_ADDR;
	wire [31:0] s_axi_araddr_masked_32b_aligned = (s_axi_araddr & (C_AXI_BASE_ADDR ^ C_AXI_HIGH_ADDR)) >> 2; // reduce address lines from byte adressing to (32 bit) word adressing
	
	always_ff @(posedge s_axi_aclk) begin
		rd_state <= rd_state_next;
		tcam_raddr <= raddr_next;
	end
	
	always_comb begin
		s_axi_arready = 0;
		s_axi_rvalid = 0;
		s_axi_rresp = 0; // OK
		s_axi_rdata = tcam_rdata;
		
		tcam_rden = 0;
		
		raddr_next = tcam_raddr;
		rd_state_next = rd_state;
		
		case (rd_state)
			RD_STATE_WAITING: begin
				s_axi_arready = 1;
				if (s_axi_arvalid && s_axi_araddr_in_range) begin
					rd_state_next = RD_STATE_PROCESSING;
					raddr_next = s_axi_araddr_masked_32b_aligned;
            end
			end
			
			RD_STATE_PROCESSING: begin
				rd_state_next = RD_STATE_ANSWERING;
				tcam_rden = 1;
			end
			
			RD_STATE_ANSWERING: begin
				s_axi_rvalid = 1;
				if (s_axi_rready)
					rd_state_next = RD_STATE_WAITING;
			end
		endcase
		
		if (~s_axi_aresetn) begin
			rd_state_next = RD_STATE_WAITING;
		end
	end
	
	
	// AXI4Lite: write
	typedef enum {WR_STATE_WAITING, WR_STATE_WAITING_FOR_DATA, WR_STATE_WAITING_FOR_ADDR, WR_STATE_PROCESSING, WR_STATE_ANSWERING} wr_states;
	wr_states wr_state = WR_STATE_WAITING, wr_state_next;
	logic [31:0] waddr_next;
	logic [31:0] wdata_next;
	wire s_axi_awaddr_in_range = (s_axi_awaddr & (C_AXI_BASE_ADDR ~^ C_AXI_HIGH_ADDR)) == C_AXI_BASE_ADDR;
	wire [31:0] s_axi_awaddr_masked_32b_aligned = (s_axi_awaddr & (C_AXI_BASE_ADDR ^ C_AXI_HIGH_ADDR)) >> 2; // reduce address lines from byte adressing to (32 bit) word adressing
	
	always_ff @(posedge s_axi_aclk) begin
		wr_state <= wr_state_next;
		tcam_waddr <= waddr_next;
		tcam_wdata <= wdata_next;
	end
	
	always_comb begin
		s_axi_bresp = 0; // OK
		s_axi_bvalid = 0;
		s_axi_awready = 0;
		s_axi_wready = 0;
		
		tcam_wen = 0;
				
		waddr_next = tcam_waddr;
		wdata_next = tcam_wdata;
		wr_state_next = wr_state;
		
		case (wr_state)
			WR_STATE_WAITING: begin
				s_axi_awready = 1;
				s_axi_wready = 1;
				if (s_axi_awvalid && s_axi_awaddr_in_range) begin
					wr_state_next = WR_STATE_WAITING_FOR_DATA;
					waddr_next = s_axi_awaddr_masked_32b_aligned;
				end
				if (s_axi_wvalid) begin
					wr_state_next = WR_STATE_WAITING_FOR_ADDR;
					wdata_next = s_axi_wdata;
				end
				if (s_axi_awvalid && s_axi_wvalid && s_axi_awaddr_in_range)
					wr_state_next = WR_STATE_PROCESSING;
			end
			
			WR_STATE_WAITING_FOR_DATA: begin
				s_axi_wready = 1;
				if (s_axi_wvalid) begin
					wr_state_next = WR_STATE_PROCESSING;
					wdata_next = s_axi_wdata;
				end
			end
			
			WR_STATE_WAITING_FOR_ADDR: begin
				s_axi_awready = 1;
				if (s_axi_awvalid) begin
					if (s_axi_awaddr_in_range) begin
						wr_state_next = WR_STATE_PROCESSING;
						waddr_next = s_axi_awaddr_masked_32b_aligned;
					end else
						wr_state_next = WR_STATE_WAITING;
				end
			end
			
			WR_STATE_PROCESSING: begin
				wr_state_next = WR_STATE_ANSWERING;
				tcam_wen = 1;				
			end
			
			WR_STATE_ANSWERING: begin
				s_axi_bvalid = 1;
				if (s_axi_bready)
					wr_state_next = WR_STATE_WAITING;
			end
		endcase
		
		if (~s_axi_aresetn) begin
			wr_state_next = WR_STATE_WAITING;
		end
	end

endmodule
