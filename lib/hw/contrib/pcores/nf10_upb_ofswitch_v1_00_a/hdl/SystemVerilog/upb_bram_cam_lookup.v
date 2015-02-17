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
`default_nettype none
`include "parameters.v"
`include "tuple_t.v"
/*
 * The UPB BRAM CAM lookup module
 * 
 * To fill the CAM via AXI4 proceed as follows:
 * 
 * Let t be the tuple (format exactly as in tuple_t but not including valid bit).
 * Calculate x = crc(t)[10:0]
 * {C_AXI_ADDR,0..0,x<<5} is the base address for the CAM entry
 * 1. Write the action as a 18bit tuple ({2bit type, 8bit vport, 8bit port}) to address {C_AXI_ADDR,0..0,x<<5}
 * 2. Write t[31:0] to address {C_AXI_ADDR,0..0,x<<5}
 * 3. Write t[63:32] to address {C_AXI_ADDR,0..0,x<<5,1<<2}
 * ...
 * 9. Write t[243:224] to address {C_AXI_ADDR,0..0,x,7<<2}
 *
 * We use the first address twice as we can use the parity bits of the 
 * bram primitives to store 36 bit words. As AXI only allows us to write
 * 32 bit words we cache the 32 bits of the first write and write them 
 * 4 bit at a time to the bram in the following write operations.
 *
 * IMPORTANT: The CAM is write only!
 * Reading addresses will NOT give the content of the CAM.
 * Read address map:
 * {C_AXI_ADDR,0..0,0} = "CAM"
 * {C_AXI_ADDR,0..0,4} = Version number
 * {C_AXI_ADDR,0..0,8} = Number of CAM entries
 * {C_AXI_ADDR,0..0,c} = Answer to The Ultimate Question of Life, the Universe, and Everything.
 */
module upb_bram_cam_lookup #(
    parameter CAM_DEPTH = 2048,   // number of entries
    parameter CAM_VERSION = 1,
    parameter [31:0] C_AXI_BASE_ADDR = 32'h00000000,
    parameter [31:0] C_AXI_HIGH_ADDR = 32'hFFFFFFFF
)(
    input wire CLK,
    input wire RST,
    
    input tuple_t tuple,
    output reg action_match, 
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
    tuple_t tuple_delayed;
    
    logic[$clog2(CAM_DEPTH)-1:0] hash;
    logic[$clog2(CAM_DEPTH)-1:0] hash_delayed;
    
    logic [$clog2(CAM_DEPTH)+2:0] waddr;
    logic [$clog2(CAM_DEPTH)+2:0] waddr_next;
    
    logic [31:0] wdata;
    logic [35:0] wdata_r; // 'real' wdata signal that is written into bram
    logic [31:0] wdata_next;
    
    logic [C_OUT_PORT_WIDTH*2-1+4:0] action; 
    logic [C_OUT_PORT_WIDTH*2-1+4:0] action_next;
    
    wire [31:0] s_axi_araddr_masked_32b_aligned = (s_axi_araddr & (C_AXI_BASE_ADDR ^ C_AXI_HIGH_ADDR)) >> 2; // reduce address lines from byte adressing to (32 bit) word adressing
    wire [31:0] s_axi_awaddr_masked_32b_aligned = (s_axi_awaddr & (C_AXI_BASE_ADDR ^ C_AXI_HIGH_ADDR)) >> 2; // reduce address lines from byte adressing to (32 bit) word adressing
    
    logic [27:0] trash; // we don't really need this (but it made a stupid decision easier, see cam_ram instantiation)
    
    crc crc_inst (
        .data_in(tuple[242:0]),
        .crc_out(hash)
    );
    
    tuple_t tuple_bram_out;
    logic [19:0] action_bram_out;
    logic wen, wen_r;
    
    cam_ram #(.CAM_DEPTH(CAM_DEPTH)) bram_inst (
        .clka(s_axi_aclk),
        .wea(wen_r),
        .addra(waddr),
        .reset(RST),
        .dina(wdata_r),
        .clkb(CLK),
        .addrb(hash),
        // the following line could have been prevented if we decided to just shift the data in the write implementation of the cam_ram instead of inserting chunks of data. feel free to fix it.
        .doutb({trash[27:16],tuple_bram_out[243 : 224],trash[15-:4],tuple_bram_out[223 -: 32],trash[11-:4],tuple_bram_out[191 -: 32],trash[7-:4],tuple_bram_out[159 -: 32],trash[3-:2],action_bram_out[17-:2],tuple_bram_out[127 -: 32],action_bram_out[15-:4],tuple_bram_out[95 -: 32],action_bram_out[11-:4],tuple_bram_out[63 -: 32],action_bram_out[7-:4],tuple_bram_out[31 -: 32],action_bram_out[3-:4]})
    ); 
   
   
    assign action_valid = tuple_delayed.valid;
    assign action_match = tuple_bram_out == tuple_delayed && tuple_delayed.valid;
    assign action_type = action_bram_out[17-:2];
    assign action_vport = action_bram_out[C_OUT_PORT_WIDTH*2-1:C_OUT_PORT_WIDTH];
    assign action_port = action_bram_out[C_OUT_PORT_WIDTH-1:0];
    assign action_match_addr = hash_delayed;
    
    always @(posedge CLK) begin
        hash_delayed <= hash;
        tuple_delayed <= tuple;
    end
    
    // AXI4Lite: write
    typedef enum {WR_STATE_WAITING, WR_STATE_WAITING_FOR_DATA, WR_STATE_WAITING_FOR_ADDR, WR_STATE_PROCESSING, WR_STATE_ANSWERING} wr_states;
    typedef enum {WR_ISTATE_WAITING_FOR_ACTION, WR_ISTATE_PROCESSING} wr_int_states;
    wr_states wr_state = WR_STATE_WAITING, wr_state_next;
    wr_int_states wr_int_state = WR_ISTATE_WAITING_FOR_ACTION, wr_int_state_next;
    
    
    
    wire s_axi_awaddr_in_range = (s_axi_awaddr & (C_AXI_BASE_ADDR ~^ C_AXI_HIGH_ADDR)) == C_AXI_BASE_ADDR;
    
    
    
    
    always_comb begin
        wr_int_state_next = wr_int_state;
        action_next = action;
        wen_r = 0;
        wdata_r = 0;
        case(wr_int_state)
            WR_ISTATE_WAITING_FOR_ACTION: begin //cache first write operation
                action_next = wdata;
                if(wen) begin
                    wr_int_state_next = WR_ISTATE_PROCESSING;
                end
            end
            WR_ISTATE_PROCESSING: begin // write data + 4 bits of first write to bram
                wen_r = wen;
                wdata_r = {wdata, {action[(waddr[2:0]+1)*4-1-:4]}};
                if(wen && (waddr[2:0] == 3'b111)) begin
                    wr_int_state_next = WR_ISTATE_WAITING_FOR_ACTION;
                end
            end
        endcase
    end
    
    always_ff @(posedge s_axi_aclk) begin
        wr_int_state <= wr_int_state_next;
        action <= action_next;
        
        wr_state <= wr_state_next;
        waddr <= waddr_next;
        wdata <= wdata_next;
    end
    
    
    always_comb begin
        s_axi_bresp = 0; // OK
        s_axi_bvalid = 0;
        s_axi_awready = 0;
        s_axi_wready = 0;
        
        wen = 0;
                
        waddr_next = waddr;
        wdata_next = wdata;
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
                wen = 1;                
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
    
    
    // AXI4Lite: read
    typedef enum {RD_STATE_WAITING, RD_STATE_PROCESSING, RD_STATE_ANSWERING} rd_states;
    rd_states rd_state = RD_STATE_WAITING, rd_state_next;
    logic [27:0] raddr_next;
    logic [27:0] raddr;
    wire s_axi_araddr_in_range = (s_axi_araddr & (C_AXI_BASE_ADDR ~^ C_AXI_HIGH_ADDR)) == C_AXI_BASE_ADDR;
    
    always_ff @(posedge s_axi_aclk) begin
        rd_state <= rd_state_next;
        raddr <= raddr_next;
    end
    
    always_comb begin
        s_axi_arready = 0;
        s_axi_rvalid = 0;
        s_axi_rresp = 0; // OK
        s_axi_rdata = 0;
        
        raddr_next = raddr;
        rd_state_next = rd_state;
        
        case (rd_state)
            RD_STATE_WAITING: begin
                s_axi_arready = 1;
                if (s_axi_arvalid && s_axi_araddr_in_range) begin
                    rd_state_next = RD_STATE_ANSWERING;
                    raddr_next = s_axi_araddr_masked_32b_aligned;
                end
            end
            RD_STATE_ANSWERING: begin
                s_axi_rvalid = 1;
                case(raddr)
                    0: s_axi_rdata = "CAM";
                    1: s_axi_rdata = CAM_VERSION;
                    2: s_axi_rdata = CAM_DEPTH;
                    3: s_axi_rdata = 42;
                endcase
                if (s_axi_rready)
                    rd_state_next = RD_STATE_WAITING;
            end
        endcase
        
        if (~s_axi_aresetn) begin
            rd_state_next = RD_STATE_WAITING;
        end
    end
    
endmodule
