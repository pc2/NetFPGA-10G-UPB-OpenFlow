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
`timescale 1ns / 1ps
`include "parameters.v"
`include "tuple_t.v"

/*
 * Testbench writes tuple + action into cam and then inputs tuple.
 * Check whether match signal and valid signal are set.
 */
module upb_bram_cam_lookup_tb;

    // Inputs
    reg CLK;
    reg RST=0;
    
    logic s_axi_awvalid=0;
    logic s_axi_awready;
    logic [31:0] s_axi_awaddr;
    logic [2:0] s_axi_awprot;
    logic s_axi_wvalid=0;
    logic s_axi_wready;
    logic [31:0] s_axi_wdata;
    logic [3:0] s_axi_wstrb;
    logic s_axi_bvalid;
    logic s_axi_bready = 1;
    logic [1:0] s_axi_bresp;
    logic s_axi_arvalid=0;
    logic s_axi_arready;
    logic s_axi_araddr;
    logic [2:0] s_axi_arprot;
    logic s_axi_rvalid;
    logic s_axi_rready=0;
    logic [31:0] s_axi_rdata;
    logic [1:0] s_axi_rresp;
    logic [31:0] wr_data [8:0];
    logic [31:0] wr_addr [8:0];
    logic [$bits(tuple_t)-1:0] tuple_in=0;
    
    tuple_t tuple;
    assign tuple.valid = 1;
    assign tuple.port = 2;
    assign tuple.vport = 0;
    assign tuple.dmac = 48'h002590d1849c;
    assign tuple.smac = 48'h002590d1849d;
    assign tuple.typ = 8;
    assign tuple.vid = 0;
    assign tuple.pcp = 0;
    assign tuple.sip = 3232287235;
    assign tuple.dip = 3232287237;
    assign tuple.prot = 0;
    assign tuple.tos = 0;
    assign tuple.tsp = 2134;
    assign tuple.tdp = 80;
    logic [15:0] action;
    logic [10:0] hash = 12'h4b1; // hash of the tuple. depends on tuple and CAM_DEPTH!
    
    // Instantiate the Unit Under Test (UUT)
    upb_bram_cam_lookup #(.CAM_DEPTH(2048)) uut (
        .CLK(CLK), 
        .RST(RST), 
        .tuple(tuple_in),
        .s_axi_aclk(CLK),
        .s_axi_aresetn(~RST),
        .s_axi_awvalid(s_axi_awvalid),
        .s_axi_awready(s_axi_awready),
        .s_axi_awaddr(s_axi_awaddr),
        .s_axi_awprot(s_axi_awprot),
        .s_axi_wvalid(s_axi_wvalid),
        .s_axi_wready(s_axi_wready),
        .s_axi_wdata(s_axi_wdata),
        .s_axi_wstrb(s_axi_wstrb),
        .s_axi_bvalid(s_axi_bvalid),
        .s_axi_bready(s_axi_bready),
        .s_axi_bresp(s_axi_bresp),
        .s_axi_arvalid(s_axi_arvalid),
        .s_axi_arready(s_axi_arready),
        .s_axi_araddr(s_axi_araddr),
        .s_axi_arprot(s_axi_arprot),
        .s_axi_rvalid(s_axi_rvalid),
        .s_axi_rready(s_axi_rready),
        .s_axi_rdata(s_axi_rdata),
        .s_axi_rresp(s_axi_rresp)
    );
    reg s = 0;
    initial begin
        // Initialize Inputs
        CLK = 0;
        RST = 0;
        wr_addr[0] = {hash,5'b00000};
        wr_addr[1] = {hash,5'b00000};
        wr_addr[2] = {hash,5'b00100};
        wr_addr[3] = {hash,5'b01000};
        wr_addr[4] = {hash,5'b01100};
        wr_addr[5] = {hash,5'b10000};
        wr_addr[6] = {hash,5'b10100};
        wr_addr[7] = {hash,5'b11000};
        wr_addr[8] = {hash,5'b11100};
        // Wait 100 ns for global reset to finish
        #100;
        s=1;
        
    end
    typedef enum {SEND,SENT} axi_state;
    axi_state addr_state = SEND, addr_state_next;
    axi_state data_state = SEND, data_state_next;
    integer counter = 0, counter_next;
    
    always_comb begin
        if(counter == 10) begin
            tuple_in = tuple;
        end
        else begin
            tuple_in = 0;
        end
    end
    
    always_comb begin
        if(s==1) begin
            addr_state_next = addr_state;
            data_state_next = data_state;
            counter_next = counter;
            if(counter <= 8) begin
                s_axi_awaddr = wr_addr[counter];
                if(counter == 0) begin
                    s_axi_wdata = 32'hffffabcd;
                end
                else if(counter<8) begin
                    s_axi_wdata = tuple[(counter)*32-1-:32];
                end
                else begin
                    s_axi_wdata = {13'b1111111111111,tuple[243-:20]};
                end
            end
            case(addr_state)
                SEND: begin
                    s_axi_awvalid = 1;
                    if(s_axi_awvalid && s_axi_awready) begin
                        addr_state_next = SENT;
                    end
                end
                SENT: begin
                    s_axi_awvalid = 0;
                end
            endcase
            case(data_state)
                SEND: begin
                    s_axi_wvalid = 1;
                    if(s_axi_wvalid && s_axi_wready) begin
                        data_state_next = SENT;
                    end
                end
                SENT: begin
                    s_axi_awvalid = 0;
                end
            endcase
            if((data_state == SENT) && (addr_state == SENT)) begin
                counter_next = counter + 1;
                data_state_next = SEND;
                addr_state_next = SEND;
            end
            if(counter>8) begin
                counter_next = counter + 1;
                s_axi_awvalid = 0;
                s_axi_wvalid = 0;
            end
        end
    end
    always @(posedge CLK) begin
        if(s==1) begin
            counter <= counter_next;
            addr_state <= addr_state_next;
            data_state <= data_state_next;
        end
    end
    
    always begin
	  #5 CLK = ~CLK;
	end
endmodule
