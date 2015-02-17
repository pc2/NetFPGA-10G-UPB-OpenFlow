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
 
/*
 * Look at pkt-/errorcounters of frame_gen and frame_check modules.
 */
 
`timescale 1ns / 1ps
module nf10_upb_ofswitch_tb;

    wire [255:0] s_axis_tdata;
    wire [31:0] s_axis_tkeep;
    wire s_axis_tvalid;
    wire s_axis_tlast;
    wire [2:0] s_axis_tuser_in_port;
    wire [2:0] s_axis_tuser_in_vport;
    wire [7:0] s_axis_tuser_out_port;
    wire [7:0] s_axis_tuser_out_vport;
    wire [13:0] s_axis_tuser_packet_length;
    wire s_axis_tready;
    
    
    wire [255:0] m_axis_tdata;
    wire [31:0] m_axis_tkeep;
    wire m_axis_tvalid;
    wire m_axis_tlast;
    wire [2:0] m_axis_tuser_in_port;
    wire [2:0] m_axis_tuser_in_vport;
    wire [7:0] m_axis_tuser_out_port;
    wire [7:0] m_axis_tuser_out_vport;
    wire [13:0] m_axis_tuser_packet_length;
    wire m_axis_tready;
    
    
    logic s_axi_stats_arvalid;
    logic s_axi_stats_arready;
    logic [31:0] s_axi_stats_araddr;
    logic [2:0] s_axi_stats_arprot;
    logic s_axi_stats_rvalid;
    logic s_axi_stats_rready=1;
    logic [31:0] s_axi_stats_rdata;
    logic [1:0] s_axi_stats_rresp;
    
    
    logic clk = 0;
    logic reset = 0;
    
    frame_gen #(.C_IN_PORT(2)) generator (
        .clk(clk),
        .axi_resetn(reset),
        .m_axis_tdata(s_axis_tdata),
        .m_axis_tvalid(s_axis_tvalid),
        .m_axis_tkeep(s_axis_tkeep),
        .m_axis_tuser_packet_length(s_axis_tuser_packet_length),
        .m_axis_tuser_in_port(s_axis_tuser_in_port),
        .m_axis_tuser_in_vport(s_axis_tuser_in_vport),
        .m_axis_tuser_out_port(s_axis_tuser_out_port),
        .m_axis_tuser_out_vport(s_axis_tuser_out_vport),
        .m_axis_tready(s_axis_tready),
        .m_axis_tlast(s_axis_tlast)
    );

    nf10_upb_ofswitch #(.C_AXI_BASE_ADDR_TCAM(32'hA0000000),
                        .C_AXI_HIGH_ADDR_TCAM(32'hAFFFFFFF),
                        .C_AXI_BASE_ADDR_CAM(32'hB0000000),
                        .C_AXI_HIGH_ADDR_CAM(32'hBFFFFFFF),
                        .C_AXI_BASE_ADDR_STATS(32'hC0000000),
                        .C_AXI_HIGH_ADDR_STATS(32'hCFFFFFFF),
                        .C_DMA_PORT(5),
                        .C_DMA_FIRST_EXTERNAL_PORT(0),
                        .C_DMA_LAST_EXTERNAL_PORT(4),
                        .C_BRIDGED_ETH_A_VPORT(5),
                        .C_BRIDGED_ETH_B_VPORT(6),
                        .TCAM_DEPTH(64),
                        .CAM_DEPTH(2048))
        uut(
        .clk(clk),
        .reset(reset),
        .m_axis_tdata(m_axis_tdata),
        .m_axis_tvalid(m_axis_tvalid),
        .m_axis_tkeep(m_axis_tkeep),
        .m_axis_tuser_packet_length(m_axis_tuser_packet_length),
        .m_axis_tuser_in_port(m_axis_tuser_in_port),
        .m_axis_tuser_in_vport(m_axis_tuser_in_vport),
        .m_axis_tuser_out_port(m_axis_tuser_out_port),
        .m_axis_tuser_out_vport(m_axis_tuser_out_vport),
        .m_axis_tready(m_axis_tready),
        .m_axis_tlast(m_axis_tlast),
        .s_axis_tdata(s_axis_tdata),
        .s_axis_tvalid(s_axis_tvalid),
        .s_axis_tkeep(s_axis_tkeep),
        .s_axis_tuser_packet_length(s_axis_tuser_packet_length),
        .s_axis_tuser_in_port(s_axis_tuser_in_port),
        .s_axis_tuser_in_vport(s_axis_tuser_in_vport),
        .s_axis_tuser_out_port(s_axis_tuser_out_port),
        .s_axis_tuser_out_vport(s_axis_tuser_out_vport),
        .s_axis_tready(s_axis_tready),
        .s_axis_tlast(s_axis_tlast),
        
        
        .s_axi_stats_aclk(clk),
        .s_axi_stats_aresetn(~reset),
        .s_axi_stats_arvalid(s_axi_stats_arvalid),
        .s_axi_stats_arready(s_axi_stats_arready),
        .s_axi_stats_araddr(s_axi_stats_araddr),
        .s_axi_stats_arprot(s_axi_stats_arprot),
        .s_axi_stats_rvalid(s_axi_stats_rvalid),
        .s_axi_stats_rready(s_axi_stats_rready),
        .s_axi_stats_rdata(s_axi_stats_rdata),
        .s_axi_stats_rresp(s_axi_stats_rresp)
    );

    frame_check checkr (
        .clk(clk),
        .axi_resetn(reset),
        .s_axis_tdata(m_axis_tdata),
        .s_axis_tvalid(m_axis_tvalid),
        .s_axis_tkeep(m_axis_tkeep),
        .s_axis_tuser_packet_length(m_axis_tuser_packet_length),
        .s_axis_tuser_in_port(m_axis_tuser_in_port),
        .s_axis_tuser_in_vport(m_axis_tuser_in_vport),
        .s_axis_tuser_out_port(m_axis_tuser_out_port),
        .s_axis_tuser_out_vport(m_axis_tuser_out_vport),
        .s_axis_tready(m_axis_tready),
        .s_axis_tlast(m_axis_tlast)
    );
    initial begin
    #100
    reset = 1;
    #100
    reset =0;
    end
    always
            #5 clk = ~clk;
    
    typedef enum {IDLE,REQ_PKT,REQ_BYTE,REQ_TIME,REQ_CTIME} t_axi_state;
    t_axi_state axi_state=IDLE, next_axi_state;
    
    logic [11:0] counter=1;
    
    always_ff @(posedge clk) begin
        counter <= counter + 1;
        axi_state <= next_axi_state;
    end
    
    
    always_comb begin
        s_axi_stats_arvalid = 0;
        next_axi_state = axi_state;
        case (axi_state)
            REQ_PKT: begin
                s_axi_stats_araddr = 32'hC0000020;
                s_axi_stats_arvalid = 1;
                if(s_axi_stats_arready)
                    next_axi_state = IDLE;
            end
            REQ_BYTE: begin
                s_axi_stats_araddr = 32'hC0000024;
                s_axi_stats_arvalid = 1;
                if(s_axi_stats_arready)
                    next_axi_state = IDLE;
            end
            REQ_TIME: begin
                s_axi_stats_araddr = 32'hC0000028;
                s_axi_stats_arvalid = 1;
                if(s_axi_stats_arready)
                    next_axi_state = IDLE;
            end
            REQ_CTIME: begin
                s_axi_stats_araddr = 32'hC0010010;
                s_axi_stats_arvalid = 1;
                if(s_axi_stats_arready)
                    next_axi_state = IDLE;
            end
            IDLE: begin
                case (counter)
                    1024: next_axi_state = REQ_PKT;
                    2048: next_axi_state = REQ_BYTE;
                    3082: next_axi_state = REQ_TIME;
                    4095: next_axi_state = REQ_CTIME;
                endcase
            end
        endcase
            
    end
endmodule
