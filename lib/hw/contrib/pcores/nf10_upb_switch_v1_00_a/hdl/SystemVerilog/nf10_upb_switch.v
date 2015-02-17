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
 
module nf10_upb_switch #(

    parameter dma_port_id   = 999 // this value has to be overwritten by the project file !!!

)(
    input clk,
    input clk2x,
    input clk2x90,
    input reset,
    
    
    
    
    
    
    output logic            qdr_c_k,                // K
    output logic            qdr_c_k_n,          // K
    output logic            qdr_c_c,                // C
    output logic            qdr_c_c_n,          // Cn
    input wire              qdr_c_cq,           // CQ
    input wire              qdr_c_cq_n,         // CQn
    output logic [18:0] qdr_c_sa,           // A[18:0]
    output logic            qdr_c_r_n,          // RPS
    input wire [35:0]       qdr_c_q,                // Q[35:0]
    output logic            qdr_c_w_n,          // WPS
    output logic [3:0]  qdr_c_bw_n,         // BWS[3:0]
    output logic [35:0] qdr_c_d,                // D[35:0]
    output logic            qdr_c_dll_off_n,    // DOFFn
    
    
    
    
    
    
    
    input [255:0] s_axis_tdata,
    input [31:0] s_axis_tkeep,
    input s_axis_tvalid,
    input s_axis_tlast,
    input [2:0] s_axis_tuser_in_port,
    input [2:0] s_axis_tuser_in_vport,
    input [7:0] s_axis_tuser_out_port,
    input [7:0] s_axis_tuser_out_vport,
    input [13:0] s_axis_tuser_packet_length,
    output s_axis_tready,
    
    
    output reg [255:0] m_axis_tdata,
    output reg [31:0] m_axis_tkeep,
    output reg m_axis_tvalid = 0,
    output reg m_axis_tlast,
    output reg [2:0] m_axis_tuser_in_port,
    output reg [2:0] m_axis_tuser_in_vport,
    output reg [7:0] m_axis_tuser_out_port,
    output reg [7:0] m_axis_tuser_out_vport,
    output reg [13:0] m_axis_tuser_packet_length,
    input m_axis_tready
    
);
    assign s_axis_tready = ((m_axis_tready && m_axis_tvalid) || ~m_axis_tvalid);
    
    reg [19:0] minute_counter = 0; // lifetime in minutes (max. 1 year).
    reg [33:0] counter = 0;
    
    
    /*
     * CRC
     */
    
    reg [47:0] mac_src=0; //crc input
    reg [47:0] mac_dst=0;
    reg wr_en = 0;
    
    reg [18:0] src_crc; //crc output
    reg [18:0] dst_crc;
    
    crc crc_src (
        .data_in(mac_src),
       .crc_out(src_crc)
    );
    
    crc crc_dst (
        .data_in(mac_dst),
        .crc_out(dst_crc)
    );
    
    
    /*
     * SRAM Hashtable
     */
    qdr2_sram_if qdr();
    assign qdr_c_k = qdr.k;
    assign qdr_c_k_n = qdr.k_n;
    assign qdr_c_c = qdr.c;
    assign qdr_c_c_n = qdr.c_n;
    assign qdr.cq = qdr_c_cq;
    assign qdr.cq_n = qdr_c_cq_n;
    assign qdr_c_sa = qdr.sa;
    assign qdr_c_r_n = qdr.r_n;
    assign qdr.q = qdr_c_q;
    assign qdr_c_w_n = qdr.w_n;
    assign qdr_c_bw_n = qdr.bw_n;
    assign qdr_c_d = qdr.d;
    assign qdr_c_dll_off_n = qdr.dll_off_n;
    
    wire [127:0] sram_write_data; // write input
    assign sram_write_data = {mac_src,s_axis_tuser_in_port,s_axis_tuser_in_vport,{54{1'b0}},minute_counter};
    
    
    
    reg [127:0] sram_read_data; // sram output
    wire [47:0] sram_read_data_mac;
    wire [2:0] sram_read_data_port;
    wire [2:0] sram_read_data_vport;
    wire [19:0] sram_read_data_timestamp;
    assign sram_read_data_mac = sram_read_data[127:80];
    assign sram_read_data_port = sram_read_data[79:77];
    assign sram_read_data_vport = sram_read_data[76:74];
    assign sram_read_data_timestamp = sram_read_data[19:0];
    wire sram_ready;

    qdr2_sram sram (
        .clk(clk),
        .clk2x(clk2x),
        .clk2x90(clk2x90),
        .reset(reset),
        .ready(sram_ready),
        .rd(1),
        .r_addr(dst_crc),
        .r_data(sram_read_data),
        .wr(wr_en),
        .w_addr(src_crc),
        .w_data(sram_write_data),
        
        .qdr(qdr)
    );
    
    /*
     * Delay Module to store AXI Stream while we wait for SRAM
     */
    
    wire delay_en; 
    assign delay_en = ((m_axis_tready && m_axis_tvalid) || ~m_axis_tvalid);
    
    wire [309:0] delay_data_out; //delay output
    wire delay_tvalid;
    wire [31:0] delay_tkeep;
    wire delay_tlast;
    wire [2:0] delay_tuser_in_port;
    wire [2:0] delay_tuser_in_vport;
    wire [13:0] delay_tuser_packet_length;
    wire [255:0] delay_tdata;
    wire [47:0] delay_tdata_mac_dst;
    assign delay_tvalid = delay_data_out[309];
    assign delay_tkeep = delay_data_out[308:277];
    assign delay_tlast = delay_data_out[276];
    assign delay_tuser_in_port = delay_data_out[275:273];
    assign delay_tuser_in_vport = delay_data_out[272:270];
    assign delay_tuser_packet_length = delay_data_out[269:256];
    assign delay_tdata = delay_data_out[255:0];
    assign delay_tdata_mac_dst = delay_tdata[47:0]; // only if first data chunk!
    
    
    delay delay_data (
        .clk(clk),
        .reset(reset),
        .en(delay_en),
        .data_in({s_axis_tvalid,s_axis_tkeep,s_axis_tlast,s_axis_tuser_in_port,s_axis_tuser_in_vport,s_axis_tuser_packet_length,s_axis_tdata}),
        .data_out(delay_data_out)
    );
    
    
    
    /*
     *  Statemachine
     */
    localparam WAIT_HEAD = 0;
    localparam HEAD = 1;
    reg [1:0] state = HEAD;
    reg [1:0] outstate = HEAD;
    
    
    
        
    reg [7:0] outport; // temp. storage for output bitmask
    reg [7:0] outvport;
    
    always @(posedge clk) begin
        wr_en <= 0;
        counter <= counter + 1;
        if(counter>=34'd8999600000) begin // ~ 1 minute has passed
            counter <= 0;
            minute_counter <= minute_counter + 1;
        end
     
        case (state)
            WAIT_HEAD: begin
                if(s_axis_tvalid && s_axis_tready && s_axis_tlast) begin
                    state <= HEAD;
                end
            end
            HEAD: begin
                if(s_axis_tvalid && s_axis_tready) begin
                    state <= WAIT_HEAD;
                    mac_src <= s_axis_tdata[96:48];
                    wr_en <= 1;
                    mac_dst <= s_axis_tdata[47:0];
                end
            end
        endcase
        
        
        case (outstate)
            WAIT_HEAD: begin
                if(delay_tvalid && delay_tlast) begin
                    outstate <= HEAD;
                end
            end
            HEAD: begin
                if(((m_axis_tvalid && m_axis_tready) || ~m_axis_tvalid) && delay_tvalid) begin // if last packet was sent AND value in delay fifo is valid
                
                    // |check for hash collision and correct entr|     | entry is not outdated                                               |    | check for system time overflow           |    | do not send out data to incoming port, except for dma port                         |
                    if((delay_tdata_mac_dst == sram_read_data_mac) && ((sram_read_data_timestamp >= minute_counter - 5) || minute_counter < 5) && (sram_read_data_timestamp <= minute_counter) && ((delay_tuser_in_port != sram_read_data_port) || (delay_tuser_in_port == dma_port_id))) begin
                        outport = 8'b00000001 << sram_read_data_port;
                        outvport = 8'b00000001 << sram_read_data_vport;
                    end
                    else begin
                        //broadcast
                        if (delay_tuser_in_port == dma_port_id) begin // if the packet comes from the DMA (the ONLY vport capable IO module!)...
                            outport = 8'b11111111; // broadcast to ALL ports (also the DMA port)...
                            outvport = ~(8'b00000001 << delay_tuser_in_vport); // but NOT to the incoming vport from the DMA

                        end else begin // packet does not come from the DMA, means: packet does not come from a vport capable IO module...
                            outport = ~(8'b00000001 << delay_tuser_in_port); // ...broadcast to all ports except the incoming one...
                            outvport = 8'b11111111; // ...and also to the all DMA vports...
                        end
                    end
                    outstate <= WAIT_HEAD;
                end
                
            end
        endcase
        if(((m_axis_tvalid && m_axis_tready) || ~m_axis_tvalid)) begin // condition == delay_en
             m_axis_tvalid <= delay_tvalid;
             m_axis_tdata <= delay_tdata;
             m_axis_tkeep <= delay_tkeep;
             m_axis_tlast <= delay_tlast;
             m_axis_tuser_in_port <= delay_tuser_in_port;
             m_axis_tuser_in_vport <= delay_tuser_in_vport;
             m_axis_tuser_packet_length <= delay_tuser_packet_length;
             m_axis_tuser_out_port <= outport;
             m_axis_tuser_out_vport <= outvport;
        end
        if(reset) begin
            state <= HEAD;
            outstate <= HEAD;
            counter <= 0;
            minute_counter <= 0;
        end
    end

endmodule
