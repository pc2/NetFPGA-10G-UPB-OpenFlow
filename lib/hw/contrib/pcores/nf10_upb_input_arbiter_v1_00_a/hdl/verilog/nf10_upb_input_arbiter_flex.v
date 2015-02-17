/*
 * UPB Input Arbiter
 *
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

module nf10_upb_input_arbiter_flex
#(
    parameter C_DATA_WIDTH=256,
    parameter C_PACKET_LENGTH_WIDTH=14,
    parameter C_TKEEP_WIDTH=32, //C_DATA_WIDTH/8
    parameter C_IN_PORT_WIDTH=3,
    parameter C_OUT_PORT_WIDTH=8,
    parameter C_NUM_INPUTS=5,
    
//    parameter C_MAX_PACKETSIZE=1500,
    parameter C_TIMESLICE_WIDTH=9 //log2((C_MAX_PACKETSIZE*8)/C_DATA_WIDTH)
)
(
    input clk,
    input reset,
    
    input [C_NUM_INPUTS*C_DATA_WIDTH-1:0] s_axis_tdata,
    input [C_NUM_INPUTS*C_TKEEP_WIDTH-1:0] s_axis_tkeep,
    input [C_NUM_INPUTS-1:0] s_axis_tlast,
    input [C_NUM_INPUTS-1:0] s_axis_tvalid,
    input [C_NUM_INPUTS*C_PACKET_LENGTH_WIDTH-1:0] s_axis_tuser_packet_length,
    input [C_NUM_INPUTS*C_IN_PORT_WIDTH-1:0] s_axis_tuser_in_port,
    input [C_NUM_INPUTS*C_OUT_PORT_WIDTH-1:0] s_axis_tuser_out_port,
    input [C_NUM_INPUTS*C_IN_PORT_WIDTH-1:0] s_axis_tuser_in_vport,
    input [C_NUM_INPUTS*C_OUT_PORT_WIDTH-1:0] s_axis_tuser_out_vport,

    input [C_NUM_INPUTS*C_TIMESLICE_WIDTH-1:0] cfg_timeslices,
    output reg [C_NUM_INPUTS-1:0] s_axis_tready,
    
    output reg [C_DATA_WIDTH-1:0] m_axis_tdata,
    output reg [C_TKEEP_WIDTH-1:0] m_axis_tkeep,
    output reg m_axis_tlast,
    output reg m_axis_tvalid=0,
    output reg [C_PACKET_LENGTH_WIDTH-1:0]m_axis_tuser_packet_length,
    output reg [C_IN_PORT_WIDTH-1:0]m_axis_tuser_in_port,
    output reg [C_OUT_PORT_WIDTH-1:0]m_axis_tuser_out_port,
    output reg [C_IN_PORT_WIDTH-1:0]m_axis_tuser_in_vport,
    output reg [C_OUT_PORT_WIDTH-1:0]m_axis_tuser_out_vport,
    input m_axis_tready
);

    function integer log2;
        input integer number;
        begin
            log2=0;
            while(2**log2<number) begin
                log2=log2+1;
            end
        end
    endfunction // log2
    
    reg [C_NUM_INPUTS*C_DATA_WIDTH-1:0] buf_s_axis_tdata;
    reg [C_NUM_INPUTS*C_TKEEP_WIDTH-1:0] buf_s_axis_tkeep;
    reg [C_NUM_INPUTS-1:0] buf_s_axis_tlast;
    reg [C_NUM_INPUTS-1:0] buf_s_axis_tvalid=0;
    reg [C_NUM_INPUTS*C_PACKET_LENGTH_WIDTH-1:0] buf_s_axis_tuser_packet_length;
    reg [C_NUM_INPUTS*C_IN_PORT_WIDTH-1:0] buf_s_axis_tuser_in_port;
    reg [C_NUM_INPUTS*C_OUT_PORT_WIDTH-1:0] buf_s_axis_tuser_out_port;
    reg [C_NUM_INPUTS*C_IN_PORT_WIDTH-1:0] buf_s_axis_tuser_in_vport;
    reg [C_NUM_INPUTS*C_OUT_PORT_WIDTH-1:0] buf_s_axis_tuser_out_vport;
    
    reg [C_DATA_WIDTH-1:0] t_m_axis_tdata;
    reg [C_TKEEP_WIDTH-1:0] t_m_axis_tkeep;
    reg t_m_axis_tlast;
    reg t_m_axis_tvalid=0;
    reg [C_PACKET_LENGTH_WIDTH-1:0] t_m_axis_tuser_packet_length;
    reg [C_IN_PORT_WIDTH-1:0] t_m_axis_tuser_in_port;
    reg [C_OUT_PORT_WIDTH-1:0] t_m_axis_tuser_out_port;
    reg [C_IN_PORT_WIDTH-1:0] t_m_axis_tuser_in_vport;
    reg [C_OUT_PORT_WIDTH-1:0] t_m_axis_tuser_out_vport;
    
    
    reg [log2(C_NUM_INPUTS)-1:0] current_input = 0;
    reg [log2(C_NUM_INPUTS)-1:0] next_input = 0;
    reg [log2(C_NUM_INPUTS)-1:0] current_input_delayed = 0;
    reg startup = 1; // set to 0 after first packet received
    reg [C_TIMESLICE_WIDTH-1:0] counter [C_NUM_INPUTS-1:0];
    
    reg [log2(C_NUM_INPUTS):0] i;
    initial begin
        for(i=0; i<C_NUM_INPUTS; i=i+1) begin
            counter[i]=0;
        end
    end
    reg [log2(C_NUM_INPUTS)-1:0]t_next_input;
    always @(*) begin
        next_input = current_input;
        t_next_input = current_input;
        for(i=0; i<C_NUM_INPUTS; i=i+1) begin
            if(next_input + i < C_NUM_INPUTS) begin
                if(buf_s_axis_tvalid[next_input + i]) begin
                    t_next_input = next_input + i;
                end
            end
            else begin
                if(buf_s_axis_tvalid[next_input + i - C_NUM_INPUTS]) begin
                    t_next_input = next_input + i - C_NUM_INPUTS;
                end
            end
        end
        next_input = t_next_input; 
        for(i=0; i<C_NUM_INPUTS; i=i+1) begin
            s_axis_tready[i]=1'b0;
            if(buf_s_axis_tvalid[i]==0) begin
                s_axis_tready[i]=1'b1;
            end
            if(i==current_input && ((m_axis_tvalid && m_axis_tready) || ~m_axis_tvalid)) begin
                s_axis_tready[i]=1'b1;
            end
        end
        if((m_axis_tvalid && m_axis_tready) || ~m_axis_tvalid) begin
            t_m_axis_tvalid = buf_s_axis_tvalid[current_input];
            t_m_axis_tdata = buf_s_axis_tdata[({1'b0,current_input}+1)*C_DATA_WIDTH-1 -: C_DATA_WIDTH];
            t_m_axis_tkeep = buf_s_axis_tkeep[({1'b0,current_input}+1)*C_TKEEP_WIDTH-1 -: C_TKEEP_WIDTH];
            t_m_axis_tlast = buf_s_axis_tlast[current_input];
            t_m_axis_tuser_packet_length = buf_s_axis_tuser_packet_length[({1'b0,current_input}+1)*C_PACKET_LENGTH_WIDTH-1 -: C_PACKET_LENGTH_WIDTH];
            t_m_axis_tuser_in_port = buf_s_axis_tuser_in_port[({1'b0,current_input}+1)*C_IN_PORT_WIDTH-1 -: C_IN_PORT_WIDTH];
            t_m_axis_tuser_out_port = buf_s_axis_tuser_out_port[({1'b0,current_input}+1)*C_OUT_PORT_WIDTH-1 -: C_OUT_PORT_WIDTH];
            t_m_axis_tuser_in_vport = buf_s_axis_tuser_in_vport[({1'b0,current_input}+1)*C_IN_PORT_WIDTH-1 -: C_IN_PORT_WIDTH];
            t_m_axis_tuser_out_vport = buf_s_axis_tuser_out_vport[({1'b0,current_input}+1)*C_OUT_PORT_WIDTH-1 -: C_OUT_PORT_WIDTH];
        end
        else begin
            t_m_axis_tvalid = 1'b0;
            t_m_axis_tdata = {C_DATA_WIDTH{1'b0}};
            t_m_axis_tkeep = {C_TKEEP_WIDTH{1'b0}};
            t_m_axis_tlast = 1'b0;
            t_m_axis_tuser_packet_length = {C_PACKET_LENGTH_WIDTH{1'b0}};
            t_m_axis_tuser_in_port = {C_IN_PORT_WIDTH{1'b0}};
            t_m_axis_tuser_out_port = {C_OUT_PORT_WIDTH{1'b0}};
            t_m_axis_tuser_in_vport = {C_IN_PORT_WIDTH{1'b0}};
            t_m_axis_tuser_out_vport = {C_OUT_PORT_WIDTH{1'b0}};
        end
    end

    reg last_valid_was_tlast = 0;
    always @(posedge clk) begin
        current_input_delayed <= current_input;
        if(s_axis_tvalid[current_input] && startup) begin
            startup = 0;
        end
        if(t_m_axis_tvalid) begin
            if(t_m_axis_tlast) begin
                last_valid_was_tlast <= 1;
            end
            else begin
                last_valid_was_tlast <=0;
            end
        end
        if((counter[current_input]>=cfg_timeslices[(current_input+1)*C_TIMESLICE_WIDTH-1 -: C_TIMESLICE_WIDTH] || ~s_axis_tvalid[current_input]) && (startup || (t_m_axis_tlast && t_m_axis_tvalid) || (~t_m_axis_tvalid && last_valid_was_tlast))) begin
            current_input <= next_input;
        end
        
        
        
        
        if(counter[current_input]<cfg_timeslices[(current_input+1)*C_TIMESLICE_WIDTH-1 -: C_TIMESLICE_WIDTH]) begin
            counter[current_input] <= counter[current_input]+1;
        end
        if(current_input_delayed != current_input) begin
            counter[current_input_delayed] <= 0;
        end
        for(i=0; i<C_NUM_INPUTS; i=i+1) begin
            if((s_axis_tvalid[i] && s_axis_tready[i]) || ~buf_s_axis_tvalid[i] || (current_input == i && (m_axis_tvalid && m_axis_tready)) ) begin
                buf_s_axis_tvalid[i] <= s_axis_tvalid[i];
                buf_s_axis_tdata[(i+1)*C_DATA_WIDTH-1 -: C_DATA_WIDTH] <= s_axis_tdata[(i+1)*C_DATA_WIDTH-1 -: C_DATA_WIDTH];
                buf_s_axis_tkeep[(i+1)*C_TKEEP_WIDTH-1 -: C_TKEEP_WIDTH] <= s_axis_tkeep[(i+1)*C_TKEEP_WIDTH-1 -: C_TKEEP_WIDTH];
                buf_s_axis_tlast[i] <= s_axis_tlast[i];
                buf_s_axis_tuser_packet_length[(i+1)*C_PACKET_LENGTH_WIDTH-1 -: C_PACKET_LENGTH_WIDTH] <= s_axis_tuser_packet_length[(i+1)*C_PACKET_LENGTH_WIDTH-1 -: C_PACKET_LENGTH_WIDTH];
                buf_s_axis_tuser_in_port[(i+1)*C_IN_PORT_WIDTH-1 -: C_IN_PORT_WIDTH] <= s_axis_tuser_in_port[(i+1)*C_IN_PORT_WIDTH-1 -: C_IN_PORT_WIDTH];
                buf_s_axis_tuser_out_port[(i+1)*C_OUT_PORT_WIDTH-1 -: C_OUT_PORT_WIDTH] <= s_axis_tuser_out_port[(i+1)*C_OUT_PORT_WIDTH-1 -: C_OUT_PORT_WIDTH];
                buf_s_axis_tuser_in_vport[(i+1)*C_IN_PORT_WIDTH-1 -: C_IN_PORT_WIDTH] <= s_axis_tuser_in_vport[(i+1)*C_IN_PORT_WIDTH-1 -: C_IN_PORT_WIDTH];
                buf_s_axis_tuser_out_vport[(i+1)*C_OUT_PORT_WIDTH-1 -: C_OUT_PORT_WIDTH] <= s_axis_tuser_out_vport[(i+1)*C_OUT_PORT_WIDTH-1 -: C_OUT_PORT_WIDTH];
            end
        end
        
        if((m_axis_tvalid && m_axis_tready) || ~m_axis_tvalid) begin
            m_axis_tvalid <= t_m_axis_tvalid;
            m_axis_tdata <= t_m_axis_tdata;
            m_axis_tkeep <= t_m_axis_tkeep;
            m_axis_tlast <= t_m_axis_tlast;
            m_axis_tuser_packet_length <= t_m_axis_tuser_packet_length;
            m_axis_tuser_in_port <= t_m_axis_tuser_in_port;
            m_axis_tuser_out_port <= t_m_axis_tuser_out_port;
            m_axis_tuser_in_vport <= t_m_axis_tuser_in_vport;
            m_axis_tuser_out_vport <= t_m_axis_tuser_out_vport;
        end
        
        if (reset) begin
            m_axis_tvalid <= 0;
            m_axis_tdata <= 0;
            m_axis_tlast <= 0;
            m_axis_tuser_packet_length <= 0;
            m_axis_tuser_in_port <= 0;
            m_axis_tuser_out_port <= 0;
            m_axis_tuser_in_vport <= 0;
            m_axis_tuser_out_vport <= 0;
            buf_s_axis_tvalid <= 0;
            buf_s_axis_tdata <= 0;
            buf_s_axis_tkeep <= 0;
            buf_s_axis_tlast <= 0;
            buf_s_axis_tuser_packet_length <= 0;
            buf_s_axis_tuser_in_port <= 0;
            buf_s_axis_tuser_out_port <= 0;
            buf_s_axis_tuser_in_vport <= 0;
            buf_s_axis_tuser_out_vport <= 0;
            current_input <= 0;
            current_input_delayed <= 0;
            for(i=0; i<C_NUM_INPUTS; i=i+1) begin
                counter[i] <= 0;
            end
        end

    end
    
    
    
endmodule
