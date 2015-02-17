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

`timescale 1ns / 1ps


module nf10_upb_input_arbiter_tb();

    // Inputs
    reg clk;
    
    reg [255:0] s_axis_tdata_0 = 256'hA0A0A0A0A0A0A0A0A0A0A0A0A0A0A0A0A0A0A0A0A0A0A0A0A0A0A0A0A0A0A0A0;
    reg [255:0] s_axis_tdata_1 = 256'hB1B1B1B1B1B1B1B1B1B1B1B1B1B1B1B1B1B1B1B1B1B1B1B1B1B1B1B1B1B1B1B1;
    reg [255:0] s_axis_tdata_2 = 256'hC2C2C2C2C2C2C2C2C2C2C2C2C2C2C2C2C2C2C2C2C2C2C2C2C2C2C2C2C2C2C2C2;
    reg [255:0] s_axis_tdata_3 = 256'hD3D3D3D3D3D3D3D3D3D3D3D3D3D3D3D3D3D3D3D3D3D3D3D3D3D3D3D3D3D3D3D3;
    wire [1023:0] s_axis_tdata = {s_axis_tdata_3,s_axis_tdata_2,s_axis_tdata_1,s_axis_tdata_0};
    
    reg [35:0] cfg_timeslices = {9'd47,9'd47,9'd47,9'd47};
    
    reg [31:0] s_axis_tkeep_0 = 0;
    reg [31:0] s_axis_tkeep_1 = 1;
    reg [31:0] s_axis_tkeep_2 = 2;
    reg [31:0] s_axis_tkeep_3 = 3;
    wire [127:0] s_axis_tkeep = {s_axis_tkeep_3,s_axis_tkeep_2,s_axis_tkeep_1,s_axis_tkeep_0};
    
    reg s_axis_tlast_0 = 1'b0;
    reg s_axis_tlast_1 = 1'b0;
    reg s_axis_tlast_2 = 1'b0;
    reg s_axis_tlast_3 = 1'b0;
    wire [3:0] s_axis_tlast = {s_axis_tlast_3,s_axis_tlast_2,s_axis_tlast_1,s_axis_tlast_0};
    
    reg s_axis_tvalid_0 = 1'b0;
    reg s_axis_tvalid_1 = 1'b0;
    reg s_axis_tvalid_2 = 1'b0;
    reg s_axis_tvalid_3 = 1'b0;
    wire [3:0] s_axis_tvalid = {s_axis_tvalid_3,s_axis_tvalid_2, s_axis_tvalid_1, s_axis_tvalid_0};
    
    reg [13:0] s_axis_tuser_packet_length_0;
    reg [13:0] s_axis_tuser_packet_length_1;
    reg [13:0] s_axis_tuser_packet_length_2;
    reg [13:0] s_axis_tuser_packet_length_3;
    wire [55:0] s_axis_tuser_packet_length = {s_axis_tuser_packet_length_3,s_axis_tuser_packet_length_2,s_axis_tuser_packet_length_1,s_axis_tuser_packet_length_0};
    
    reg [2:0] s_axis_tuser_in_port_0 = 0;
    reg [2:0] s_axis_tuser_in_port_1 = 1;
    reg [2:0] s_axis_tuser_in_port_2 = 2;
    reg [2:0] s_axis_tuser_in_port_3 = 3;
    wire [11:0] s_axis_tuser_in_port = {s_axis_tuser_in_port_3,s_axis_tuser_in_port_2,s_axis_tuser_in_port_1,s_axis_tuser_in_port_0};
    
    reg [7:0] s_axis_tuser_out_port_0 = 0;
    reg [7:0] s_axis_tuser_out_port_1 = 1;
    reg [7:0] s_axis_tuser_out_port_2 = 2;
    reg [7:0] s_axis_tuser_out_port_3 = 3;
    wire [31:0] s_axis_tuser_out_port = {s_axis_tuser_out_port_3,s_axis_tuser_out_port_2,s_axis_tuser_out_port_1,s_axis_tuser_out_port_0};
    
    reg m_axis_tready;
    
    reg reset = 0;

    // Outputs
    wire [3:0] s_axis_tready;
    wire [255:0] m_axis_tdata;
    wire [31:0] m_axis_tkeep;
    wire m_axis_tlast;
    wire m_axis_tvalid;
    wire [13:0] m_axis_tuser_packet_length;
    wire [2:0] m_axis_tuser_in_port;
    wire [7:0] m_axis_tuser_out_port;
    defparam uut.C_NUM_INPUTS = 4;
    // Instantiate the Unit Under Test (UUT)
    nf10_upb_input_arbiter_flex uut (
        .clk(clk), 
        .s_axis_tdata(s_axis_tdata), 
        .s_axis_tkeep(s_axis_tkeep), 
        .s_axis_tlast(s_axis_tlast), 
        .s_axis_tvalid(s_axis_tvalid), 
        .s_axis_tuser_packet_length(s_axis_tuser_packet_length), 
        .s_axis_tuser_in_port(s_axis_tuser_in_port), 
        .s_axis_tuser_out_port(s_axis_tuser_out_port), 
        .s_axis_tready(s_axis_tready), 
        .m_axis_tdata(m_axis_tdata), 
        .m_axis_tkeep(m_axis_tkeep), 
        .m_axis_tlast(m_axis_tlast), 
        .m_axis_tvalid(m_axis_tvalid), 
        .m_axis_tuser_packet_length(m_axis_tuser_packet_length), 
        .m_axis_tuser_in_port(m_axis_tuser_in_port), 
        .m_axis_tuser_out_port(m_axis_tuser_out_port), 
        .m_axis_tready(m_axis_tready),
        .cfg_timeslices(cfg_timeslices),
        .reset(reset)
    );

    localparam IDLE = 0;
    localparam WORD1 = 1;
    localparam WORD2 = 2;
    localparam WORD3 = 3;
    localparam SLEEP = 4;
    reg [2:0] state0 = WORD1;
    reg [2:0] next_state0 = WORD1;
    reg [2:0] state2 = IDLE;
    reg [2:0] next_state2 = IDLE;
    // 1 Frame = 3 words.
    always @(*) begin
        case(state0)
            IDLE: begin
                s_axis_tvalid_0 = 1'b0;
            end
            WORD1: begin
                s_axis_tdata_0 = 256'hA0A0A0A0A0A0A0A0A0A0A0A0A0A0A0A0A0A0A0A0A0A0A0A0A0A0A0A0A0A0A0A0;
                s_axis_tkeep_0 = 5'b00000;
                s_axis_tlast_0 = 1'b0;
                s_axis_tvalid_0 = 1'b1;
                s_axis_tuser_packet_length_0 = 0;
                s_axis_tuser_out_port_0 = 0;
                s_axis_tuser_in_port_0 = 0;
            end
            WORD2: begin
                s_axis_tdata_0 = 256'hB0B0B0B0B0B0B0B0B0B0B0B0B0B0B0B0B0B0B0B0B0B0B0B0B0B0B0B0B0B0B0B0;
                s_axis_tkeep_0 = 5'b00000;
                s_axis_tlast_0 = 1'b0;
                s_axis_tvalid_0 = 1'b1;
                s_axis_tuser_packet_length_0 = 0;
                s_axis_tuser_out_port_0 = 0;
                s_axis_tuser_in_port_0 = 0;
            end
            WORD3: begin
                s_axis_tdata_0 = 256'hC0C0C0C0C0C0C0C0C0C0C0C0C0C0C0C0C0C0C0C0C0C0C0C0C0C0C0C0C0C0C0C0;
                s_axis_tkeep_0 = 5'b00000;
                s_axis_tlast_0 = 1'b1;
                s_axis_tvalid_0 = 1'b1;
                s_axis_tuser_packet_length_0 = 0;
                s_axis_tuser_out_port_0 = 0;
                s_axis_tuser_in_port_0 = 0;
            end
            SLEEP: begin
                s_axis_tvalid_0 = 1'b0;
            end
        endcase
        
        case(state2)
            IDLE: begin
                s_axis_tvalid_2 = 1'b0;
            end
            WORD1: begin
                s_axis_tdata_2 = 256'hA2A2A2A2A2A2A2A2A2A2A2A2A2A2A2A2A2A2A2A2A2A2A2A2A2A2A2A2A2A2A2A2;
                s_axis_tkeep_2 = 5'b00010;
                s_axis_tlast_2 = 1'b0;
                s_axis_tvalid_2 = 1'b1;
                s_axis_tuser_packet_length_2 = 2;
                s_axis_tuser_out_port_2 = 2;
                s_axis_tuser_in_port_2 = 2;
            end
            WORD2: begin
                s_axis_tdata_2 = 256'hB2B2B2B2B2B2B2B2B2B2B2B2B2B2B2B2B2B2B2B2B2B2B2B2B2B2B2B2B2B2B2B2;
                s_axis_tkeep_2 = 5'b00010;
                s_axis_tlast_2 = 1'b0;
                s_axis_tvalid_2 = 1'b1;
                s_axis_tuser_packet_length_2 = 2;
                s_axis_tuser_out_port_2 = 2;
                s_axis_tuser_in_port_2 = 2;
            end
            WORD3: begin
                s_axis_tdata_2 = 256'hC2C2C2C2C2C2C2C2C2C2C2C2C2C2C2C2C2C2C2C2C2C2C2C2C2C2C2C2C2C2C2C2;
                s_axis_tkeep_2 = 5'b00010;
                s_axis_tlast_2 = 1'b1;
                s_axis_tvalid_2 = 1'b1;
                s_axis_tuser_packet_length_2 = 0;
                s_axis_tuser_out_port_2 = 0;
                s_axis_tuser_in_port_2 = 0;
            end
            SLEEP: begin
                s_axis_tvalid_2 = 1'b0;
            end
        endcase
    end


    integer counter = 0;
    integer counter2 = 0;
    reg sleeped=0;
    reg [10:0] sleep_timer=0;
    reg [2:0] sleep_state;
    always @(posedge clk) begin
    
        // input0 sends continuous datastream of 50 frames
        if(s_axis_tready[0]==1'b1) begin // okay to wait for tready as we always issue tvalid
            next_state0 = state0+1'b1;
            if(state0==WORD3) begin
                next_state0 = WORD1;
                counter = counter + 1;
            end
            if(counter >= 50) begin
                next_state0 = IDLE;
            end
        end
        
        
        // input2 sends 20 frames. 
        // Frame 10 is interrupted by 5 cycles (input arbiter must not switch as
        // frame is incomplete) and Frame 17 starts with 1 cycle delay (input arbiter
        // must switch as a gap of 1 cycle is interpreted as lack of packets)
        if(state2==WORD3 && counter2!=16 && s_axis_tready[2]) begin
            next_state2 = WORD1; 
            counter2 = counter2 + 1;
        end
        else begin
            if(state2==WORD3 && counter2 == 16) begin
                next_state2 = SLEEP;
                sleep_timer=0;
                counter2= counter2+1;
                sleep_state= WORD1;
            end
            else begin
                if(state2 == WORD2 && counter2 == 10  && s_axis_tready[2]) begin
                    next_state2 = SLEEP;
                    sleep_timer = 5;
                    sleep_state = WORD3;
                end
                else begin
                    if(state2 == SLEEP && sleep_timer>0) begin
                        sleep_timer = sleep_timer - 1;
                    end
                    else begin
                        if(state2 == SLEEP) begin
                            next_state2 = sleep_state;
                        end
                        else begin
                            if(state2==IDLE || s_axis_tready[2]) begin
                            next_state2 = state2+1'b1;
                            end
                        end
                    end
                end
            end
            
        end
        if((counter2==20 && (state2 == WORD3 || state2 == IDLE)) || counter2>20) begin
            next_state2 = IDLE;
        end
        state0 <= next_state0;
        state2 <= next_state2;
    end
    initial begin
        // Initialize Inputs
        clk = 0;
        m_axis_tready = 1;
        // Wait 100 ns for global reset to finish
        #100;
        
        // Add stimulus here
        #1100
        //next_state2 = WORD1;
        #2200
        @(posedge clk);
        m_axis_tready = 0;
        #400
        @(posedge clk);
        m_axis_tready = 1;

    end
    
    always #100 clk = ~clk;
   
endmodule


