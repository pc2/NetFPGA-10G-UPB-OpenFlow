/*
 * UPB frame_gen
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
 
 /*
  * Only for testbenches, not synthesizable!
  * 
  * Use this in combination with frame_check to generate random packets with
  * random spaceing between them to check whether a user logic module 
  * forwards packets correctly.
  */
`timescale 1ns / 1ps
module frame_gen #(
    parameter C_INPORT_WIDTH = 3,
    parameter C_OUTPORT_WIDTH = 8,
    parameter C_PACKET_LENGTH_WIDTH = 14,
    parameter C_AXIS_DATA_WIDTH = 256, //currently hardcoded. if you want to change this remember to change m_axis_tdata generation too
    parameter C_MAX_PACKET_SIZE = 9000,
    parameter C_IN_PORT = 0,
    parameter C_IN_VPORT = 0,
    parameter C_OUT_PORT = 0,
    parameter C_OUT_VPORT = 0,
    parameter C_MAX_IDLE_CYCLES = 10
    )
    (
    
    // axi clock and reset //
    input clk,
    input axi_resetn,
    // Master Stream Ports
    output reg [C_AXIS_DATA_WIDTH-1:0] m_axis_tdata,
    output reg [(C_AXIS_DATA_WIDTH/8)-1:0] m_axis_tkeep,
    output reg [C_PACKET_LENGTH_WIDTH-1:0] m_axis_tuser_packet_length,
    output reg [C_INPORT_WIDTH-1:0] m_axis_tuser_in_port = C_IN_PORT,
    output reg [C_OUTPORT_WIDTH-1:0] m_axis_tuser_out_port = C_OUT_PORT,
    output reg [C_INPORT_WIDTH-1:0] m_axis_tuser_in_vport = C_IN_VPORT,
    output reg [C_OUTPORT_WIDTH-1:0] m_axis_tuser_out_vport = C_OUT_VPORT,
    output reg m_axis_tvalid=0,
    input m_axis_tready,
    output reg m_axis_tlast
    );
    `include "tkeep_coder.v"
    reg [31:0] seed;
    reg [C_AXIS_DATA_WIDTH-1:0] payload;
    reg [C_PACKET_LENGTH_WIDTH-1:0] tosend=0;
    reg [C_PACKET_LENGTH_WIDTH-1:0] length;
    reg [31:0] pkt_counter = 0;
    reg invert = 0;
    reg [31:0] sleep = 0;
    always @(posedge clk) begin
        if(tosend==0 && (sleep == 0 ) && (C_MAX_IDLE_CYCLES > 0 )) begin
            sleep = $unsigned($random) % C_MAX_IDLE_CYCLES;
            m_axis_tvalid = 0;
            m_axis_tlast = 0;
        end
        if(sleep > 0 )
            sleep = sleep -1;
        if((m_axis_tvalid && m_axis_tready || !m_axis_tvalid) && sleep == 0) begin
            if ( tosend == 0) begin
                pkt_counter = pkt_counter + 1;
                seed = $unsigned($random);
                m_axis_tuser_packet_length = seed[C_PACKET_LENGTH_WIDTH-1:0] % C_MAX_PACKET_SIZE;
                if (m_axis_tuser_packet_length < 64) begin
                    m_axis_tuser_packet_length = 64;
                end
                tosend=m_axis_tuser_packet_length;
                m_axis_tdata = {pkt_counter,seed,~seed,seed,~seed,seed,~seed,seed};
                invert = 1;
            end else begin
                     if(invert)
                        m_axis_tdata = {~seed,seed,~seed,seed,~seed,seed,~seed,seed};
                     else
                        m_axis_tdata = {seed,~seed,seed,~seed,seed,~seed,seed,~seed};
                     invert = ~invert;
            end
            m_axis_tkeep = {(C_AXIS_DATA_WIDTH/8){1'b1}};
            if(tosend < (C_AXIS_DATA_WIDTH/8)) begin
                m_axis_tkeep = decode(tosend-1);
            end
            if(tosend<=(C_AXIS_DATA_WIDTH/8))
                m_axis_tlast = 1;
            else
                m_axis_tlast = 0;
               if(tosend >=(C_AXIS_DATA_WIDTH/8))
                tosend = tosend - (C_AXIS_DATA_WIDTH/8);
            else
                tosend = 0;
            m_axis_tvalid = 1;
        end
        if(axi_resetn) begin
            seed <= 0;
            payload <= 0;
            tosend <=0;
            length <=0;
            pkt_counter <= 0;
            invert <= 0;
            sleep <= 0;
        end
    end
endmodule
