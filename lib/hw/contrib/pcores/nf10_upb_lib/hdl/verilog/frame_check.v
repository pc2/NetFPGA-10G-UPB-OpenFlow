/*
 * UPB frame_check
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
  * Use this in combination with frame_gen to generate random packets with
  * random spaceing between them to check whether a user logic module 
  * forwards packets correctly.
  * Look at error_counter and pkt_counter.
  */
module frame_check #(
    parameter C_INPORT_WIDTH = 3,
    parameter C_OUTPORT_WIDTH = 8,
    parameter C_AXIS_DATA_WIDTH = 256, //currently hardcoded. if you want to change this remember to change s_axis_tdata validation too
    parameter C_PACKET_LENGTH_WIDTH = 14
    )
    (
    
    // axi clock and reset //
    input clk,
    input axi_resetn,
    input [C_AXIS_DATA_WIDTH-1:0] s_axis_tdata,
    input [(C_AXIS_DATA_WIDTH/8)-1:0] s_axis_tkeep,
    input [C_PACKET_LENGTH_WIDTH-1:0] s_axis_tuser_packet_length,
    input [C_INPORT_WIDTH-1:0] s_axis_tuser_in_port,
    input [C_OUTPORT_WIDTH-1:0] s_axis_tuser_out_port,
    input [C_INPORT_WIDTH-1:0] s_axis_tuser_in_vport,
    input [C_OUTPORT_WIDTH-1:0] s_axis_tuser_out_vport,
    input s_axis_tvalid,
    output reg s_axis_tready=1,
    input s_axis_tlast
    );
    `include "tkeep_coder.v"
    reg [31:0] error_counter = 0;
    reg [31:0] int_pkt_counter = 0;
    reg invert = 0;
    reg [2:0] state = 0;
    reg [31:0] seed;
    reg [C_PACKET_LENGTH_WIDTH-1:0] torecv;
    reg [31:0] pkt_counter = 0;
    localparam START = 0;
    localparam RECV = 1;
    localparam ERROR = 2;
    always @(posedge clk) begin
        if(s_axis_tvalid) begin
            if (state == START) begin
                int_pkt_counter = int_pkt_counter + 1;
                seed = s_axis_tdata[32:0];
                pkt_counter = s_axis_tdata[255-:32];
                if(int_pkt_counter > pkt_counter) begin
                    $display("dropped packet");
                    error_counter = error_counter + int_pkt_counter - pkt_counter;
                    int_pkt_counter = pkt_counter;
                end
                torecv=s_axis_tuser_packet_length;
                if(s_axis_tdata != {pkt_counter,seed,~seed,seed,~seed,seed,~seed,seed} ) begin
                    $display("faulty first packet");
                    error_counter = error_counter + 1;
                    state = ERROR;
                end
                else begin
                    state = RECV;
                    torecv = torecv-32;
                end
                invert = 1;
            end
            else begin
                if (state == RECV) begin
                    if (invert==1) begin
                        if (s_axis_tdata != {~seed,seed,~seed,seed,~seed,seed,~seed,seed}) begin
                            $display("faulty packet");
                            error_counter = error_counter + 1;
                            state = ERROR;
                        end
                        else begin
                            if(s_axis_tkeep == {(C_AXIS_DATA_WIDTH/8){1'b1}}) begin
                                torecv = torecv-C_AXIS_DATA_WIDTH/8;
                            end else begin
                                torecv = torecv - (encode(s_axis_tkeep)+1);
                            end
                        end
                    end else begin
                        if (s_axis_tdata != {seed,~seed,seed,~seed,seed,~seed,seed,~seed}) begin
                            $display("faulty packet");
                            error_counter = error_counter + 1;
                            state = ERROR;
                        end
                        else begin
                            if(s_axis_tkeep == {(C_AXIS_DATA_WIDTH/8){1'b1}}) begin
                                torecv = torecv-C_AXIS_DATA_WIDTH/8;
                            end else begin
                                torecv = torecv - (encode(s_axis_tkeep)+1);
                            end
                        end
                    end
                    invert = ~invert;
                    if (((torecv == 0) && !s_axis_tlast) || (s_axis_tlast && (torecv > 0 ))) begin
                        $display("tlast at wrong position");
                        error_counter = error_counter + 1;
                        state = ERROR;
                    end
                    if (s_axis_tlast==1)
                        state = START;
                end
            end
            if (state == ERROR) begin
                if (s_axis_tlast ==1)
                    state = START;
            end
        end
        if(axi_resetn) begin
             error_counter <= 0;
            int_pkt_counter <= 0;
            invert <= 0;
            state <= 0;
            seed <= 0;
            torecv <=0;
            pkt_counter <= 0;
        end
    end
endmodule
