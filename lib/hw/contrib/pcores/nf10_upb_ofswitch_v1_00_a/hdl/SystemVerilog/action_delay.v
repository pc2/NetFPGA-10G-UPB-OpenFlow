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



`include "parameters.v"
module action_delay #(
    parameter C_NUM_DELAY_CYCLES=5,
    parameter C_WIDTH=C_IN_PORT_WIDTH*2+2+C_MATCH_ADDR_WIDTH
)(
    input clk,
    input reset,
    
    input [C_WIDTH-1:0] inp,
    output [C_WIDTH-1:0] outp
);


logic [C_WIDTH-1:0]delay[C_NUM_DELAY_CYCLES-1:0];

generate
    for(genvar i = 0; i < C_NUM_DELAY_CYCLES-1; i = i+1) begin
        always_ff @(posedge clk) begin
            delay[i] <= delay[i+1];
        end
    end
endgenerate


assign delay[C_NUM_DELAY_CYCLES-1] = inp;
assign outp = delay[0];

endmodule
