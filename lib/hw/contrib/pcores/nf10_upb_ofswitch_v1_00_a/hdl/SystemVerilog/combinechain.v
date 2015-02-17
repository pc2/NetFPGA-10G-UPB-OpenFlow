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
 * Chain Module which is used by selector.v to determine match with highest
 * priority.
 * Connect chain_in inputs to chain_out outputs of module with lower priority,
 * chain_out outputs chain_in signals if there is no valid match on action 
 * inputs.
 */
`include "parameters.v"
module combinechain(
    input[C_OUT_PORT_WIDTH-1:0] action_port,
    input[C_OUT_PORT_WIDTH-1:0] action_vport,
    input action_match,
    input[C_MATCH_ADDR_WIDTH-1:0] action_match_addr,
    input action_valid,
    input[1:0] action_type,
    input[C_OUT_PORT_WIDTH-1:0] chain_in_port,
    input[C_OUT_PORT_WIDTH-1:0] chain_in_vport,
    input chain_in_match,
    input[1:0] chain_in_type,
    input[C_MATCH_ADDR_WIDTH-1:0] chain_in_match_addr,
    input chain_in_valid,
    output reg [1:0] chain_out_type,
    output reg [C_OUT_PORT_WIDTH-1:0] chain_out_port,
    output reg [C_OUT_PORT_WIDTH-1:0] chain_out_vport,
    output reg chain_out_match,
    output reg [C_MATCH_ADDR_WIDTH-1:0] chain_out_match_addr,
    output reg chain_out_valid
);

    always_comb begin
        if(chain_in_valid) begin
            if(action_valid && action_match) begin
                chain_out_port = action_port;
                chain_out_vport = action_vport;
                chain_out_type = action_type;
                chain_out_match_addr = action_match_addr;
                chain_out_valid = 1;
                chain_out_match = 1;
            end
            else begin
                chain_out_valid = chain_in_valid;
                chain_out_match = chain_in_match;
                chain_out_port = chain_in_port;
                chain_out_type = chain_in_type;
                chain_out_match_addr = chain_in_match_addr;
                chain_out_vport = chain_in_vport;
            end
        end
        else begin
            chain_out_valid = 0;
            chain_out_match = 0;
            chain_out_port = 0;
            chain_out_type = 0;
            chain_out_vport = 0;
            chain_out_match_addr = 0;
        end
    end
endmodule
