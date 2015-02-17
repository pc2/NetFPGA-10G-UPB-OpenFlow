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
 *  Selector to select which action to use.
 * Note: all valid signals have to be synchronized and have to be set 
 * for exactly one cycle!
 */
`include "parameters.v"
module selector #(
    parameter C_NUM_INPUTS=2
)(
    input clk, // actually, we don't use a clock...
    input reset, // nor a reset.
    logic[1:0] atype [C_NUM_INPUTS-1:0], //type is reserved so this signal is now called atype :/
    logic[C_OUT_PORT_WIDTH-1:0] port [C_NUM_INPUTS-1:0],
    logic[C_OUT_PORT_WIDTH-1:0] vport [C_NUM_INPUTS-1:0],
    logic match [C_NUM_INPUTS-1:0],
    logic[C_MATCH_ADDR_WIDTH-1:0] match_addr [C_NUM_INPUTS-1:0],
    logic valid [C_NUM_INPUTS-1:0],
    
    output [1:0] action_selected_type,
    output action_selected_match,
    output action_selected_valid,
    output [C_OUT_PORT_WIDTH-1:0] action_selected_port,
    output [C_OUT_PORT_WIDTH-1:0] action_selected_vport,
    output [C_MATCH_ADDR_WIDTH-1:0] action_selected_match_addr
);

    generate 
        for(genvar i = C_NUM_INPUTS-1; i>=0; i--) begin: chain
            logic chain_in_match;
            logic chain_in_valid;
            logic[1:0] chain_in_type;
            logic[C_MATCH_ADDR_WIDTH-1:0] chain_in_match_addr;
            logic[C_OUT_PORT_WIDTH-1:0] chain_in_port;
            logic[C_OUT_PORT_WIDTH-1:0] chain_in_vport;
            logic chain_out_match;
            logic chain_out_valid;
            logic[1:0] chain_out_type;
            logic[C_MATCH_ADDR_WIDTH-1:0] chain_out_match_addr;
            logic[C_OUT_PORT_WIDTH-1:0] chain_out_port;
            logic[C_OUT_PORT_WIDTH-1:0] chain_out_vport;
            logic inp_match;
            logic inp_valid;
            logic[1:0] inp_type;
            logic[C_MATCH_ADDR_WIDTH-1:0] inp_match_addr;
            logic[C_OUT_PORT_WIDTH-1:0] inp_port;
            logic[C_OUT_PORT_WIDTH-1:0] inp_vport;
            assign inp_vport = vport[i];
            assign inp_port = port[i];
            assign inp_match = match[i];
            assign inp_match_addr = match_addr[i];
            assign inp_valid = valid[i];
            assign inp_type = atype[i];
            combinechain chain(
                .action_valid(inp_valid),
                .action_match(inp_match),
                .action_match_addr(inp_match_addr),
                .action_port(inp_port),
                .action_vport(inp_vport),
                .action_type(inp_type),
                .chain_out_type(chain_out_type),
                .chain_out_valid(chain_out_valid),
                .chain_out_match(chain_out_match),
                .chain_out_match_addr(chain_out_match_addr),
                .chain_out_port(chain_out_port),
                .chain_out_vport(chain_out_vport),
                .chain_in_type(chain_in_type),
                .chain_in_valid(chain_in_valid),
                .chain_in_match(chain_in_match),
                .chain_in_match_addr(chain_in_match_addr),
                .chain_in_port(chain_in_port),
                .chain_in_vport(chain_in_vport)
            );
        end
        for(genvar i = C_NUM_INPUTS-2; i>=0; i--) begin: chaining
            assign chain[i].chain_in_port = chain[i+1].chain_out_port;
            assign chain[i].chain_in_vport = chain[i+1].chain_out_vport;
            assign chain[i].chain_in_match = chain[i+1].chain_out_match;
            assign chain[i].chain_in_match_addr = chain[i+1].chain_out_match_addr;
            assign chain[i].chain_in_valid = chain[i+1].chain_out_valid;
            assign chain[i].chain_in_type = chain[i+1].chain_out_type;
        end
    endgenerate
    assign chain[C_NUM_INPUTS-1].chain_in_valid = valid[C_NUM_INPUTS-1];
    assign chain[C_NUM_INPUTS-1].chain_in_match = 0;
    assign action_selected_match = chain[0].chain_out_match;
    assign action_selected_match_addr = chain[0].chain_out_match_addr;
    assign action_selected_valid = chain[0].chain_out_valid;
    assign action_selected_port = chain[0].chain_out_port;
    assign action_selected_vport = chain[0].chain_out_vport;
    assign action_selected_type = chain[0].chain_out_type;



endmodule
