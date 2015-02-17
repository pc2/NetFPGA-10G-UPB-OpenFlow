/*
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
`include "tuple_t.v"

/*
Default forwadings to forward DMA packets always to their respective physical ports.
Connect this module with highest priority.
*/

module vport_lookup #(
    parameter C_DMA_FIRST_EXTERNAL_PORT = 0, // 0, nf10av0
    parameter C_DMA_LAST_EXTERNAL_PORT = 0, // 4, nf10av4
    parameter C_DMA_PORT = 0, // 5
    parameter C_BRIDGED_ETH_A_VPORT = 0, // 5, nf10av5
    parameter C_BRIDGED_ETH_B_VPORT = 0 // 6, nf10a0
)(
    input clk,
    input reset,
    input tuple_t tuple_in,
    
    output reg action_match,
    output reg action_valid,
    output reg [C_OUT_PORT_WIDTH-1:0] action_port,
    output reg [C_OUT_PORT_WIDTH-1:0] action_vport,
    output reg [1:0] action_type = 0,
    output reg [C_MATCH_ADDR_WIDTH-1:0] action_match_addr
    );
    
    always_ff @(posedge clk) begin

        if(tuple_in.valid) begin

            action_match_addr <= 0;
            action_valid <= 1;
            action_match <= 0;
            action_port <= 0;
            action_vport <= 0;
            

            
            if (tuple_in.port == C_DMA_PORT) begin // packet comes from the DMA
                if (tuple_in.vport >= C_DMA_FIRST_EXTERNAL_PORT && tuple_in.vport <= C_DMA_LAST_EXTERNAL_PORT) begin // packet comes from the DMA and should go to one of the external ports (10G, Interconnect)
                    action_match_addr <= tuple_in.vport; // TODO: Is this correct here?
                    action_port <= 8'b00000001 << tuple_in.vport;
                    action_vport <= 0;
                    action_match <= 1;
                end
            end
        end
        else begin
            action_valid <= 0;
            action_match <= 0;
        end
    end
endmodule
