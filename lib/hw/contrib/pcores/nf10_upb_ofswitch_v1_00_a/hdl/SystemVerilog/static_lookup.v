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


Default forwadings (static lookups):

10G Port 1 (closest to PCIe slot) (port 0, vport x) <-> (port 5 [dma], vport 0) /dev/nf10av0
10G Port 2 (port 1, vport x) <-> (port 5 [dma], vport 1) /dev/nf10av1
10G Port 3 (port 2, vport x) <-> (port 5 [dma], vport 2) /dev/nf10av2
10G Port 4 (port 3, vport x) <-> (port 5 [dma], vport 3) /dev/nf10av3
Interconnect (top left, J2) (port 4, vport x) <-> (port 5 [dma], vport 4) /dev/nf10av4
/dev/nf10a0 (port 5 [dma], vport 6) <-> (port 5 [dma], vport 5) /dev/nf10av5

*/

module static_lookup #(
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
            action_match <= 1; // default (soft error case): no match - drop packet
            action_port <= 0;
            action_vport <= 0;
            

            if (tuple_in.port >= C_DMA_FIRST_EXTERNAL_PORT && tuple_in.port <= C_DMA_LAST_EXTERNAL_PORT) begin // packet comes from one of the external ports (10G, Interconnect)
                action_match_addr <= tuple_in.port;
                action_port <= 8'b00000001 << C_DMA_PORT;
                action_vport <= 8'b00000001 << tuple_in.port;
                action_match <= 1;

            end else begin
                if (tuple_in.port == C_DMA_PORT) begin // packet comes from the DMA

                    if (tuple_in.vport >= C_DMA_FIRST_EXTERNAL_PORT && tuple_in.vport <= C_DMA_LAST_EXTERNAL_PORT) begin // packet should go to one of the external ports (10G, Interconnect)
                        action_match_addr <= tuple_in.port;
                        action_port <= 8'b00000001 << tuple_in.vport;
                        action_vport <= 0;
                        action_match <= 1;

                    end else begin

                        // just bridge DMA's vport C_BRIDGED_ETH_A_VPORT (5) C_BRIDGED_ETH_B_VPORT (6)

                        if (tuple_in.vport == C_BRIDGED_ETH_A_VPORT) begin
                            action_match_addr <= tuple_in.port;
                            action_port <= 8'b00000001 << C_DMA_PORT;
                            action_vport <= 8'b00000001 << C_BRIDGED_ETH_B_VPORT;
                            action_match <= 1;

                        end else if (tuple_in.vport == C_BRIDGED_ETH_B_VPORT) begin
                            action_match_addr <= tuple_in.port;
                            action_port <= 8'b00000001 << C_DMA_PORT;
                            action_vport <= 8'b00000001 << C_BRIDGED_ETH_A_VPORT;
                            action_match <= 1;

                        end
                    end
                end
            end
        end
        else begin
            action_valid <= 0;
            action_match <= 0;
        end
    end
endmodule
