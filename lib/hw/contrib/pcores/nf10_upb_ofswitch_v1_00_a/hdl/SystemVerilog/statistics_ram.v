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

/*
 * Please have a look at UG621 and/or UG190 for correct widths of BRAM primitive in- and outputs
 */
module statistics_ram #(
    parameter C_BRAM_PRIMITIVE_ADDR_WIDTH = 10, // addr width of bram primitive
    parameter C_NUM_BRAMS = 2**(C_MATCH_ADDR_WIDTH - C_BRAM_PRIMITIVE_ADDR_WIDTH)
    ) (
    input clka,
    input clkb,
    input reset,
    
    input [C_MATCH_ADDR_WIDTH-1:0] addra,
    input [C_MATCH_ADDR_WIDTH-1:0] addrb,
    
    input wea,
    input web,
    
    
    input[C_COUNTER_WIDTH-1:0] dina,
    input[C_COUNTER_WIDTH-1:0] dinb,
    
    output[C_COUNTER_WIDTH-1:0] douta,
    output[C_COUNTER_WIDTH-1:0] doutb
);

logic [C_NUM_BRAMS-1:0] bram_selector_a;
logic [C_NUM_BRAMS-1:0] bram_selector_a_d;
logic [C_NUM_BRAMS-1:0] bram_selector_b;
logic [C_NUM_BRAMS-1:0] bram_selector_b_d;
generate 
    for(genvar i = 0; i<C_NUM_BRAMS; i++) begin: bram
        logic [C_COUNTER_WIDTH-1:0] douta;
        logic [C_COUNTER_WIDTH-1:0] doutb;
        logic [C_COUNTER_WIDTH-1:0] douta_t;
        logic [C_COUNTER_WIDTH-1:0] doutb_t;
        BRAM_TDP_MACRO #(
            .BRAM_SIZE("36Kb"), // Target BRAM: "18Kb" or "36Kb"
            .DEVICE("VIRTEX5"), // Target device: "VIRTEX5", "VIRTEX6", "SPARTAN6"
            .DOA_REG(0),
            // Optional port A output register (0 or 1)
            .DOB_REG(0),
            // Optional port B output register (0 or 1)
            .INIT_A(36'h000000000), // Initial values on port A output port
            .INIT_B(36'h0DEADD00D), // Initial values on port B output port
            .INIT_FILE ("NONE"),
            .READ_WIDTH_A (C_COUNTER_WIDTH),    // Valid values are 1-36 (19-36 only valid when BRAM_SIZE="36Kb")
            .READ_WIDTH_B (C_COUNTER_WIDTH),    // Valid values are 1-36 (19-36 only valid when BRAM_SIZE="36Kb")
            .SIM_COLLISION_CHECK ("ALL"), // Collision check enable "ALL", "WARNING_ONLY",
                                         //"GENERATE_X_ONLY" or "NONE"
            .SIM_MODE("SAFE"), // Simulation: "SAFE" vs. "FAST", see "Synthesis and Simulation Design Guide" for details
            .SRVAL_A(36'h00000000), // Set/Reset value for port A output
            .SRVAL_B(36'h00000000), // Set/Reset value for port B output
            .WRITE_MODE_A("READ_FIRST"), // "WRITE_FIRST", "READ_FIRST", or "NO_CHANGE"
            .WRITE_MODE_B("READ_FIRST"), // "WRITE_FIRST", "READ_FIRST", or "NO_CHANGE"
            .WRITE_WIDTH_A(C_COUNTER_WIDTH), // Valid values are 1-36 (19-36 only valid when BRAM_SIZE="36Kb")
            .WRITE_WIDTH_B(C_COUNTER_WIDTH) // Valid values are 1-36 (19-36 only valid when BRAM_SIZE="36Kb")
            
            ) BRAM_TDP_MACRO_inst (
            .DOA(douta_t),    // Output port-A data
            .DOB(doutb_t),    // Output port-B data
            .ADDRA(addra),    // Input port-A address
            .ADDRB(addrb),    // Input port-B address
            .CLKA(clka),    // Input port-A clock
            .CLKB(clkb),    // Input port-B clock
            .DIA(dina),    // Input port-A data
            .DIB(dinb),    // Input port-B data
            .ENA(bram_selector_a[i]),    // Input port-A enable
            .ENB(bram_selector_b[i]),    // Input port-B enable
            .REGCEA(0), // Input port-A output register enable
            .REGCEB(0), // Input port-B output register enable
            .RSTA(reset),
            .RSTB(reset),
            .WEA({4{bram_selector_a[i] && wea}}),
            .WEB({4{bram_selector_b[i] && web}})
        );
    end
    assign bram[C_NUM_BRAMS-1].douta = (bram[C_NUM_BRAMS-1].douta_t & {C_COUNTER_WIDTH{bram_selector_a_d[C_NUM_BRAMS-1]}});
    assign bram[C_NUM_BRAMS-1].doutb = (bram[C_NUM_BRAMS-1].doutb_t & {C_COUNTER_WIDTH{bram_selector_b_d[C_NUM_BRAMS-1]}});
    for(genvar i = C_NUM_BRAMS-2; i>=0; i--) begin: chaining
        assign bram[i].doutb = bram[i+1].doutb | (bram[i].doutb_t & {C_COUNTER_WIDTH{bram_selector_b_d[i]}});
        assign bram[i].douta = bram[i+1].douta | (bram[i].douta_t & {C_COUNTER_WIDTH{bram_selector_a_d[i]}});
    end
    assign douta = bram[0].douta;
    assign doutb = bram[0].doutb;

endgenerate

always_comb begin
    bram_selector_a = 32'b00000000000000000000000000000001 << addra[C_MATCH_ADDR_WIDTH-1:C_BRAM_PRIMITIVE_ADDR_WIDTH];
    bram_selector_b = 32'b00000000000000000000000000000001 << addrb[C_MATCH_ADDR_WIDTH-1:C_BRAM_PRIMITIVE_ADDR_WIDTH];
end

always_ff @(posedge clkb) begin
    bram_selector_a_d <= bram_selector_a;
    bram_selector_b_d <= bram_selector_b;
end
endmodule
