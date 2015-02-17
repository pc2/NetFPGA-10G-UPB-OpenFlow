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
 * RAM Module which has one read-only and one write-only interface.
 * Used in upb_bram_lookup.v
 *
 * Write width:  36 bits (we don't use the parity bits of the bram module)
 * Read width:  288 bits (enough to store action + tuple)
 * 
 * Please have a look at UG621 and/or UG190 for correct widths of BRAM primitive in- and outputs
 */
 
module cam_ram #(
    parameter C_BRAM_PRIMITIVE_ADDR_WIDTH = 10, // addr width of bram primitive
    parameter CAM_DEPTH = 0, // overwrite this at instanciation
    parameter C_NUM_BRAM_ROWS = CAM_DEPTH/(2**C_BRAM_PRIMITIVE_ADDR_WIDTH)
    ) (
    input clka,
    input clkb,
    input reset,
    
    input [$clog2(CAM_DEPTH)-1+3:0] addra,
    input [$clog2(CAM_DEPTH)-1:0] addrb,
    
    input wea,
    
    
    input[35:0] dina,
    
    output[287:0] doutb
);

logic [C_NUM_BRAM_ROWS-1:0] bram_selector_a;
logic [C_NUM_BRAM_ROWS-1:0] bram_selector_b;
logic [C_NUM_BRAM_ROWS-1:0] bram_selector_b_d;
generate 
    for(genvar i = 0; i<C_NUM_BRAM_ROWS; i++) begin: bram_row
        logic [287:0] doutb;
        logic [287:0] doutb_t;
        for(genvar j = 0; j<8; j++) begin: bram
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
                .READ_WIDTH_A (36),    // Valid values are 1-36 (19-36 only valid when BRAM_SIZE="36Kb")
                .READ_WIDTH_B (36),    // Valid values are 1-36 (19-36 only valid when BRAM_SIZE="36Kb")
                .SIM_COLLISION_CHECK ("ALL"), // Collision check enable "ALL", "WARNING_ONLY",
                                             //"GENERATE_X_ONLY" or "NONE"
                .SIM_MODE("SAFE"), // Simulation: "SAFE" vs. "FAST", see "Synthesis and Simulation Design Guide" for details
                .SRVAL_A(36'h00000000), // Set/Reset value for port A output
                .SRVAL_B(36'h00000000), // Set/Reset value for port B output
                .WRITE_MODE_A("READ_FIRST"), // "WRITE_FIRST", "READ_FIRST", or "NO_CHANGE"
                .WRITE_MODE_B("READ_FIRST"), // "WRITE_FIRST", "READ_FIRST", or "NO_CHANGE"
                .WRITE_WIDTH_A(36), // Valid values are 1-36 (19-36 only valid when BRAM_SIZE="36Kb")
                .WRITE_WIDTH_B(36) // Valid values are 1-36 (19-36 only valid when BRAM_SIZE="36Kb")
                
                ) BRAM_TDP_MACRO_inst (
                .DOA(),    // Output port-A data
                .DOB(doutb_t[36*(j+1)-1-:36]),    // Output port-B data
                .ADDRA(addra[C_BRAM_PRIMITIVE_ADDR_WIDTH-1+3:3]),    // Input port-A address
                .ADDRB(addrb),    // Input port-B address
                .CLKA(clka),    // Input port-A clock
                .CLKB(clkb),    // Input port-B clock
                .DIA(dina),    // Input port-A data
                .DIB(0),    // Input port-B data
                .ENA(bram_selector_a[i]),    // Input port-A enable
                .ENB(bram_selector_b[i]),    // Input port-B enable
                .REGCEA(0), // Input port-A output register enable
                .REGCEB(0), // Input port-B output register enable
                .RSTA(reset),
                .RSTB(reset),
                .WEA({4{bram_selector_a[i] && (addra[2:0] == j) && wea}}),
                .WEB(0)
            );
        end
    end
    assign bram_row[C_NUM_BRAM_ROWS-1].doutb = (bram_row[C_NUM_BRAM_ROWS-1].doutb_t & {288{bram_selector_b_d[C_NUM_BRAM_ROWS-1]}});
    for(genvar i = C_NUM_BRAM_ROWS-2; i>=0; i--) begin: chaining
        assign bram_row[i].doutb = bram_row[i+1].doutb | (bram_row[i].doutb_t & {288{bram_selector_b_d[i]}});
    end
    assign doutb = bram_row[0].doutb;

endgenerate

always_comb begin
    bram_selector_a = 32'b00000000000000000000000000000001 << addra[$clog2(CAM_DEPTH)-1+3:C_BRAM_PRIMITIVE_ADDR_WIDTH+3];
    bram_selector_b = 32'b00000000000000000000000000000001 << addrb[$clog2(CAM_DEPTH)-1:C_BRAM_PRIMITIVE_ADDR_WIDTH];
end

always_ff @(posedge clka) begin
    bram_selector_b_d <= bram_selector_b;
end
endmodule
