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
 * Read and store statistics per flow here.
 */
module statistics #( 
    parameter C_MODE = 0 // 0 = increment by din, 1 = write din
    ) (
    input clk,
    input reset,
    
    input [C_MATCH_ADDR_WIDTH-1:0] addr,
    input we,
    input [C_COUNTER_WIDTH-1:0] din,
    input rclk,
    input [C_MATCH_ADDR_WIDTH-1:0] raddr,
    output [C_COUNTER_WIDTH-1:0] rdata
);

logic [C_MATCH_ADDR_WIDTH-1:0] addra;
logic [C_MATCH_ADDR_WIDTH-1:0] addra_d;
logic [C_COUNTER_WIDTH-1:0] dina;
logic [C_COUNTER_WIDTH-1:0] douta;
logic wea = 0;

typedef enum logic [2:0] {
    IDLE, WRITE
} state_t;
state_t state = IDLE;
state_t next_state = IDLE;

always_comb begin
    if(C_MODE == 0) begin
        dina = douta + din;
        case(state)
            IDLE: begin
                if(we) begin
                    addra = addr;
                    next_state = WRITE;
                end
                else begin
                    addra = 0;
                    next_state = IDLE;
                end
                wea = 0;
            end
            WRITE: begin
                wea = 1;
                addra = addra_d;
                next_state = IDLE;
            end
        endcase
    end
    else begin
        dina = din;
        addra = addr;
        wea = we;
    end
end

always_ff @(posedge clk) begin
    state <= next_state;
    addra_d <= addra;
end
        

statistics_ram ram (
  .clka(clk),
  .reset(reset),
  .wea(wea),
  .addra(addra),
  .dina(dina), 
  .douta(douta),
  .clkb(rclk),
  .web(0),
  .addrb(raddr),
  .doutb(rdata)
);
endmodule
