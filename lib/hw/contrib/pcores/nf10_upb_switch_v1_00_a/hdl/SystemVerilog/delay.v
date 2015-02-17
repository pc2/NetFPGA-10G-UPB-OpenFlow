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
 
module delay
#(
    parameter C_DATA_WIDTH=310,
    parameter C_DELAY_CYCLES=5
)
(
    input clk,
    input reset,
    input en,
    input [C_DATA_WIDTH-1:0] data_in,
    output [C_DATA_WIDTH-1:0] data_out
);
    function integer log2;
        input integer number;
        begin
            log2=0;
            while(2**log2<number) begin
                log2=log2+1;
            end
        end
    endfunction // log2
    reg [log2(C_DELAY_CYCLES)-1:0] addr = 0;
    ram data (
        .clk(clk),
        .doutA(data_out),
        .dinA(data_in),
        .enA(en),
        .wr_enA(en),
        .addrA(addr)
    );
    always @(posedge clk) begin
        if(en) begin
            addr <= addr + 1;
            if (addr>=C_DELAY_CYCLES) addr <= 0;
        end
    end
endmodule
