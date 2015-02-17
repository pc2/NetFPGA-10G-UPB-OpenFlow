module ram
#(
    parameter RAM_WIDTH = 310,
    parameter RAM_ADDR_BITS = 3
)
(
    input [RAM_ADDR_BITS-1:0] addrA,
    input [RAM_ADDR_BITS-1:0] addrB,
    input [RAM_WIDTH-1:0] dinA,
    input enA,
    input wr_enA,
    input enB,
    input clk,
    output reg [RAM_WIDTH-1:0] doutA=0, doutB=0
);
    reg [RAM_WIDTH-1:0] data [(2**RAM_ADDR_BITS)-1:0];
    reg [RAM_ADDR_BITS-1:0] k;
    initial begin
        for (k=0; k < (2**RAM_ADDR_BITS)-1; k = k+1) begin
            data[k]=0;
        end
    end
    always @(posedge clk) begin
        if (enA) begin
            if (wr_enA) data[addrA] <= dinA;
            doutA <= data[addrA];
        end
        if (enB)  doutB <= data[addrB];
    end
endmodule
