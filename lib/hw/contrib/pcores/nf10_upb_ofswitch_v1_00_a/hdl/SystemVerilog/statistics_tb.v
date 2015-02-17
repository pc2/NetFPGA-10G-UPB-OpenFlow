`timescale 1ns / 1ps
module statistics_tb;
    reg clk=0;
    reg we=0;
    reg[15:0] addr=1024;
    reg[15:0] raddr=1024;
    wire[31:0] rdata;
    statistics uut(
        .clk(clk),
        .addr(addr),
        .we(we),
        .din(1),
        .reset(0),
        .rclk(clk),
        .raddr(raddr),
        .rdata(rdata)
    );
    
    always @(posedge clk) begin
        we <= ~we;
    end
    initial begin
        #1000
        addr = 42;
        #400
        @(posedge clk) raddr = 42;
    end
    always
            #5 clk = ~clk;
endmodule
