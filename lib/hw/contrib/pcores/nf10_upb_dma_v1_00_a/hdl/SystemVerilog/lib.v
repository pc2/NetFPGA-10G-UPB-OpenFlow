/*******************************************************************************
 *
 *  NetFPGA-10G http://www.netfpga.org
 *
 *  File:
 *        lib.v
 *
 *  Library:
 *        hw/contrib/pcores/dma_v1_00_a
 *
 *  Module:
 *        dma
 *
 *  Author:
 *        Mario Flajslik
 *
 *        Modifications for the UPB OpenFlow Switch project:
 *         Jörg Niklas, osjsn@niklasfamily.de
 *         Project Group "On-the-Fly Networking for Big Data"
 *         Computer Engineering Group, University of Paderborn
 *
 *  Description:
 *        Library of common used modules such as RAM, FIFO and cross clock
 *        domain synchronization modules.
 *
 *  Copyright notice:
 *        Copyright (C) 2010, 2011 The Board of Trustees of The Leland Stanford
 *                                 Junior University
 *
 *        Modifications for the UPB OpenFlow Switch project:
 *          Copyright (c) 2014, 2015 Jörg Niklas, osjsn@niklasfamily.de
 *
 *  Licence:
 *        This file is part of the NetFPGA 10G development base package.
 *
 *        This file is free code: you can redistribute it and/or modify it under
 *        the terms of the GNU Lesser General Public License version 2.1 as
 *        published by the Free Software Foundation.
 *
 *        This package is distributed in the hope that it will be useful, but
 *        WITHOUT ANY WARRANTY; without even the implied warranty of
 *        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *        Lesser General Public License for more details.
 *
 *        You should have received a copy of the GNU Lesser General Public
 *        License along with the NetFPGA source package.  If not, see
 *        http://www.gnu.org/licenses/.
 *
 */

// RAM module
module ram #(parameter WIDTH=32, parameter DEPTH=32, parameter ADDR_BITS=(DEPTH==1)?1:$clog2(DEPTH))
   (
    input logic                 wr_en,
    input logic [ADDR_BITS-1:0] wr_addr,
    input logic [WIDTH-1:0]     wr_data,

    input logic                 rd_en,
    input logic [ADDR_BITS-1:0] rd_addr,
    output logic [WIDTH-1:0]    rd_data,

    input logic                 wr_clk,
    input logic                 rd_clk
    );

   generate

      //if(DEPTH >= 128)
        //bram #(.WIDTH(WIDTH), .DEPTH(DEPTH)) u_bram(.*);
        aram #(.WIDTH(WIDTH), .DEPTH(DEPTH)) u_aram(.*);        
      //if(WIDTH*DEPTH >= 1024)
        //bram #(.WIDTH(WIDTH), .DEPTH(DEPTH)) u_bram(.*);
      //else if(WIDTH*DEPTH >= 128)
        //dram #(.WIDTH(WIDTH), .DEPTH(DEPTH)) u_dram(.*);
      //else
        //rram #(.WIDTH(WIDTH), .DEPTH(DEPTH)) u_rram(.*);                

   endgenerate

endmodule

// auto RAM module
module aram #(parameter WIDTH=32, parameter DEPTH=32, parameter ADDR_BITS=(DEPTH==1)?1:$clog2(DEPTH))
   (
    input logic                 wr_en,
    input logic [ADDR_BITS-1:0] wr_addr,
    input logic [WIDTH-1:0]     wr_data,

    input logic                 rd_en,
    input logic [ADDR_BITS-1:0] rd_addr,
    output logic [WIDTH-1:0]    rd_data,

    input logic                 wr_clk,
    input logic                 rd_clk
    );

   logic [WIDTH-1:0]                mem[DEPTH-1:0];

   always_ff @(posedge rd_clk) begin
      if(rd_en) rd_data <= mem[rd_addr];
   end
   always_ff @(posedge wr_clk) begin
      if(wr_en) mem[wr_addr] <= wr_data;
   end
   
endmodule

// BRAM module
module bram #(parameter WIDTH=32, parameter DEPTH=32, parameter ADDR_BITS=(DEPTH==1)?1:$clog2(DEPTH))
   (
    input logic                 wr_en,
    input logic [ADDR_BITS-1:0] wr_addr,
    input logic [WIDTH-1:0]     wr_data,

    input logic                 rd_en,
    input logic [ADDR_BITS-1:0] rd_addr,
    output logic [WIDTH-1:0]    rd_data,

    input logic                 wr_clk,
    input logic                 rd_clk
    );

   logic [WIDTH-1:0]                mem[DEPTH-1:0]; /* synthesis syn_ramstyle="block_ram" */

   always_ff @(posedge rd_clk) begin
      if(rd_en) rd_data <= mem[rd_addr];
   end
   always_ff @(posedge wr_clk) begin
      if(wr_en) mem[wr_addr] <= wr_data;
   end
   
endmodule

// distributed RAM module
module dram #(parameter WIDTH=32, parameter DEPTH=32, parameter ADDR_BITS=(DEPTH==1)?1:$clog2(DEPTH))
   (
    input logic                 wr_en,
    input logic [ADDR_BITS-1:0] wr_addr,
    input logic [WIDTH-1:0]     wr_data,

    input logic                 rd_en,
    input logic [ADDR_BITS-1:0] rd_addr,
    output logic [WIDTH-1:0]    rd_data,

    input logic                 wr_clk,
    input logic                 rd_clk
    );

   logic [WIDTH-1:0]                mem[DEPTH-1:0]; /* synthesis syn_ramstyle="select_ram" */

   always_ff @(posedge rd_clk) begin
      if(rd_en) rd_data <= mem[rd_addr];
   end
   always_ff @(posedge wr_clk) begin
      if(wr_en) mem[wr_addr] <= wr_data;
   end
   
endmodule

// registers RAM module
module rram #(parameter WIDTH=32, parameter DEPTH=32, parameter ADDR_BITS=(DEPTH==1)?1:$clog2(DEPTH))
   (
    input logic                 wr_en,
    input logic [ADDR_BITS-1:0] wr_addr,
    input logic [WIDTH-1:0]     wr_data,

    input logic                 rd_en,
    input logic [ADDR_BITS-1:0] rd_addr,
    output logic [WIDTH-1:0]    rd_data,

    input logic                 wr_clk,
    input logic                 rd_clk
    );

   logic [WIDTH-1:0]                mem[DEPTH-1:0]; /* synthesis syn_ramstyle="registers" */

   always_ff @(posedge rd_clk) begin
      if(rd_en) rd_data <= mem[rd_addr];
   end
   always_ff @(posedge wr_clk) begin
      if(wr_en) mem[wr_addr] <= wr_data;
   end
   
endmodule

// FWFT async fifo (design from http://billauer.co.il/reg_fifo.html)
module fifo #(parameter WIDTH=32, parameter DEPTH=32, parameter ALMOST_FULL=3)
  (
   input logic             enq_en,
   input logic[WIDTH-1:0]  enq_data,

   input logic             deq_en,
   output logic[WIDTH-1:0] deq_data,

   output logic            empty,
   output logic            almost_full,
   output logic            full,
   
   input logic             enq_clk,
   input logic             deq_clk,
   input logic             rst
   );

   logic                   fifo_valid, middle_valid, dout_valid;
   logic [(WIDTH-1):0]     dout, middle_dout;
   
   logic [(WIDTH-1):0]     fifo_dout;
   logic                   fifo_empty, fifo_rd_en;
   logic                   will_update_middle, will_update_dout;

   
   small_async_fifo #(.DSIZE(WIDTH), .ASIZE($clog2(DEPTH)), .ALMOST_FULL_SIZE(DEPTH-ALMOST_FULL)) 
   afifo(
         .wfull(full),
         .w_almost_full(almost_full),
         .wdata(enq_data),
         .winc(enq_en),
         .wclk(enq_clk),
         .wrst_n(~rst),
         .rdata(fifo_dout),
         .rempty(fifo_empty),
         .r_almost_empty(),
         .rinc(fifo_rd_en),
         .rclk(deq_clk),
         .rrst_n(~rst)
         );

   assign will_update_middle = fifo_valid && (middle_valid == will_update_dout);
   assign will_update_dout = (middle_valid || fifo_valid) && (deq_en || !dout_valid);
   assign fifo_rd_en = (!fifo_empty) && !(middle_valid && dout_valid && fifo_valid);
   assign empty = !dout_valid;
   assign deq_data = dout;

   always_ff @(posedge deq_clk)
     if(rst)
       begin
          fifo_valid <= 0;
          middle_valid <= 0;
          dout_valid <= 0;
          dout <= 0;
          middle_dout <= 0;
       end
     else
       begin
          if (will_update_middle)
            middle_dout <= fifo_dout;
          
          if (will_update_dout)
            dout <= middle_valid ? middle_dout : fifo_dout;
          
          if (fifo_rd_en)
            fifo_valid <= 1;
          else if (will_update_middle || will_update_dout)
            fifo_valid <= 0;
          
          if (will_update_middle)
            middle_valid <= 1;
          else if (will_update_dout)
            middle_valid <= 0;
          
          if (will_update_dout)
            dout_valid <= 1;
          else if (deq_en)
            dout_valid <= 0;
       end 
endmodule


// Cross clock domain synchronizers
// design from: fpga4fun.com
module x_signal #(parameter WIDTH=1)
   (
    clkA, SignalIn, 
    clkB, SignalOut);

   // clkA domain signals
   input              clkA;
   input [WIDTH-1:0]  SignalIn;
   
   // clkB domain signals
   input              clkB;
   output [WIDTH-1:0] SignalOut;
   
   // Now let's transfer SignalIn into the clkB clock domain
   // We use a two-stages shift-register to synchronize the signal
   reg [WIDTH-1:0]    SyncA_clkB_0;
   reg [WIDTH-1:0]    SyncA_clkB_1;
   always @(posedge clkB) SyncA_clkB_0 <= SignalIn;      // notice that we use clkB
   always @(posedge clkB) SyncA_clkB_1 <= SyncA_clkB_0; // notice that we use clkB

   assign SignalOut = SyncA_clkB_1;  // new signal synchronized to (=ready to be used in) clkB domain
endmodule


module x_flag(
              clkA, FlagIn_clkA, 
              clkB, FlagOut_clkB);

   // clkA domain signals
   input clkA, FlagIn_clkA;

   // clkB domain signals
   input clkB;
   output FlagOut_clkB;

   reg    FlagToggle_clkA;
   reg [2:0] SyncA_clkB;

   // this changes level when a flag is seen
   always @(posedge clkA) if(FlagIn_clkA) FlagToggle_clkA <= ~FlagToggle_clkA;

   // which can then be sync-ed to clkB
   always @(posedge clkB) SyncA_clkB <= {SyncA_clkB[1:0], FlagToggle_clkA};

   // and recreate the flag from the level change
   assign FlagOut_clkB = (SyncA_clkB[2] ^ SyncA_clkB[1]);
endmodule

module x_flag_ack(
                  clkA, FlagIn_clkA, Busy_clkA, 
                  clkB, FlagOut_clkB);

   // clkA domain signals
   input clkA, FlagIn_clkA;
   output Busy_clkA;

   // clkB domain signals
   input  clkB;
   output FlagOut_clkB;

   reg    FlagToggle_clkA;
   reg [2:0] SyncA_clkB;
   reg [1:0] SyncB_clkA;

   always @(posedge clkA) if(FlagIn_clkA & ~Busy_clkA) FlagToggle_clkA <= ~FlagToggle_clkA;
   always @(posedge clkB) SyncA_clkB <= {SyncA_clkB[1:0], FlagToggle_clkA};
   always @(posedge clkA) SyncB_clkA <= {SyncB_clkA[0], SyncA_clkB[2]};

   assign FlagOut_clkB = (SyncA_clkB[2] ^ SyncA_clkB[1]);
   assign Busy_clkA = FlagToggle_clkA ^ SyncB_clkA[1];
endmodule


module x_task(
              clkA, TaskStart_clkA, TaskBusy_clkA, TaskDone_clkA, 
              clkB, TaskStart_clkB, TaskBusy_clkB, TaskDone_clkB);

   // clkA domain signals
   input clkA;
   input TaskStart_clkA;
   output TaskBusy_clkA, TaskDone_clkA;

   // clkB domain signals
   input  clkB;
   output TaskBusy_clkB, TaskStart_clkB;
   input  TaskDone_clkB;

   reg    FlagToggle_clkA, FlagToggle_clkB, Busyhold_clkB;
   reg [2:0] SyncA_clkB, SyncB_clkA;

   always @(posedge clkA) if(TaskStart_clkA & ~TaskBusy_clkA) FlagToggle_clkA <= ~FlagToggle_clkA;

   always @(posedge clkB) SyncA_clkB <= {SyncA_clkB[1:0], FlagToggle_clkA};
   assign TaskStart_clkB = (SyncA_clkB[2] ^ SyncA_clkB[1]);
   assign TaskBusy_clkB = TaskStart_clkB | Busyhold_clkB;
   always @(posedge clkB) Busyhold_clkB <= ~TaskDone_clkB & TaskBusy_clkB;
   always @(posedge clkB) if(TaskBusy_clkB & TaskDone_clkB) FlagToggle_clkB <= FlagToggle_clkA;

   always @(posedge clkA) SyncB_clkA <= {SyncB_clkA[1:0], FlagToggle_clkB};
   assign TaskBusy_clkA = FlagToggle_clkA ^ SyncB_clkA[2];
   assign TaskDone_clkA = SyncB_clkA[2] ^ SyncB_clkA[1];
endmodule
