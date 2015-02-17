//-----------------------------------------------------------------
//
// The original verilog code provided by Clifford E. Cummings,
// Sunburst Design, Inc.
//
// Email: <cliffc@sunburst-design.com>
// Web:   www.sunburst-design.com
//
// Minor modifications by Mario Flajslik (mariof@stanford.edu)
// February 2012
//----------------------------------------------------------------

module small_async_fifo
  #(
    parameter DSIZE = 8,
    parameter ASIZE = 3,
    parameter ALMOST_FULL_SIZE = 5,
    parameter ALMOST_EMPTY_SIZE = 3
    )

   (
    //wr interface
    output             wfull,
    output             w_almost_full,
    input [DSIZE-1:0]  wdata,
    input              winc, wclk, wrst_n,

    //rd interface
    output [DSIZE-1:0] rdata,
    output             rempty,
    output             r_almost_empty,
    input              rinc, rclk, rrst_n
    );

   wire [ASIZE-1:0]    waddr, raddr;
   wire [ASIZE:0]      wptr, rptr, wq2_rptr, rq2_wptr;

   sync_r2w #(ASIZE) sync_r2w (.wq2_rptr(wq2_rptr), .rptr(rptr),
		                       .wclk(wclk), .wrst_n(wrst_n));

   sync_w2r #(ASIZE) sync_w2r (.rq2_wptr(rq2_wptr), .wptr(wptr),
		                       .rclk(rclk), .rrst_n(rrst_n));

   fifo_mem #(DSIZE, ASIZE) fifo_mem
     (.rdata(rdata), .wdata(wdata),
      .waddr(waddr), .raddr(raddr),
      .wclken(winc), .wfull(wfull),
      .rden(rinc),
      .wclk(wclk),   .rclk(rclk));

   rptr_empty #(.ADDRSIZE(ASIZE), .ALMOST_EMPTY_SIZE(ALMOST_EMPTY_SIZE))
   rptr_empty
     (.rempty(rempty),
      .r_almost_empty(r_almost_empty),
      .raddr(raddr),
      .rptr(rptr),
      .rq2_wptr(rq2_wptr),
      .rinc(rinc),
      .rclk(rclk),
      .rrst_n(rrst_n));

   wptr_full #(.ADDRSIZE(ASIZE), .ALMOST_FULL_SIZE(ALMOST_FULL_SIZE))
   wptr_full
     (.wfull(wfull),
      .w_almost_full(w_almost_full),
      .waddr(waddr),
      .wptr(wptr),
      .wq2_rptr(wq2_rptr),
      .winc(winc),
      .wclk(wclk),
      .wrst_n(wrst_n));

endmodule // small_async_fifo


module sync_r2w #(parameter ADDRSIZE = 3)
   (output reg [ADDRSIZE:0] wq2_rptr,
    input [ADDRSIZE:0] rptr,
    input              wclk, wrst_n);

   reg [ADDRSIZE:0]    wq1_rptr;

   always @(posedge wclk or negedge wrst_n)

     if (!wrst_n) {wq2_rptr,wq1_rptr} <= 0;

     else {wq2_rptr,wq1_rptr} <= {wq1_rptr,rptr};

endmodule // sync_r2w


module sync_w2r #(parameter ADDRSIZE = 3)
   (output reg [ADDRSIZE:0] rq2_wptr,
    input [ADDRSIZE:0] wptr,
    input              rclk, rrst_n);

   reg [ADDRSIZE:0]    rq1_wptr;

   always @(posedge rclk or negedge rrst_n)

     if (!rrst_n) {rq2_wptr,rq1_wptr} <= 0;

     else {rq2_wptr,rq1_wptr} <= {rq1_wptr,wptr};

endmodule // sync_w2r


module rptr_empty
  #(parameter ADDRSIZE = 3,
    parameter ALMOST_EMPTY_SIZE=3)
   (output reg rempty,
    output reg               r_almost_empty,
    output [ADDRSIZE-1:0]    raddr,
    output reg [ADDRSIZE :0] rptr,
    input [ADDRSIZE :0]      rq2_wptr,
    input                    rinc, rclk, rrst_n);

   reg [ADDRSIZE:0]          rbin;
   wire [ADDRSIZE:0]         rgraynext, rbinnext;
   reg [ADDRSIZE :0]         rq2_wptr_bin;
   integer                   i;

   //------------------
   // GRAYSTYLE2 pointer
   //------------------
   always @(posedge rclk or negedge rrst_n)

     if (!rrst_n) {rbin, rptr} <= 0;
     else {rbin, rptr} <= {rbinnext, rgraynext};

   // Memory read-address pointer (okay to use binary to address memory)
   assign 	     raddr = rbin[ADDRSIZE-1:0];

   assign 	     rbinnext = (rinc & ~rempty) ? rbin + 1 : rbin;
   assign 	     rgraynext = (rbinnext>>1) ^ rbinnext;

   //--------------------------------------------------------------
   // FIFO empty when the next rptr == synchronized wptr or on reset
   //--------------------------------------------------------------
   wire                      rempty_val = (rgraynext == rq2_wptr);

   // Gray code to Binary code conversion
   always @(rq2_wptr)
     for (i=0; i<(ADDRSIZE+1); i=i+1)
       rq2_wptr_bin[i] = ^ (rq2_wptr >> i);

   wire [ADDRSIZE:0]         subtract = (rbinnext + ALMOST_EMPTY_SIZE)-rq2_wptr_bin;

   wire                      r_almost_empty_val = ~subtract[ADDRSIZE];

   always @(posedge rclk or negedge rrst_n)
     if (!rrst_n) begin
	    rempty <= 1'b1;
	    r_almost_empty <= 1'b 1;
     end
     else begin
	    rempty <= rempty_val;
	    r_almost_empty <= r_almost_empty_val;
     end

endmodule // rptr_empty


module wptr_full
  #(parameter ADDRSIZE = 3,
    parameter ALMOST_FULL_SIZE=5
    )
   (output reg wfull,
    output reg               w_almost_full,
    output [ADDRSIZE-1:0]    waddr,
    output reg [ADDRSIZE :0] wptr,
    input [ADDRSIZE :0]      wq2_rptr,
    input                    winc, wclk, wrst_n);

   reg [ADDRSIZE:0]          wbin;
   wire [ADDRSIZE:0]         wgraynext, wbinnext;
   reg [ADDRSIZE :0]         wq2_rptr_bin;
   integer                   i;

   // GRAYSTYLE2 pointer

   always @(posedge wclk or negedge wrst_n)
     if (!wrst_n) {wbin, wptr} <= 0;
     else {wbin, wptr} <= {wbinnext, wgraynext};

   // Memory write-address pointer (okay to use binary to address memory)
   assign waddr = wbin[ADDRSIZE-1:0];

   assign wbinnext = (winc & ~wfull) ? wbin + 1 : wbin;
   assign wgraynext = (wbinnext>>1) ^ wbinnext;


   //-----------------------------------------------------------------
   // Simplified version of the three necessary full-tests:
   // assign wfull_val=((wgnext[ADDRSIZE] !=wq2_rptr[ADDRSIZE] ) &&
   // (wgnext[ADDRSIZE-1] !=wq2_rptr[ADDRSIZE-1]) &&
   // (wgnext[ADDRSIZE-2:0]==wq2_rptr[ADDRSIZE-2:0]));
   //-----------------------------------------------------------------
   wire                      wfull_val = (wgraynext ==
		                                  {~wq2_rptr[ADDRSIZE:ADDRSIZE-1],wq2_rptr[ADDRSIZE-2:0]});

   // Gray code to Binary code conversion
   always @(wq2_rptr)
     for (i=0; i<(ADDRSIZE+1); i=i+1)
       wq2_rptr_bin[i] = ^ (wq2_rptr >> i);

   wire [ADDRSIZE :0]        subtract = wbinnext - wq2_rptr_bin - ALMOST_FULL_SIZE[ADDRSIZE:0];

   wire                      w_almost_full_val = ~subtract[ADDRSIZE];

   always @(posedge wclk or negedge wrst_n)
     if (!wrst_n) begin
	    wfull <= 1'b0;
	    w_almost_full <= 1'b 0;
     end
     else begin
	    wfull <= wfull_val;
	    w_almost_full <= w_almost_full_val;
     end

endmodule // wptr_full


module fifo_mem #(parameter DATASIZE = 8, // Memory data word width
		          parameter ADDRSIZE = 3) // Number of mem address bits
   (output logic [DATASIZE-1:0] rdata,
    input logic [DATASIZE-1:0] wdata,
    input logic [ADDRSIZE-1:0] waddr, raddr,
    input logic                rden,
    input logic                wclken, wfull, wclk, rclk);

   // RTL Verilog memory model
   localparam DEPTH = 1<<ADDRSIZE;

   generate
      if(DEPTH > 32)
        fifo_bram #(.DATASIZE(DATASIZE), .ADDRSIZE(ADDRSIZE)) u_fifo_bram(.*);     
      else if(DEPTH > 4)
        fifo_dram #(.DATASIZE(DATASIZE), .ADDRSIZE(ADDRSIZE)) u_fifo_dram(.*);     
      else
        fifo_rram #(.DATASIZE(DATASIZE), .ADDRSIZE(ADDRSIZE)) u_fifo_rram(.*);           
   endgenerate
   
endmodule // fifo_mem

module fifo_bram #(parameter DATASIZE = 8, // Memory data word width
		          parameter ADDRSIZE = 3) // Number of mem address bits
   (output logic [DATASIZE-1:0] rdata,
    input logic [DATASIZE-1:0] wdata,
    input logic [ADDRSIZE-1:0] waddr, raddr,
    input logic                rden,
    input logic                wclken, wfull, wclk, rclk);
   
   // RTL Verilog memory model
   localparam DEPTH = 1<<ADDRSIZE;

   reg [DATASIZE-1:0]    mem [0:DEPTH-1];  /* synthesis syn_ramstyle="block_ram" */

   //assign rdata = mem[raddr];
   always_ff @(posedge rclk)
     if(rden)
       rdata <= mem[raddr];
     else
       rdata <= rdata;
   
   always_ff @(posedge wclk)
     if (wclken && !wfull) mem[waddr] <= wdata;

endmodule // fifo_mem

module fifo_dram #(parameter DATASIZE = 8, // Memory data word width
		          parameter ADDRSIZE = 3) // Number of mem address bits
   (output logic [DATASIZE-1:0] rdata,
    input logic [DATASIZE-1:0] wdata,
    input logic [ADDRSIZE-1:0] waddr, raddr,
    input logic                rden,
    input logic                wclken, wfull, wclk, rclk);
   
   // RTL Verilog memory model
   localparam DEPTH = 1<<ADDRSIZE;

   reg [DATASIZE-1:0]    mem [0:DEPTH-1];  /* synthesis syn_ramstyle="select_ram" */

   //assign rdata = mem[raddr];
   always_ff @(posedge rclk)
     if(rden)
       rdata <= mem[raddr];
     else
       rdata <= rdata;
   
   always_ff @(posedge wclk)
     if (wclken && !wfull) mem[waddr] <= wdata;

endmodule // fifo_mem

module fifo_rram #(parameter DATASIZE = 8, // Memory data word width
		          parameter ADDRSIZE = 3) // Number of mem address bits
   (output logic [DATASIZE-1:0] rdata,
    input logic [DATASIZE-1:0] wdata,
    input logic [ADDRSIZE-1:0] waddr, raddr,
    input logic                rden,
    input logic                wclken, wfull, wclk, rclk);
   
   // RTL Verilog memory model
   localparam DEPTH = 1<<ADDRSIZE;

   reg [DATASIZE-1:0]    mem [0:DEPTH-1];  /* synthesis syn_ramstyle="registers" */

   //assign rdata = mem[raddr];
   always_ff @(posedge rclk)
     if(rden)
       rdata <= mem[raddr];
     else
       rdata <= rdata;
   
   always_ff @(posedge wclk)
     if (wclken && !wfull) mem[waddr] <= wdata;

endmodule // fifo_mem
