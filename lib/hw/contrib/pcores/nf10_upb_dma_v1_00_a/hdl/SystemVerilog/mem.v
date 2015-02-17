/*******************************************************************************
 *
 *  NetFPGA-10G http://www.netfpga.org
 *
 *  File:
 *        mem.v
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
 *  Description:
 *        Custom memory module that can include multiple read interfaces, 
 *        valid bits and byte masks depending on the parameters.
 *
 *  Copyright notice:
 *        Copyright (C) 2010, 2011 The Board of Trustees of The Leland Stanford
 *                                 Junior University
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

`include "dma_defs.vh"

module mem #(
             // depth in number of lines
             parameter DEPTH=64, 
             // width of each line, write to last byte of the line triggers valid bit
             parameter WIDTH=64,
             // mode=0 no valid bits; 
             // mode=1 valid bit with read interface; 
             // mode=2 random valid bit interface
             // mode=3 same as 2 with two separate read interfaces for valid bit
             parameter VALID_MODE=0,
             parameter HAS_WR_MASK=1)
   (
    // memory interface signals valid
    input logic                       wr_mem_valid,
    input logic                       rd_mem_valid,
    
    // memory write interface
    input logic [`MEM_ADDR_BITS-1:0]  wr_addr_hi,
    input logic [31:0]                wr_data_hi,
    input logic [3:0]                 wr_mask_hi,
    input logic                       wr_en_hi,
    input logic [`MEM_ADDR_BITS-1:0]  wr_addr_lo,
    input logic [31:0]                wr_data_lo,
    input logic [3:0]                 wr_mask_lo,
    input logic                       wr_en_lo,

    // memory read interface
    input logic [`MEM_ADDR_BITS-1:0]  rd_addr_hi,
    output logic [31:0]               rd_data_hi,
    input logic                       rd_en_hi,
    input logic [`MEM_ADDR_BITS-1:0]  rd_addr_lo,
    output logic [31:0]               rd_data_lo,
    output logic                      rd_vld_lo,
    input logic                       rd_en_lo,

    // valid write interface (asynchronous)
    input logic [`MEM_ADDR_BITS-12:0] valid_wr_addr,
    input logic [31:0]                valid_wr_mask,
    input logic                       valid_wr_clear,
    output logic                      valid_wr_stall,

    // valid read interface
    input logic [`MEM_ADDR_BITS-12:0] valid_rd_addr,
    output logic [31:0]               valid_rd_bits,
    input logic [`MEM_ADDR_BITS-12:0] valid_rd_addr_x,
    output logic [31:0]               valid_rd_bits_x,

    // misc
    input logic                       wr_clk,
    input logic                       rd_clk,
    input logic                       valid_rd_clk,
    input logic                       valid_wr_clk,
    input logic                       rst
    );

   localparam LAST_BYTE = WIDTH - 1;
   localparam ADDR_BITS = (DEPTH<=32) ? 1 : $clog2((DEPTH+31)/32);

   //-------------------------------------------------
   // *** Cross clock domain fifo for valid writes ***
   //-------------------------------------------------
   logic [`MEM_ADDR_BITS-12:0]        valid_wr_addr_reg;
   logic [31:0]                       valid_wr_mask_reg;
   logic                              valid_wr_empty;
   logic                              valid_wr_deq;                              

   generate
      if(VALID_MODE != 0) begin
         fifo #(.WIDTH(`MEM_ADDR_BITS-11+32), .DEPTH(`MEM_VALID_X_DEPTH), .ALMOST_FULL(0))
         u_fifo_x(
                  .enq_en(valid_wr_clear),
                  .enq_data({valid_wr_addr, valid_wr_mask}),
                  .deq_en(valid_wr_deq),
                  .deq_data({valid_wr_addr_reg, valid_wr_mask_reg}),
                  .empty(valid_wr_empty),
                  .almost_full(),
                  .full(valid_wr_stall),
                  .enq_clk(valid_wr_clk),
                  .deq_clk(wr_clk),
                  .rst(rst));
      end
   endgenerate
   
   //----------------------------------
   // *** Return data only if valid *** 
   //----------------------------------
   logic [`MEM_ADDR_BITS-1:0]        rd_addr_hi_d1;
   logic [`MEM_ADDR_BITS-1:0]        rd_addr_lo_d1;
   logic [31:0]                      rd_vld_bits;

   generate
      if(VALID_MODE == 1)
        assign rd_vld_lo = rd_vld_bits[rd_addr_lo_d1[10:6]];
      else
        assign rd_vld_lo = 0;
   endgenerate

   always_ff @(posedge rd_clk) begin
      if(rd_en_lo)
        rd_addr_lo_d1 <= rd_addr_lo;
      if(rd_en_hi)
        rd_addr_hi_d1 <= rd_addr_hi;
   end
   
   //--------------------------
   // *** Handle valid bits *** 
   //--------------------------
   
   //--------------------------
   // pipeline stage READ
   //--------------------------
   logic                             vld_wrrd_en;
   logic [ADDR_BITS-1:0]             vld_wrrd_addr;
   logic [31:0]                      vld_wrrd_data;

   logic                             vld_ppl_set,   vld_ppl_set_nxt;
   logic                             vld_ppl_clear, vld_ppl_clear_nxt;
   logic [ADDR_BITS-1:0]             vld_ppl_addr,  vld_ppl_addr_nxt;
   logic [31:0]                      vld_ppl_mask,  vld_ppl_mask_nxt;

   generate
      if(VALID_MODE != 0) begin
         always_comb begin
            vld_wrrd_en    = 0;
            vld_wrrd_addr  = valid_wr_addr_reg[ADDR_BITS-1:0];

            vld_ppl_set_nxt   = 0;
            vld_ppl_clear_nxt = 0;
            vld_ppl_mask_nxt  = valid_wr_mask_reg;
            vld_ppl_addr_nxt  = valid_wr_addr_reg[ADDR_BITS-1:0];

            valid_wr_deq = 0;
            
            // Absolute priority to writes coming in because they have to be granted
            if(LAST_BYTE[2] && wr_en_hi && (wr_addr_hi[5:3] == LAST_BYTE[5:3]) && wr_mask_hi[LAST_BYTE[1:0]] && wr_mem_valid) begin
               vld_ppl_mask_nxt = 1 << wr_addr_hi[10:6];
               vld_ppl_addr_nxt = wr_addr_hi[11+:ADDR_BITS];
               vld_ppl_set_nxt = 1;
               vld_wrrd_en = 1;
               vld_wrrd_addr = wr_addr_hi[11+:ADDR_BITS];
            end
            else if(!LAST_BYTE[2] && wr_en_lo && (wr_addr_lo[5:3] == LAST_BYTE[5:3]) && wr_mask_lo[LAST_BYTE[1:0]] && wr_mem_valid) begin
               vld_ppl_mask_nxt = 1 << wr_addr_lo[10:6];
               vld_ppl_addr_nxt = wr_addr_lo[11+:ADDR_BITS];
               vld_ppl_set_nxt = 1;
               vld_wrrd_en = 1;
               vld_wrrd_addr = wr_addr_lo[11+:ADDR_BITS];
            end
            else if(~valid_wr_empty) begin
               vld_wrrd_en       = 1;
               vld_ppl_clear_nxt = 1;
               valid_wr_deq      = 1;
            end

         end
         always_ff @(posedge wr_clk) begin
            if(rst) begin
               vld_ppl_set   <= 0;
               vld_ppl_clear <= 0;
               vld_ppl_mask  <= 0;
               vld_ppl_addr  <= 0;
            end
            else begin
               vld_ppl_set   <= vld_ppl_set_nxt;
               vld_ppl_clear <= vld_ppl_clear_nxt;
               vld_ppl_mask  <= vld_ppl_mask_nxt;
               vld_ppl_addr  <= vld_ppl_addr_nxt;
            end
         end
      end
   endgenerate           

   //--------------------------
   // pipeline stage WRITE
   //--------------------------
   logic                             vld_wr_en;
   logic [ADDR_BITS-1:0]             vld_wr_addr;
   logic [31:0]                      vld_wr_data;

   logic [ADDR_BITS-1:0]             vld_rst_wr_addr, vld_rst_wr_addr_nxt;
   logic                             vld_rst_done, vld_rst_done_nxt;

   // need to cache one entry in case we read and write same address in the same cycle
   logic [ADDR_BITS-1:0]             vld_wr_addr_reg, vld_wr_addr_reg_nxt;
   logic [31:0]                      vld_wr_data_reg, vld_wr_data_reg_nxt;

   generate
      if(VALID_MODE != 0) begin
         always_comb begin
            vld_wr_en = 0;
            vld_wr_addr = vld_ppl_addr;
            vld_wr_data = 0;

            vld_rst_done_nxt = vld_rst_done;
            vld_rst_wr_addr_nxt = vld_rst_wr_addr;

            vld_wr_addr_reg_nxt = vld_wr_addr_reg;
            vld_wr_data_reg_nxt = vld_wr_data_reg;
            
            // reset the memories
            if(~vld_rst_done) begin
               vld_rst_wr_addr_nxt = vld_rst_wr_addr + 1;
               vld_wr_addr = vld_rst_wr_addr;
               vld_wr_data = 32'b0;
               vld_wr_en = 1;
               if(integer'(vld_rst_wr_addr) == ((DEPTH+31)/32)-1)
                 vld_rst_done_nxt = 1;
            end
            else begin
               if(vld_ppl_set) begin
                  if(vld_wr_addr_reg != vld_wr_addr) begin
                     vld_wr_data = vld_wrrd_data | vld_ppl_mask;
                     vld_wr_en = 1;
                     vld_wr_addr_reg_nxt = vld_wr_addr;
                     vld_wr_data_reg_nxt = vld_wr_data;
                  end
                  else begin
                     vld_wr_data = vld_wr_data_reg | vld_ppl_mask;
                     vld_wr_en = 1;
                     vld_wr_data_reg_nxt = vld_wr_data;
                  end
               end
               else if(vld_ppl_clear) begin
                  if(vld_wr_addr_reg != vld_wr_addr) begin
                     vld_wr_data = vld_wrrd_data & ~vld_ppl_mask;
                     vld_wr_en = 1;
                     vld_wr_addr_reg_nxt = vld_wr_addr;
                     vld_wr_data_reg_nxt = vld_wr_data;
                  end
                  else begin
                     vld_wr_data = vld_wr_data_reg & ~vld_ppl_mask;
                     vld_wr_en = 1;
                     vld_wr_data_reg_nxt = vld_wr_data;
                  end
               end
            end
            
         end
         always_ff @(posedge wr_clk) begin
            if(rst) begin
               vld_rst_wr_addr <= 0;
               vld_rst_done    <= 0;
               vld_wr_addr_reg <= 0;
               vld_wr_data_reg <= 0;
            end
            else begin
               vld_rst_done    <= vld_rst_done_nxt;
               vld_rst_wr_addr <= vld_rst_wr_addr_nxt;
               vld_wr_addr_reg <= vld_wr_addr_reg_nxt;
               vld_wr_data_reg <= vld_wr_data_reg_nxt;
            end
         end
      end
   endgenerate
   
   // ----------------------------------------
   // *** VALID MEMORY ***
   // ----------------------------------------
   generate
      if(VALID_MODE == 2 || VALID_MODE == 3) begin
        ram #(.WIDTH(32), .DEPTH((DEPTH+31)/32)) u_ram_vld_1
          (.wr_en(vld_wr_en),
           .wr_addr(vld_wr_addr),
           .wr_data(vld_wr_data),
           .rd_en(1'b1),
           .rd_addr(valid_rd_addr[ADDR_BITS-1:0]),
           .rd_data(valid_rd_bits),
           .wr_clk(wr_clk),
           .rd_clk(valid_rd_clk));
      end
      if(VALID_MODE == 3) begin
        ram #(.WIDTH(32), .DEPTH((DEPTH+31)/32)) u_ram_vld_1x
          (.wr_en(vld_wr_en),
           .wr_addr(vld_wr_addr),
           .wr_data(vld_wr_data),
           .rd_en(1'b1),
           .rd_addr(valid_rd_addr_x[ADDR_BITS-1:0]),
           .rd_data(valid_rd_bits_x),
           .wr_clk(wr_clk),
           .rd_clk(valid_rd_clk));
      end
      else begin 
         assign valid_rd_bits_x = 32'b0;
      end
      
      if(VALID_MODE == 1 && LAST_BYTE[2]) begin
         ram #(.WIDTH(32), .DEPTH((DEPTH+31)/32)) u_ram_vld_2
           (.wr_en(vld_wr_en),
            .wr_addr(vld_wr_addr),
            .wr_data(vld_wr_data),
            .rd_en(rd_en_hi & rd_mem_valid),
            .rd_addr(rd_addr_hi[11+:ADDR_BITS]),
            .rd_data(rd_vld_bits),
            .wr_clk(wr_clk),
            .rd_clk(rd_clk));
      end
      else if(VALID_MODE == 1) begin
         ram #(.WIDTH(32), .DEPTH((DEPTH+31)/32)) u_ram_vld_2
           (.wr_en(vld_wr_en),
            .wr_addr(vld_wr_addr),
            .wr_data(vld_wr_data),
            .rd_en(rd_en_lo & rd_mem_valid),
            .rd_addr(rd_addr_lo[11+:ADDR_BITS]),
            .rd_data(rd_vld_bits),
            .wr_clk(wr_clk),
            .rd_clk(rd_clk));
      end

      if(VALID_MODE != 0) begin
         ram #(.WIDTH(32), .DEPTH((DEPTH+31)/32)) u_ram_vld_3
           (.wr_en(vld_wr_en),
            .wr_addr(vld_wr_addr),
            .wr_data(vld_wr_data),
            .rd_en(vld_wrrd_en),
            .rd_addr(vld_wrrd_addr),
            .rd_data(vld_wrrd_data),
            .wr_clk(wr_clk),
            .rd_clk(wr_clk));
      end
   endgenerate
   
   // ---------------------
   // *** DATA MEMORIES ***
   // ---------------------
   genvar                            i;
   generate
      if(HAS_WR_MASK) begin
         for(i = 0; i < 4; i++) begin: low_mems
            if(WIDTH > 32) begin
               ram #(.WIDTH(8), .DEPTH(8*DEPTH)) u_ram_lo
                 (.wr_en(wr_en_lo & wr_mask_lo[i] & wr_mem_valid),
                  .wr_addr(wr_addr_lo[3+:$clog2(8*DEPTH)]),
                  .wr_data(wr_data_lo[8*i+:8]),
                  .rd_en(rd_en_lo & rd_mem_valid),
                  .rd_addr(rd_addr_lo[3+:$clog2(8*DEPTH)]),
                  .rd_data(rd_data_lo[8*i+:8]),
                  .*);            
            end
            else if(WIDTH > 16) begin
               ram #(.WIDTH(8), .DEPTH(4*DEPTH)) u_ram_lo
                 (.wr_en(wr_en_lo & wr_mask_lo[i] & wr_mem_valid & ~wr_addr_lo[5]),
                  .wr_addr({wr_addr_lo[6+:($clog2(4*DEPTH)-2)], wr_addr_lo[3+:2]}),
                  .wr_data(wr_data_lo[8*i+:8]),
                  .rd_en(rd_en_lo & rd_mem_valid),
                  .rd_addr({rd_addr_lo[6+:($clog2(4*DEPTH)-2)], rd_addr_lo[3+:2]}),
                  .rd_data(rd_data_lo[8*i+:8]),
                  .*);
            end
            else if(WIDTH > 8) begin
               ram #(.WIDTH(8), .DEPTH(2*DEPTH)) u_ram_lo
                 (.wr_en(wr_en_lo & wr_mask_lo[i] & wr_mem_valid & ~|wr_addr_lo[5:4]),
                  .wr_addr({wr_addr_lo[6+:($clog2(2*DEPTH)-1)], wr_addr_lo[3+:1]}),
                  .wr_data(wr_data_lo[8*i+:8]),
                  .rd_en(rd_en_lo & rd_mem_valid),
                  .rd_addr({rd_addr_lo[6+:($clog2(2*DEPTH)-1)], rd_addr_lo[3+:1]}),
                  .rd_data(rd_data_lo[8*i+:8]),
                  .*);
            end
            else begin
               ram #(.WIDTH(8), .DEPTH(1*DEPTH)) u_ram_lo
                 (.wr_en(wr_en_lo & wr_mask_lo[i] & wr_mem_valid & ~|wr_addr_lo[5:3]),
                  .wr_addr(wr_addr_lo[6+:$clog2(1*DEPTH)]),
                  .wr_data(wr_data_lo[8*i+:8]),
                  .rd_en(rd_en_lo & rd_mem_valid),
                  .rd_addr(rd_addr_lo[6+:$clog2(1*DEPTH)]),
                  .rd_data(rd_data_lo[8*i+:8]),
                  .*);
            end
         end
         for(i = 0; i < 4; i++) begin: high_mems
            if(WIDTH > 32) begin
               ram #(.WIDTH(8), .DEPTH(8*DEPTH)) u_ram_hi
                 (.wr_en(wr_en_hi & wr_mask_hi[i] & wr_mem_valid),
                  .wr_addr(wr_addr_hi[3+:$clog2(8*DEPTH)]),
                  .wr_data(wr_data_hi[8*i+:8]),
                  .rd_en(rd_en_hi & rd_mem_valid),
                  .rd_addr(rd_addr_hi[3+:$clog2(8*DEPTH)]),
                  .rd_data(rd_data_hi[8*i+:8]),
                  .*);            
            end
            else if(WIDTH > 16) begin
               ram #(.WIDTH(8), .DEPTH(4*DEPTH)) u_ram_hi
                 (.wr_en(wr_en_hi & wr_mask_hi[i] & wr_mem_valid & ~wr_addr_hi[5]),
                  .wr_addr({wr_addr_hi[6+:($clog2(4*DEPTH)-2)], wr_addr_hi[3+:2]}),
                  .wr_data(wr_data_hi[8*i+:8]),
                  .rd_en(rd_en_hi & rd_mem_valid),
                  .rd_addr({rd_addr_hi[6+:($clog2(4*DEPTH)-2)], rd_addr_hi[3+:2]}),
                  .rd_data(rd_data_hi[8*i+:8]),
                  .*);
            end
            else if(WIDTH > 8) begin
               ram #(.WIDTH(8), .DEPTH(2*DEPTH)) u_ram_hi
                 (.wr_en(wr_en_hi & wr_mask_hi[i] & wr_mem_valid & ~|wr_addr_hi[5:4]),
                  .wr_addr({wr_addr_hi[6+:($clog2(2*DEPTH)-1)], wr_addr_hi[3+:1]}),
                  .wr_data(wr_data_hi[8*i+:8]),
                  .rd_en(rd_en_hi & rd_mem_valid),
                  .rd_addr({rd_addr_hi[6+:($clog2(2*DEPTH)-1)], rd_addr_hi[3+:1]}),
                  .rd_data(rd_data_hi[8*i+:8]),
                  .*);
            end
            else begin
               ram #(.WIDTH(8), .DEPTH(1*DEPTH)) u_ram_hi
                 (.wr_en(wr_en_hi & wr_mask_hi[i] & wr_mem_valid & ~|wr_addr_hi[5:3]),
                  .wr_addr(wr_addr_hi[6+:$clog2(1*DEPTH)]),
                  .wr_data(wr_data_hi[8*i+:8]),
                  .rd_en(rd_en_hi & rd_mem_valid),
                  .rd_addr(rd_addr_hi[6+:$clog2(1*DEPTH)]),
                  .rd_data(rd_data_hi[8*i+:8]),
                  .*);
            end
         end
      end
      else begin // no write mask
         
         if(WIDTH > 32) begin
            ram #(.WIDTH(32), .DEPTH(8*DEPTH)) u_ram_lo
              (.wr_en(wr_en_lo & wr_mem_valid),
               .wr_addr(wr_addr_lo[3+:$clog2(8*DEPTH)]),
               .wr_data(wr_data_lo),
               .rd_en(rd_en_lo & rd_mem_valid),
               .rd_addr(rd_addr_lo[3+:$clog2(8*DEPTH)]),
               .rd_data(rd_data_lo),
               .*);            
         end
         else if(WIDTH > 16) begin
            ram #(.WIDTH(32), .DEPTH(4*DEPTH)) u_ram_lo
              (.wr_en(wr_en_lo & wr_mem_valid & ~wr_addr_lo[5]),
               .wr_addr({wr_addr_lo[6+:($clog2(4*DEPTH)-2)], wr_addr_lo[3+:2]}),
               .wr_data(wr_data_lo),
               .rd_en(rd_en_lo & rd_mem_valid),
               .rd_addr({rd_addr_lo[6+:($clog2(4*DEPTH)-2)], rd_addr_lo[3+:2]}),
               .rd_data(rd_data_lo),
               .*);
         end
         else if(WIDTH > 8) begin
            ram #(.WIDTH(32), .DEPTH(2*DEPTH)) u_ram_lo
              (.wr_en(wr_en_lo & wr_mem_valid & ~|wr_addr_lo[5:4]),
               .wr_addr({wr_addr_lo[6+:($clog2(2*DEPTH)-1)], wr_addr_lo[3+:1]}),
               .wr_data(wr_data_lo),
               .rd_en(rd_en_lo & rd_mem_valid),
               .rd_addr({rd_addr_lo[6+:($clog2(2*DEPTH)-1)], rd_addr_lo[3+:1]}),
               .rd_data(rd_data_lo),
               .*);
         end
         else begin
            ram #(.WIDTH(32), .DEPTH(1*DEPTH)) u_ram_lo
              (.wr_en(wr_en_lo & wr_mem_valid & ~|wr_addr_lo[5:3]),
               .wr_addr(wr_addr_lo[6+:$clog2(1*DEPTH)]),
               .wr_data(wr_data_lo),
               .rd_en(rd_en_lo & rd_mem_valid),
               .rd_addr(rd_addr_lo[6+:$clog2(1*DEPTH)]),
               .rd_data(rd_data_lo),
               .*);
         end

         if(WIDTH > 32) begin
            ram #(.WIDTH(32), .DEPTH(8*DEPTH)) u_ram_hi
              (.wr_en(wr_en_hi & wr_mem_valid),
               .wr_addr(wr_addr_hi[3+:$clog2(8*DEPTH)]),
               .wr_data(wr_data_hi),
               .rd_en(rd_en_hi & rd_mem_valid),
               .rd_addr(rd_addr_hi[3+:$clog2(8*DEPTH)]),
               .rd_data(rd_data_hi),
               .*);            
         end
         else if(WIDTH > 16) begin
            ram #(.WIDTH(32), .DEPTH(4*DEPTH)) u_ram_hi
              (.wr_en(wr_en_hi & wr_mem_valid & ~wr_addr_hi[5]),
               .wr_addr({wr_addr_hi[6+:($clog2(4*DEPTH)-2)], wr_addr_hi[3+:2]}),
               .wr_data(wr_data_hi),
               .rd_en(rd_en_hi & rd_mem_valid),
               .rd_addr({rd_addr_hi[6+:($clog2(4*DEPTH)-2)], rd_addr_hi[3+:2]}),
               .rd_data(rd_data_hi),
               .*);
         end
         else if(WIDTH > 8) begin
            ram #(.WIDTH(32), .DEPTH(2*DEPTH)) u_ram_hi
              (.wr_en(wr_en_hi & wr_mem_valid & ~|wr_addr_hi[5:4]),
               .wr_addr({wr_addr_hi[6+:($clog2(2*DEPTH)-1)], wr_addr_hi[3+:1]}),
               .wr_data(wr_data_hi),
               .rd_en(rd_en_hi & rd_mem_valid),
               .rd_addr({rd_addr_hi[6+:($clog2(2*DEPTH)-1)], rd_addr_hi[3+:1]}),
               .rd_data(rd_data_hi),
               .*);
         end
         else begin
            ram #(.WIDTH(32), .DEPTH(1*DEPTH)) u_ram_hi
              (.wr_en(wr_en_hi & wr_mem_valid & ~|wr_addr_hi[5:3]),
               .wr_addr(wr_addr_hi[6+:$clog2(1*DEPTH)]),
               .wr_data(wr_data_hi),
               .rd_en(rd_en_hi & rd_mem_valid),
               .rd_addr(rd_addr_hi[6+:$clog2(1*DEPTH)]),
               .rd_data(rd_data_hi),
               .*);
         end

      end
   endgenerate
   
endmodule