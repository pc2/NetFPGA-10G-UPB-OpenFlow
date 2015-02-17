`timescale 1ns / 1ps
/*
 * Copyright (c) 2014, 2015 Thomas Löcke
 * tloecke@mail.uni-paderborn.de
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
 * 
 */
module wide_fifo #(
    parameter C_SIM_MODE = "FAST",
    parameter C_ALMOST_FULL_OFFSET = 9'hA,
    parameter C_ALMOST_EMPTY_OFFSET = 9'hA,
    parameter C_DO_REG = 1,
    parameter C_EN_ECC_READ = "FALSE",
    parameter C_EN_ECC_WRITE = "FALSE",
    parameter C_EN_SYN = "FALSE",
    parameter C_FIRST_WORD_FALL_THROUGH = "TRUE",
    parameter C_NUMBER_FIFOS = 4
)
(
    output wire ALMOSTEMPTY,
    output wire ALMOSTFULL,
    output wire [64*C_NUMBER_FIFOS-1:0] DO,
    output wire [8*C_NUMBER_FIFOS-1:0] DOP,
    output wire EMPTY,
    output wire FULL,
    output wire RDERR,
    output wire WRERR,
    input wire [64*C_NUMBER_FIFOS-1:0] DI,
    input wire [8*C_NUMBER_FIFOS-1:0] DIP,
    input wire RDCLK,
    input wire RDEN,
    input wire RST,
    input wire WRCLK,
    input wire WREN
    );
  
  wire [C_NUMBER_FIFOS-1:0] wALMOSTEMPTY;
  wire [C_NUMBER_FIFOS-1:0] wALMOSTFULL;
  wire [C_NUMBER_FIFOS-1:0] wEMPTY;
  wire [C_NUMBER_FIFOS-1:0] wFULL;
  wire [C_NUMBER_FIFOS-1:0] wRDERR;
  wire [C_NUMBER_FIFOS-1:0] wWRERR;
  
  assign ALMOSTEMPTY = | wALMOSTEMPTY;
  assign ALMOSTFULL = | wALMOSTFULL;
  assign EMPTY = | wEMPTY;
  assign FULL = | wFULL;
  assign RDERR = | wRDERR;
  assign WRERR = | wWRERR;
  
  genvar i;
  generate
    for (i = 0; i < C_NUMBER_FIFOS; i = i + 1) begin : fifos
      FIFO36_72 #(
        .SIM_MODE(C_SIM_MODE),
        .ALMOST_FULL_OFFSET(C_ALMOST_FULL_OFFSET), 
        .ALMOST_EMPTY_OFFSET(C_ALMOST_EMPTY_OFFSET),
        .DO_REG(C_DO_REG),
        .EN_ECC_READ(C_EN_ECC_READ),
        .EN_ECC_WRITE(C_EN_ECC_WRITE),
        .EN_SYN(C_EN_SYN),
        .FIRST_WORD_FALL_THROUGH(C_FIRST_WORD_FALL_THROUGH)
        ) fifo (
        .ALMOSTEMPTY(wALMOSTEMPTY[i]),
        .ALMOSTFULL(wALMOSTFULL[i]),
        .DO(DO[(i+1)*64-1-:64]),
        .DOP(DOP[(i+1)*8-1-:8]),
        .EMPTY(wEMPTY[i]),
        .FULL(wFULL[i]),
        .RDERR(wRDERR[i]),
        .SBITERR(),
        .WRERR(wWRERR[i]),
        .DI(DI[(i+1)*64-1-:64]),
        .DIP(DIP[(i+1)*8-1-:8]),
        .RDCLK(RDCLK),
        .RDEN(RDEN),
        .RST(RST),
        .WRCLK(WRCLK),
        .WREN(WREN)
      );
    end
  endgenerate
endmodule
