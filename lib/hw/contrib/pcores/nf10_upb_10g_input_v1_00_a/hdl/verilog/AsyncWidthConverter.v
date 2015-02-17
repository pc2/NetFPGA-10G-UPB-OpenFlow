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
module AsyncWidthConverter(
    // MAC side //
    input clk156,

    // Slave Stream Ports
    input [63:0] s_axis_tdata,
    input [7:0] s_axis_tkeep,
    input [0:0] s_axis_tuser,
    input s_axis_tvalid,
    output reg s_axis_tready = 1, // always able to receive
    input s_axis_tlast,
    
    
    // packet ctrl / packet fifo side //
    input axi_aclk,
    input axi_resetn,    
    
    output must_read,
    output reg error, // one or two packets were destroyed
    
    // Master Stream Ports
    output [255:0] m_axis_tdata,
    output [31:0] m_axis_tkeep, 
    output [0:0] m_axis_tuser, 
    output m_axis_tvalid,
    input  m_axis_tready,
    output m_axis_tlast
  );
  `include "../../../nf10_upb_lib/hdl/verilog/tkeep_coder.v"
  
  // framework clk
  wire clk = axi_aclk;
  wire reset = ~axi_resetn;
  
  // connections between mul1 and fifos (no ready)
  wire [127:0] to_fifos_axis_tdata;
  wire [3:0] to_fifos_axis_tkeep; // encoded
  wire [0:0] to_fifos_axis_tuser;
  wire to_fifos_axis_tvalid;
  wire to_fifos_axis_tlast;
  
  // connections between fifos and mul2 (no valid)
  wire [127:0] from_fifos_axis_tdata;
  wire [3:0] from_fifos_axis_tkeep; // encoded
  wire [0:0] from_fifos_axis_tuser;
  wire from_fifos_axis_tready;
  wire from_fifos_axis_tlast;

  // tkeep en- and decoding
  wire [2:0] s_axis_tkeep_enc = encode(s_axis_tkeep);    
  wire [7:0] dip_internal;
  assign dip_internal = {
    to_fifos_axis_tkeep,
    to_fifos_axis_tuser, 
    to_fifos_axis_tlast,
    2'b00
  };  
  wire [4:0] m_axis_tkeep_enc;
  assign m_axis_tkeep = decode(m_axis_tkeep_enc);  
  wire [7:0] dop_internal; 
  assign {from_fifos_axis_tkeep, from_fifos_axis_tuser, from_fifos_axis_tlast} = dop_internal[7:2];
  
  // ctrl fifos
  wire full1;
  wire full2;
  wire full = full1 || full2;
  wire almost_full1;
  wire almost_full2;
  reg almost_full;
  always @(posedge clk) begin
    almost_full = almost_full1 || almost_full2;
    error <= full;
  end
  wire wr_en = (to_fifos_axis_tvalid != 0) && !full;
  wire empty1;
  wire empty2;
  wire empty = empty1 || empty2;
  wire rd_en = from_fifos_axis_tready && !empty;
  assign must_read = almost_full;
  
  // double width in mac clk domain
  width_multiplier #(.MULTIPLY_VALUE(2), .INPUT_WIDTH(64)) mul1 (
    .clk(clk156), 
    .reset(reset),
    .m_axis_tdata(to_fifos_axis_tdata), 
    .m_axis_tkeep(to_fifos_axis_tkeep), 
    .m_axis_tvalid(to_fifos_axis_tvalid), 
    .m_axis_tready(1'b1), 
    .m_axis_tuser(to_fifos_axis_tuser),
    .m_axis_tlast(to_fifos_axis_tlast), 
    .s_axis_tdata(s_axis_tdata), 
    .s_axis_tkeep(s_axis_tkeep_enc), 
    .s_axis_tvalid(s_axis_tvalid), 
    .s_axis_tready(), // must be 1
    .s_axis_tuser(s_axis_tuser),
    .s_axis_tlast(s_axis_tlast)
  );

  // fifo 1 of 2
  FIFO36_72 #(
   	.SIM_MODE("FAST"),
   	.ALMOST_FULL_OFFSET(9'hA), 
   	.ALMOST_EMPTY_OFFSET(9'hA),
   	.DO_REG(1),
   	.EN_ECC_READ("FALSE"),
   	.EN_ECC_WRITE("FALSE"),
   	.EN_SYN("FALSE"),
   	.FIRST_WORD_FALL_THROUGH("TRUE")
   	) fifo1 (
		.ALMOSTEMPTY(),
		.ALMOSTFULL(almost_full1),
		.DBITERR(),
		.DO(from_fifos_axis_tdata[127-:64]),
		.DOP(dop_internal),
		.ECCPARITY(),
		.EMPTY(empty1),
		.FULL(full1),
		.RDCOUNT(),
		.RDERR(),
		.SBITERR(),
		.WRCOUNT(),
		.WRERR(),
		.DI(to_fifos_axis_tdata[127-:64]),
		.DIP(dip_internal),
		.RDCLK(clk),
		.RDEN(rd_en),
		.RST(reset),
		.WRCLK(clk156),
		.WREN(wr_en)
   	);
    
  // fifo 2 of 2
  FIFO36_72 #(
   	.SIM_MODE("FAST"),
   	.ALMOST_FULL_OFFSET(9'hA),
   	.ALMOST_EMPTY_OFFSET(9'hA),
   	.DO_REG(1),
   	.EN_ECC_READ("FALSE"),
   	.EN_ECC_WRITE("FALSE"),
   	.EN_SYN("FALSE"),
   	.FIRST_WORD_FALL_THROUGH("TRUE")
   	) fifo2 (
		.ALMOSTEMPTY(),
		.ALMOSTFULL(almost_full2),
		.DBITERR(),
		.DO(from_fifos_axis_tdata[63-:64]),
		.DOP(),
		.ECCPARITY(),
		.EMPTY(empty2),
		.FULL(full2),
		.RDCOUNT(),
		.RDERR(),
		.SBITERR(),
		.WRCOUNT(),
		.WRERR(),
		.DI(to_fifos_axis_tdata[63-:64]),
		.DIP(dip_internal),
		.RDCLK(clk),
		.RDEN(rd_en),
		.RST(reset),
		.WRCLK(clk156),
		.WREN(wr_en)
   	);
  
  // double width in framework clk domain
  width_multiplier #(.MULTIPLY_VALUE(2), .INPUT_WIDTH(128)) mul2 (
    .clk(clk),
    .reset(reset), 
    .m_axis_tdata(m_axis_tdata), 
    .m_axis_tkeep(m_axis_tkeep_enc), 
    .m_axis_tvalid(m_axis_tvalid), 
    .m_axis_tready(m_axis_tready), 
    .m_axis_tuser(m_axis_tuser),
    .m_axis_tlast(m_axis_tlast), 
    .s_axis_tdata(from_fifos_axis_tdata), 
    .s_axis_tkeep(from_fifos_axis_tkeep), 
    .s_axis_tvalid(!empty), 
    .s_axis_tready(from_fifos_axis_tready), 
    .s_axis_tuser(from_fifos_axis_tuser),
    .s_axis_tlast(from_fifos_axis_tlast)
  );

endmodule
