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
module OutputBufferForTest #( 
    parameter C_INPORT_WIDTH = 3,
    parameter C_OUTPORT_WIDTH = 8,
    parameter C_PACKET_LENGTH_WIDTH = 14,
    parameter C_AXIS_DATA_WIDTH = 256
)
(
    input axi_aclk,
    input axi_resetn,  
    input clk156,
    
    // Master Stream Ports
    output [C_AXIS_DATA_WIDTH-1:0] m_axis_tdata,
    output [(C_AXIS_DATA_WIDTH/8)-1:0] m_axis_tkeep, 
    output reg [C_PACKET_LENGTH_WIDTH-1:0] m_axis_tuser_packet_length = 0,
    output reg [C_INPORT_WIDTH-1:0] m_axis_tuser_in_port = 0,
    output reg [C_OUTPORT_WIDTH-1:0] m_axis_tuser_out_port = 0,
    output reg [C_INPORT_WIDTH-1:0] m_axis_tuser_in_vport = 0,
    output reg [C_OUTPORT_WIDTH-1:0] m_axis_tuser_out_vport = 0,
    output m_axis_tvalid,
    input  m_axis_tready,
    output m_axis_tlast,
    
    // Slave Stream Ports
    input [C_AXIS_DATA_WIDTH-1:0] s_axis_tdata,
    input [(C_AXIS_DATA_WIDTH/8)-1:0] s_axis_tkeep,
    input [C_PACKET_LENGTH_WIDTH-1:0] s_axis_tuser_packet_length,
    input [C_INPORT_WIDTH-1:0] s_axis_tuser_in_port,
    input [C_OUTPORT_WIDTH-1:0] s_axis_tuser_out_port,
    input [C_INPORT_WIDTH-1:0] s_axis_tuser_in_vport,
    input [C_OUTPORT_WIDTH-1:0] s_axis_tuser_out_vport,
    input s_axis_tvalid,
    output s_axis_tready,
    input s_axis_tlast
    );
  `include "../../../nf10_upb_lib/hdl/verilog/tkeep_coder.v"

  wire clk = axi_aclk;
  wire reset = !axi_resetn;

  wire [63:0] dip_internal;
  wire [63:0] dop_internal;
  assign dip_internal = {
    s_axis_tkeep,
    s_axis_tlast,
    31'b0
  };  
  assign {m_axis_tkeep, m_axis_tlast} = dop_internal[63-:33];
  
  wire fifo_full_1, fifo_full_2, fifo_full_3, fifo_full_4, fifo_empty_1, fifo_empty_2, fifo_empty_3, fifo_empty_4, fifo_full_5, fifo_empty_5;
  wire fifo_full = fifo_full_1 || fifo_full_2 || fifo_full_3 || fifo_full_4 || fifo_full_5;
  wire fifo_empty = fifo_empty_1 || fifo_empty_2 || fifo_empty_3 || fifo_empty_4 || fifo_empty_5;
  
  wire wr_en = s_axis_tvalid && !fifo_full;
  assign s_axis_tready = !fifo_full;
  
  wire rd_en = !fifo_empty && m_axis_tready;
  assign m_axis_tvalid = !fifo_empty;

  FIFO36_72 #(
   	.SIM_MODE("FAST"),
   	.ALMOST_FULL_OFFSET(9'd300),
   	.ALMOST_EMPTY_OFFSET(9'hA),
   	.DO_REG(1),
   	.EN_ECC_READ("FALSE"),
   	.EN_ECC_WRITE("FALSE"),
   	.EN_SYN("FALSE"),
   	.FIRST_WORD_FALL_THROUGH("TRUE")
   	) fifo1 (
		.ALMOSTEMPTY(),
		.ALMOSTFULL(),
		.DBITERR(),
		.DO(m_axis_tdata[255-:64]),
		.DOP(),
		.ECCPARITY(),
		.EMPTY(fifo_empty_1),
		.FULL(fifo_full_1),
		.RDCOUNT(),
		.RDERR(),
		.SBITERR(),
		.WRCOUNT(),
		.WRERR(),
		.DI(s_axis_tdata[255-:64]),
		.DIP(),
		.RDCLK(clk156),
		.RDEN(rd_en),
		.RST(reset),
		.WRCLK(clk),
		.WREN(wr_en)
   	);

  FIFO36_72 #(
   	.SIM_MODE("FAST"),
   	.ALMOST_FULL_OFFSET(9'd300),
   	.ALMOST_EMPTY_OFFSET(9'hA),
   	.DO_REG(1),
   	.EN_ECC_READ("FALSE"),
   	.EN_ECC_WRITE("FALSE"),
   	.EN_SYN("FALSE"),
   	.FIRST_WORD_FALL_THROUGH("TRUE")
   	) fifo2 (
		.ALMOSTEMPTY(),
		.ALMOSTFULL(),
		.DBITERR(),
		.DO(m_axis_tdata[191-:64]),
		.DOP(),
		.ECCPARITY(),
		.EMPTY(fifo_empty_2),
		.FULL(fifo_full_2),
		.RDCOUNT(),
		.RDERR(),
		.SBITERR(),
		.WRCOUNT(),
		.WRERR(),
		.DI(s_axis_tdata[191-:64]),
		.DIP(),
		.RDCLK(clk156),
		.RDEN(rd_en),
		.RST(reset),
		.WRCLK(clk),
		.WREN(wr_en)
   	);

  FIFO36_72 #(
   	.SIM_MODE("FAST"),
   	.ALMOST_FULL_OFFSET(9'd300),
   	.ALMOST_EMPTY_OFFSET(9'hA),
   	.DO_REG(1),
   	.EN_ECC_READ("FALSE"),
   	.EN_ECC_WRITE("FALSE"),
   	.EN_SYN("FALSE"),
   	.FIRST_WORD_FALL_THROUGH("TRUE")
   	) fifo3 (
		.ALMOSTEMPTY(),
		.ALMOSTFULL(),
		.DBITERR(),
		.DO(m_axis_tdata[127-:64]),
		.DOP(),
		.ECCPARITY(),
		.EMPTY(fifo_empty_3),
		.FULL(fifo_full_3),
		.RDCOUNT(),
		.RDERR(),
		.SBITERR(),
		.WRCOUNT(),
		.WRERR(),
		.DI(s_axis_tdata[127-:64]),
		.DIP(),
		.RDCLK(clk156),
		.RDEN(rd_en),
		.RST(reset),
		.WRCLK(clk),
		.WREN(wr_en)
   	);

  FIFO36_72 #(
   	.SIM_MODE("FAST"),
   	.ALMOST_FULL_OFFSET(9'd300),
   	.ALMOST_EMPTY_OFFSET(9'hA),
   	.DO_REG(1),
   	.EN_ECC_READ("FALSE"),
   	.EN_ECC_WRITE("FALSE"),
   	.EN_SYN("FALSE"),
   	.FIRST_WORD_FALL_THROUGH("TRUE")
   	) fifo4 (
		.ALMOSTEMPTY(),
		.ALMOSTFULL(),
		.DBITERR(),
		.DO(m_axis_tdata[63-:64]),
		.DOP(),
		.ECCPARITY(),
		.EMPTY(fifo_empty_4),
		.FULL(fifo_full_4),
		.RDCOUNT(),
		.RDERR(),
		.SBITERR(),
		.WRCOUNT(),
		.WRERR(),
		.DI(s_axis_tdata[63-:64]),
		.DIP(),
		.RDCLK(clk156),
		.RDEN(rd_en),
		.RST(reset),
		.WRCLK(clk),
		.WREN(wr_en)
   	);
 
  FIFO36_72 #(
   	.SIM_MODE("FAST"),
   	.ALMOST_FULL_OFFSET(9'd300),
   	.ALMOST_EMPTY_OFFSET(9'hA),
   	.DO_REG(1),
   	.EN_ECC_READ("FALSE"),
   	.EN_ECC_WRITE("FALSE"),
   	.EN_SYN("FALSE"),
   	.FIRST_WORD_FALL_THROUGH("TRUE")
   	) fifo5 (
		.ALMOSTEMPTY(),
		.ALMOSTFULL(),
		.DBITERR(),
		.DO(dop_internal),
		.DOP(),
		.ECCPARITY(),
		.EMPTY(fifo_empty_5),
		.FULL(fifo_full_5),
		.RDCOUNT(),
		.RDERR(),
		.SBITERR(),
		.WRCOUNT(),
		.WRERR(),
		.DI(dip_internal),
		.DIP(),
		.RDCLK(clk156),
		.RDEN(rd_en),
		.RST(reset),
		.WRCLK(clk),
		.WREN(wr_en)
   	);
  
endmodule
