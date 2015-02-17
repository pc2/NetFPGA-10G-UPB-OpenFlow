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
module width_multiplier #(
    parameter INPUT_WIDTH = 64,
    parameter MULTIPLY_VALUE = 4
)
(
    input clk,
    input reset,    
    
    // Master Stream Ports
    output [(INPUT_WIDTH*MULTIPLY_VALUE)-1:0] m_axis_tdata,
    output [log2(INPUT_WIDTH*MULTIPLY_VALUE/8)-1:0] m_axis_tkeep, 
    output m_axis_tvalid,
    input  m_axis_tready,
    output [0:0] m_axis_tuser,
    output m_axis_tlast,

    // Slave Stream Ports
    input [INPUT_WIDTH-1:0] s_axis_tdata,
    input [log2(INPUT_WIDTH/8)-1:0] s_axis_tkeep,
    input s_axis_tvalid,
    output s_axis_tready,
    input [0:0] s_axis_tuser,
    input s_axis_tlast
    );
  
  reg [INPUT_WIDTH-1:0] data [MULTIPLY_VALUE-2:0];
  reg [log2(MULTIPLY_VALUE):0] received = 0;  
  
  // assign m_axis_tdata
  genvar i;
  generate
    for (i = 0; i < MULTIPLY_VALUE - 1; i = i + 1) begin: data_assignment
      assign m_axis_tdata[(i+1)*INPUT_WIDTH-1-:INPUT_WIDTH] = getData(i, received, data[i], s_axis_tdata); //data[i];
    end
  endgenerate
  assign m_axis_tdata[MULTIPLY_VALUE*INPUT_WIDTH-1-:INPUT_WIDTH] = getData(MULTIPLY_VALUE - 1, received, 0, s_axis_tdata); //data[last];
  function [INPUT_WIDTH-1:0] getData;
    input [MULTIPLY_VALUE-1:0] i;
    input [log2(MULTIPLY_VALUE):0] received;  
    input [INPUT_WIDTH-1:0] data;
    input [INPUT_WIDTH-1:0] s_axis_tdata;
    begin
      if (i == received)
        getData = s_axis_tdata;
      else if (i < received)
        getData = data;
      else
        getData = 0;
    end
  endfunction
    
  // assign other m_axis signals  
  assign m_axis_tkeep = {2'b0, received} * (INPUT_WIDTH/{4'd8}) + {1'b0, s_axis_tkeep};
  assign m_axis_tuser = s_axis_tuser;
  assign m_axis_tlast = s_axis_tlast;
  assign m_axis_tvalid = (s_axis_tvalid && s_axis_tlast) || ((received == MULTIPLY_VALUE - 1) && s_axis_tvalid);

  // assign s_axis signal
  assign s_axis_tready = ((received < MULTIPLY_VALUE - 1) && !s_axis_tlast) || m_axis_tready;

  // control data and received
  always @(posedge clk) begin
    if (reset) begin 
      received <= 0;
    end else begin
      if (m_axis_tvalid && m_axis_tready) begin
        received <= 0;
      end else if (s_axis_tvalid && s_axis_tready) begin
        data[received] <= s_axis_tdata;
        received <= received + 1;
      end
    end
  end


 function integer log2;
    input integer number;
    begin
       log2=0;
       while(2**log2<number) begin
          log2=log2+1;
       end
    end
 endfunction // log2

endmodule
