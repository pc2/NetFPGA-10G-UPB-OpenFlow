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
module width_divider #(
    parameter OUTPUT_WIDTH = 64,
    parameter DIVISOR = 4
)
(
    input clk,
    input reset,    
    
    // Master Stream Ports
    output [OUTPUT_WIDTH-1:0] m_axis_tdata,
    output [(OUTPUT_WIDTH/8)-1:0] m_axis_tkeep, 
    output m_axis_tvalid,
    input  m_axis_tready,
    output [0:0] m_axis_tuser,
    output m_axis_tlast,

    // Slave Stream Ports
    input [(OUTPUT_WIDTH*DIVISOR)-1:0] s_axis_tdata,
    input [(OUTPUT_WIDTH*DIVISOR/8)-1:0] s_axis_tkeep,
    input s_axis_tvalid,
    output s_axis_tready,
    input [0:0] s_axis_tuser,
    input s_axis_tlast
    );
  
  reg [log2(DIVISOR)-1:0] sent = 0;
  
  wire [(OUTPUT_WIDTH*DIVISOR)-1:0] data = s_axis_tdata >> sent * OUTPUT_WIDTH;
  wire [(OUTPUT_WIDTH*DIVISOR/8)-1:0] keep = s_axis_tkeep >> sent * OUTPUT_WIDTH / 8;
  
  assign m_axis_tvalid = s_axis_tvalid;
  assign m_axis_tkeep = keep[OUTPUT_WIDTH/8-1:0];
  assign m_axis_tdata = data[OUTPUT_WIDTH-1:0];
  assign s_axis_tready = ((sent == DIVISOR - 1) || m_axis_tlast) && m_axis_tready;
  assign m_axis_tlast = s_axis_tlast && ((sent == DIVISOR - 1) || (s_axis_tkeep[(sent+1)*OUTPUT_WIDTH/8] == 1'b0)); 
  assign m_axis_tuser = s_axis_tuser;  
  
  always @(posedge clk) begin
    if (reset)
      sent <= 0;
    else begin
      if (m_axis_tready && s_axis_tvalid) begin
        if ((sent == DIVISOR - 1) || m_axis_tlast)
          sent <= 0;
        else
          sent <= sent + 1;
      end
    end
  end


  function [32:0] log2;
    input [32:0] number;
    reg [5:0] i;
    begin
      log2 = number + 1;
      for (i = 0; i <= 32; i = i + 1) begin
        if (2**i == log2)
          log2 = i;
      end
    end
  endfunction // log2

endmodule
