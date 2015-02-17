/*
 * UPB flow_ctrl
 *
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
module flow_ctrl(
    input reset,

    // Packet FIFO side //
    input clk,
    input below_low,
    input above_high,
    
    // MAC side //
    input clk156,
    output reg pause_req = 0,
    output reg [15:0] pause_val
    );
    
  reg state = 0;
  reg state156 = 0;
  reg state156_old = 0;
  
  always @(posedge clk) begin
    if (reset)
      state <= 0;
    else if (!state && above_high)
      state <= 1;
    else if (state && below_low)
      state <= 0;
  end
  
  always @(posedge clk156) begin
    if (!reset) begin
      state156 <= state;
      state156_old <= state156;
    end else begin
      state156 <= 0;
      state156_old <= 0;
    end
  end
  
  always @(posedge clk156) begin
    pause_req <= 0;
    if (state156 && !state156_old) begin
      pause_req <= 1;
      pause_val <= 16'hFFFF;
    end
    else if (!state156 && state156_old) begin
      pause_req <= 1;
      pause_val <= 16'h0000;    
    end
  end

endmodule
