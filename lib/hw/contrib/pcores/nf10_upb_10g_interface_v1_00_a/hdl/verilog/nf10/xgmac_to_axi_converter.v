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
 * Description: adapter between xgmac and axi stream
 *
 */
module xgmac_to_axi_converter
(
    input reset,
    input clk156,	 
	 
    // xgmac ports
    output reg [63:0] tx_data,
    output reg [7:0] tx_data_valid,
    output reg tx_start,
    input  tx_ack,
    output reg tx_underrun,

    input [63:0] rx_data,
    input [7:0] rx_data_valid,

    input rx_good_frame,
    input rx_bad_frame,
	 
    // Master Stream Ports
    output [63:0] m_axis_tdata,
    output [7:0] m_axis_tkeep, 
    output [0:0] m_axis_tuser,
    output m_axis_tvalid,
    input  m_axis_tready, // ignored
    output m_axis_tlast,

    // Slave Stream Ports
    input [63:0] s_axis_tdata,
    input [7:0] s_axis_tkeep, 
    input [0:0] s_axis_tuser,
    input  s_axis_tvalid,
    output s_axis_tready,
    input  s_axis_tlast
  );

  // rx
  reg [63:0] rx_data_delayed;
  reg [7:0] rx_data_valid_delayed;
  reg rx_good_frame_delayed;
  reg rx_bad_frame_delayed;

  always @(posedge clk156) begin // delay signals
    if ((rx_data_valid != 0) || (rx_good_frame_delayed || rx_bad_frame_delayed)) begin
      rx_data_delayed <= rx_data;
      rx_data_valid_delayed <= rx_data_valid;
    end
    rx_good_frame_delayed <= rx_good_frame;
    rx_bad_frame_delayed <= rx_bad_frame;
  end

  wire rx_last = (rx_data_valid_delayed != 0) && (rx_data_valid == 0);
  assign m_axis_tvalid =  rx_last && (rx_good_frame_delayed || rx_bad_frame_delayed) // transmit last frame after receiving good / bad
    || (rx_data_valid_delayed != 0) && !rx_last; // transmit valid frame if not last
  assign m_axis_tlast = rx_last;
  assign m_axis_tdata = rx_data_delayed;
  assign m_axis_tkeep = rx_data_valid_delayed;
  assign m_axis_tuser = rx_bad_frame_delayed;

	 
   
   
  // tx
  localparam IDLE = 0;
  localparam WAIT_FOR_ACK = 1;
  localparam SEND_PKT = 2;

  
    reg [1:0] tx_state = IDLE;
  

  
    assign s_axis_tready = tx_ack || (tx_state == SEND_PKT);
 
    always @(*) begin
        if(tx_state==IDLE || tx_start) begin
            tx_data_valid = 8'b0;
        end
        else begin
            tx_data_valid = s_axis_tkeep;
        end
        tx_data = s_axis_tdata;
    end
    
    always @(posedge clk156) begin
        tx_start <= 0;
        tx_underrun <= 0;
        case(tx_state)
            IDLE: begin
                if(s_axis_tvalid) begin
                    tx_start <= 1;
                    tx_state <= WAIT_FOR_ACK;
                end
            end
            WAIT_FOR_ACK: begin
                if(tx_ack) begin
                    tx_state <= SEND_PKT;
                end
            end
            SEND_PKT: begin
                if(~s_axis_tvalid) begin
                    tx_underrun <=1;
                end
                if(s_axis_tlast) begin
                    tx_state<=IDLE;
                end 
            end
        endcase
        if(reset) begin
            tx_state <= IDLE;
        end
    end


endmodule
