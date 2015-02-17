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

`default_nettype none

module nf10_upb_aurora_input#( 
    //parameter C_PORT_NUMBER = 5,
    //parameter C_INPORT_WIDTH = 3,
    //parameter C_OUTPORT_WIDTH = 8,
    parameter C_PACKET_LENGTH_WIDTH = 14,
    //parameter C_MAX_PACKET_LENGTH = 10000, // in byte
    parameter C_AXIS_DATA_WIDTH = 256,
    parameter SIM_GTXRESET_SPEEDUP = 1 
)
(
    // input wire arbiter side //
    input wire axi_resetn,    
    
    // Master Stream Ports
    output wire [C_AXIS_DATA_WIDTH-1:0] m_axis_tdata,
    output wire [(C_AXIS_DATA_WIDTH/8)-1:0] m_axis_tkeep, 
    output wire [C_PACKET_LENGTH_WIDTH-1:0] m_axis_tuser_packet_length,
    //output reg [C_INPORT_WIDTH-1:0] m_axis_tuser_in_port = C_PORT_NUMBER,
    //output reg [C_OUTPORT_WIDTH-1:0] m_axis_tuser_out_port = 0,
    //output reg [C_INPORT_WIDTH-1:0] m_axis_tuser_in_vport = 0,
    //output reg [C_OUTPORT_WIDTH-1:0] m_axis_tuser_out_vport = 0,
    output wire m_axis_tvalid,
    //input  wire m_axis_tready, // !
    output wire m_axis_tlast,
    output wire error,
	 
    // output queue side //
    output wire clk156,
    
    // Slave Stream Ports
    input wire [C_AXIS_DATA_WIDTH-1:0] s_axis_tdata,
    input wire [(C_AXIS_DATA_WIDTH/8)-1:0] s_axis_tkeep,
    input wire [C_PACKET_LENGTH_WIDTH-1:0] s_axis_tuser_packet_length,
    //input wire [C_INPORT_WIDTH-1:0] s_axis_tuser_in_port,
    //input wire [C_OUTPORT_WIDTH-1:0] s_axis_tuser_out_port,
    //input wire [C_INPORT_WIDTH-1:0] s_axis_tuser_in_vport,
    //input wire [C_OUTPORT_WIDTH-1:0] s_axis_tuser_out_vport,
    input wire s_axis_tvalid,
    output wire s_axis_tready,
    input wire s_axis_tlast,
    
    input wire flow_control_pause_req,
    input wire flow_control_pause_val,

    input wire RESET,
    input wire INIT_CLK,
    input wire GT_RESET_IN,
    input wire GTXD8_P,
    input wire GTXD8_N,
    input wire [0:9] RXP,
    input wire [0:9] RXN,
    output wire [0:9] TXP,
    output wire [0:9] TXN
    );
    
  `include "../../../../nf10_upb_lib/hdl/verilog/tkeep_coder.v"
  //`include "tkeep_coder.v"
  
  localparam AXIS_DATA_WIDTH_BYTE = log2(C_AXIS_DATA_WIDTH/8);
  
  wire axi_reset = !axi_resetn;

  // Stream TX Interface
  wire    [0:319]    tx_d_i;
  wire               tx_src_rdy_n_i;
  wire               tx_dst_rdy_n_i;
  // Stream RX Interface
  wire    [0:319]    rx_d_i;
  wire               rx_src_rdy_n_i;
  // V5 Reference Clock Interface
  wire               GTXD8_left_i;

  // Error Detection Interface
  wire               hard_err_i;
  wire               soft_err_i;
  // Status
  wire               channel_up_i;
  wire    [0:9]      lane_up_i;
  // Clock Compensation Control Interface
  wire               warn_cc_i;
  wire               do_cc_i;
  // System Interface
  wire               pll_not_locked_i;
  wire               user_clk_i;
  wire               sync_clk_i;
  wire               reset_i; // = system_reset_i
  wire               power_down_i;
  wire    [2:0]      loopback_i;
  wire               tx_lock_i;
  wire               tx_lock_out_i;

  wire               tx_out_clk_i;
  wire               buf_tx_out_clk_i;

  wire               gt_reset_i; 
  wire               system_reset_i;
  //Frame check signals
  wire    [0:7]      err_count_i;

  wire [63:0] sync_in_i;
  wire [15:0] sync_out_i;

  wire        lane_up_i_i;
  wire        tx_lock_i_i;
  wire        lane_up_reduce_i;
  wire        rst_cc_module_i;

  wire    [0:319]    tied_to_gnd_vec_i;

  assign lane_up_reduce_i  = &lane_up_i;
  assign rst_cc_module_i   = !lane_up_reduce_i;
  
  // System Interface
  assign  tied_to_gnd_vec_i   =   320'd0;
  assign  power_down_i        =   1'b0;
  assign  loopback_i          =   3'b000;
  
  assign clk156 = user_clk_i;
  
  //
  // clock buffers
  //
  IBUFDS IBUFDS
  (
    .I(GTXD8_P),
    .IB(GTXD8_N),
    .O(GTXD8_left_i)
  );

  BUFG BUFG
  (
    .I(tx_out_clk_i),
    .O(buf_tx_out_clk_i)
  );

  // Instantiate a clock module for clock division.
  aurora_8b10b_v5_3_CLOCK_MODULE clock_module_i
  (
    .GT_CLK(buf_tx_out_clk_i),
    .GT_CLK_LOCKED(tx_lock_out_i),
    .USER_CLK(user_clk_i),
    .SYNC_CLK(sync_clk_i),
    .PLL_NOT_LOCKED(pll_not_locked_i)
  );  
  
  //
  // instatiation of aurora core modules
  //
  aurora_8b10b_v5_3 #
  (
    .SIM_GTXRESET_SPEEDUP(SIM_GTXRESET_SPEEDUP)
  )
  aurora_module_i
  (
    // Stream TX Interface
    .TX_D(tx_d_i),
    .TX_SRC_RDY_N(tx_src_rdy_n_i),
    .TX_DST_RDY_N(tx_dst_rdy_n_i),
    // Stream RX Interface
    .RX_D(rx_d_i),
    .RX_SRC_RDY_N(rx_src_rdy_n_i),
    // V5 Serial I/O
    .RXP(RXP),
    .RXN(RXN),
    .TXP(TXP),
    .TXN(TXN),
    // V5 Reference Clock Interface
    .GTXD8(GTXD8_left_i),
    // Error Detection Interface
    .HARD_ERR(hard_err_i),
    .SOFT_ERR(soft_err_i),
    // Status
    .CHANNEL_UP(channel_up_i),
    .LANE_UP(lane_up_i),
    // Clock Compensation Control Interface
    .WARN_CC(warn_cc_i),
    .DO_CC(do_cc_i),
    // System Interface
    .USER_CLK(user_clk_i),
    .SYNC_CLK(sync_clk_i),
    .RESET(reset_i),
    .POWER_DOWN(power_down_i),
    .LOOPBACK(loopback_i),
    .GT_RESET(gt_reset_i),
    .TX_LOCK(tx_lock_i),
    .TX_OUT_CLK(tx_out_clk_i)
  );

  aurora_8b10b_v5_3_STANDARD_CC_MODULE standard_cc_module_i
  (
    .RESET(rst_cc_module_i),
    // Clock Compensation Control Interface
    .WARN_CC(warn_cc_i),
    .DO_CC(do_cc_i),
    // System Interface
    .PLL_NOT_LOCKED(pll_not_locked_i),
    .USER_CLK(user_clk_i)
  );

  aurora_8b10b_v5_3_RESET_LOGIC reset_logic_i
  (
    .RESET(RESET),
    .USER_CLK(user_clk_i),
    .INIT_CLK(INIT_CLK),
    .GT_RESET_IN(GT_RESET_IN),
    .TX_LOCK_IN(tx_lock_i),
    .PLL_NOT_LOCKED(pll_not_locked_i),
    .TX_LOCK_OUT(tx_lock_out_i),
    .SYSTEM_RESET(system_reset_i),
    .GT_RESET_OUT(gt_reset_i)
  );
  
  // signal names after computing encoded s_axis_tkeep
  wire [C_AXIS_DATA_WIDTH-1:0] s_axis_tdata_encoded_stream;
  wire [log2(C_AXIS_DATA_WIDTH/8)-1:0] s_axis_tkeep_encoded_stream;
  wire [C_PACKET_LENGTH_WIDTH-1:0] s_axis_tuser_packet_length_encoded_stream;
  wire s_axis_tvalid_encoded_stream;
  wire s_axis_tready_encoded_stream;
  wire s_axis_tlast_encoded_stream;
  
  localparam TX_DELAY_CYCLES = 2;
  // tx signals
  reg [C_AXIS_DATA_WIDTH-1:0] tx_data = 0;
  reg tx_stopped = 0; // if an stop request has been received
  reg flow_control_valid = 0;
  reg [C_AXIS_DATA_WIDTH-1:0] tx_data_delayed [TX_DELAY_CYCLES-1:0];
  reg [31:0] tx_control_delayed [TX_DELAY_CYCLES-1:0];
  reg [31:0] tx_crc_delayed [TX_DELAY_CYCLES-1:0];
  wire [AXIS_DATA_WIDTH_BYTE-1:0] s_axis_tkeep_enc;
  wire [31:0] tx_crc;
  wire [31:0] tx_control_lane;
  wire [319:0] tx_d_i_reverse;
  wire fetch_s_axis_data;
  wire to_send_data_valid;
  wire to_send_flow_ctrl_valid;
  assign to_send_data_valid = tx_control_delayed[0][6] === 1'b1;
  assign to_send_flow_ctrl_valid = tx_control_delayed[0][8] === 1'b1;
  assign s_axis_tkeep_enc = s_axis_tkeep_encoded_stream;
  assign tx_src_rdy_n_i = !(to_send_data_valid || to_send_flow_ctrl_valid); // assert if data or flow control valid
  assign fetch_s_axis_data = (!tx_dst_rdy_n_i && !tx_src_rdy_n_i || !to_send_data_valid) && !tx_stopped && channel_up_i;
  assign s_axis_tready_encoded_stream = fetch_s_axis_data;
  assign tx_control_lane = {9'b0, s_axis_tuser_packet_length_encoded_stream, flow_control_valid, flow_control_pause_val, 
    s_axis_tvalid_encoded_stream && fetch_s_axis_data, s_axis_tlast_encoded_stream, s_axis_tkeep_enc};
  assign tx_d_i_reverse = {tx_data_delayed[0], tx_control_delayed[0], tx_crc_delayed[0]};
  assign tx_d_i = tx_d_i_reverse;

  // register shifting for retiming
  genvar i;
  generate 
    for (i = 0; i < TX_DELAY_CYCLES - 1; i = i + 1) begin : txretiming
      always @(posedge user_clk_i) begin
        if (axi_reset) begin
          tx_data_delayed[i] <= 0;
          tx_control_delayed[i] <= 0;
          tx_crc_delayed[i] <= 0;
        end else if (!tx_dst_rdy_n_i && !tx_src_rdy_n_i && channel_up_i || !to_send_data_valid && !to_send_flow_ctrl_valid) begin
          tx_data_delayed[i] <= tx_data_delayed[i+1];
          tx_control_delayed[i] <= tx_control_delayed[i+1];
          tx_crc_delayed[i] <= tx_crc_delayed[i+1];
        end    
      end
    end
  endgenerate  
  always @(posedge user_clk_i) begin    
    if (axi_reset) begin
      tx_data_delayed[TX_DELAY_CYCLES-1] <= 0;
      tx_control_delayed[TX_DELAY_CYCLES-1] <= 0;
      tx_crc_delayed[TX_DELAY_CYCLES-1] <= 0;
    end else if (!tx_dst_rdy_n_i && !tx_src_rdy_n_i && channel_up_i || !to_send_data_valid && !to_send_flow_ctrl_valid) begin
      tx_data_delayed[TX_DELAY_CYCLES-1] <= s_axis_tdata_encoded_stream;      
      tx_control_delayed[TX_DELAY_CYCLES-1] <= tx_control_lane;      
      tx_crc_delayed[TX_DELAY_CYCLES-1] <= tx_crc;
    end
  end
  
  // make sure to transmit stop request
  always @(posedge user_clk_i) begin
    if (!tx_dst_rdy_n_i && !tx_src_rdy_n_i && channel_up_i || !flow_control_valid)
      flow_control_valid <= flow_control_pause_req;
  end
  
  localparam RX_DELAY_CYCLES = 2;
  // rx signals
  reg [C_AXIS_DATA_WIDTH-1:0] rx_data_delayed [RX_DELAY_CYCLES-1:0];
  reg [31:0] rx_control_delayed [RX_DELAY_CYCLES-1:0];
  reg [31:0] rx_crc_delayed [TX_DELAY_CYCLES-1:0];
  reg [31:0] rx_crc_calc_delayed [TX_DELAY_CYCLES-1:0];
  reg [RX_DELAY_CYCLES-1:0] rx_src_rdy_n_i_delayed; // as vector to use unary operator |
  wire [31:0] rx_crc;
  wire [319:0] rx_d_i_reverse;
  assign rx_d_i_reverse = rx_d_i;

  // register shifting for retiming
  genvar j;
  generate 
    for (j = 0; j < RX_DELAY_CYCLES - 1; j = j + 1) begin : rxretiming
      always @(posedge user_clk_i) begin
        if (axi_reset) begin
          rx_data_delayed[j] <= 0;
          rx_control_delayed[j] <= 0;
          rx_crc_delayed[j] <= 0;
          rx_crc_calc_delayed[j] <= 0;
          rx_src_rdy_n_i_delayed[j] <= 1;
        end else if (!rx_src_rdy_n_i && channel_up_i || !(& rx_src_rdy_n_i_delayed)) begin
          rx_data_delayed[j] <= rx_data_delayed[j+1];
          rx_control_delayed[j] <= rx_control_delayed[j+1];
          rx_crc_delayed[j] <= rx_crc_delayed[j+1];
          rx_crc_calc_delayed[j] <= rx_crc_calc_delayed[j+1];
          rx_src_rdy_n_i_delayed[j] <= rx_src_rdy_n_i_delayed[j+1];
        end    
      end
    end
  endgenerate
  always @(posedge user_clk_i) begin    
    if (axi_reset) begin
      rx_data_delayed[RX_DELAY_CYCLES-1] <= 0;      
      rx_control_delayed[RX_DELAY_CYCLES-1] <= 0;  
      rx_crc_delayed[RX_DELAY_CYCLES-1] <= 0;
      rx_crc_calc_delayed[RX_DELAY_CYCLES-1] <= 0;
      rx_src_rdy_n_i_delayed[RX_DELAY_CYCLES-1] <= 1;
    end else if (!rx_src_rdy_n_i && channel_up_i || !(& rx_src_rdy_n_i_delayed)) begin
      rx_data_delayed[RX_DELAY_CYCLES-1] <= rx_d_i_reverse[319:64];      
      rx_control_delayed[RX_DELAY_CYCLES-1] <= rx_d_i_reverse[63:32];  
      rx_crc_delayed[RX_DELAY_CYCLES-1] <= rx_d_i_reverse[31:0];
      rx_crc_calc_delayed[RX_DELAY_CYCLES-1] <= rx_crc;
      rx_src_rdy_n_i_delayed[RX_DELAY_CYCLES-1] <= rx_src_rdy_n_i;
    end
  end

  // assign master stream signals
  assign m_axis_tvalid = !rx_src_rdy_n_i_delayed[0] && rx_control_delayed[0][6];
  assign m_axis_tkeep = decode(rx_control_delayed[0][AXIS_DATA_WIDTH_BYTE-1:0]);
  assign m_axis_tlast = rx_control_delayed[0][5];
  assign m_axis_tdata = rx_data_delayed[0];
  assign m_axis_tuser_packet_length = rx_control_delayed[0][22-:C_PACKET_LENGTH_WIDTH];

  // react on pause values
  always @(posedge user_clk_i) begin
    if (axi_reset)
      tx_stopped <= 0;
    else if (!rx_src_rdy_n_i && channel_up_i && rx_d_i_reverse[40])
      tx_stopped <= rx_d_i_reverse[39]; // skip 32bit crc
    else if (!channel_up_i)
      tx_stopped <= 0;
  end

  assign error = !(rx_crc_delayed[0] == rx_crc_calc_delayed[0]) && m_axis_tvalid; // || hard_err_i || soft_err_i , delayed

  // Instantiate the crc modules
  crc802_3 rx_crc_eth (
    .data_in(rx_d_i_reverse[319:32]),
    .crc_en(!rx_src_rdy_n_i && channel_up_i),
    .crc_out(rx_crc), 
    .rst(axi_reset || !channel_up_i || rx_d_i_reverse[37] && !rx_src_rdy_n_i), // rx_d_i_reverse[37] = tlast
    .clk(user_clk_i)
    );
  crc802_3 tx_crc_eth (
    .data_in({s_axis_tdata_encoded_stream, tx_control_lane}),
    .crc_en(s_axis_tvalid_encoded_stream && s_axis_tready_encoded_stream || flow_control_valid), 
    .crc_out(tx_crc),
    .rst(axi_reset || !channel_up_i || s_axis_tvalid_encoded_stream && s_axis_tready_encoded_stream && s_axis_tlast_encoded_stream), 
    .clk(user_clk_i)
  );
  
  // Instantiate the encodeInOneCycle module
  encodeInOneCycle encodeOutputQueueSAxis (
    .axi_aclk(user_clk_i), 
    .axi_resetn(axi_resetn), 
    .m_axis_tdata(s_axis_tdata_encoded_stream), 
    .m_axis_tkeep_enc(s_axis_tkeep_encoded_stream), 
    .m_axis_tuser_packet_length(s_axis_tuser_packet_length_encoded_stream), 
    .m_axis_tuser_in_port(), 
    .m_axis_tuser_out_port(), 
    .m_axis_tuser_in_vport(), 
    .m_axis_tuser_out_vport(), 
    .m_axis_tvalid(s_axis_tvalid_encoded_stream), 
    .m_axis_tready(s_axis_tready_encoded_stream), 
    .m_axis_tlast(s_axis_tlast_encoded_stream), 
    .s_axis_tdata(s_axis_tdata), 
    .s_axis_tkeep(s_axis_tkeep), 
    .s_axis_tuser_packet_length(s_axis_tuser_packet_length), 
    .s_axis_tuser_in_port(), 
    .s_axis_tuser_out_port(), 
    .s_axis_tuser_in_vport(), 
    .s_axis_tuser_out_vport(), 
    .s_axis_tvalid(s_axis_tvalid), 
    .s_axis_tready(s_axis_tready), 
    .s_axis_tlast(s_axis_tlast)
    );


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

