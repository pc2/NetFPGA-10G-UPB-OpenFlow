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

module nf10_upb_interconnect_tb;

  //parameter CLOCKPERIOD_AXI = (20/3);
  parameter CLOCKPERIOD_AXI = 6.7;
  parameter       CLOCKPERIOD_1 = 6.4;
  parameter       CLOCKPERIOD_2 = 6.4;

  parameter       LATENCY0 = 0;
  parameter       LATENCY1 = 0;
  parameter       LATENCY2 = 0;
  parameter       LATENCY3 = 0;
  parameter       LATENCY4 = 0;
  parameter       LATENCY5 = 0;
  parameter       LATENCY6 = 0;
  parameter       LATENCY7 = 0;
  parameter       LATENCY8 = 0;
  parameter       LATENCY9 = 0;
  
  reg                gsr_r;
  reg                gts_r;
  reg                reset_i;
  
  reg                reference_clk_1_n_r;
  reg                reference_clk_2_n_r;
  wire               reference_clk_1_p_r;
  wire               reference_clk_2_p_r;    
    

	reg axi_aclk;
	reg axi_resetn;
	wire [255:0] output_queue_s_axis1_tdata;
	wire [31:0] output_queue_s_axis1_tkeep;
	wire [13:0] output_queue_s_axis1_tuser_packet_length;
	wire [2:0] output_queue_s_axis1_tuser_in_port;
	wire [7:0] output_queue_s_axis1_tuser_out_port;
	wire [2:0] output_queue_s_axis1_tuser_in_vport;
	wire [7:0] output_queue_s_axis1_tuser_out_vport;
	wire output_queue_s_axis1_tvalid;
	wire output_queue_s_axis1_tlast;
	reg RESET;
  
	wire arbiter_m_axis1_tready;
	wire arbiter_m_axis2_tready;
  
	wire [255:0] output_queue_s_axis2_tdata;
	wire [31:0] output_queue_s_axis2_tkeep;
	wire [13:0] output_queue_s_axis2_tuser_packet_length;
	wire [2:0] output_queue_s_axis2_tuser_in_port;
	wire [7:0] output_queue_s_axis2_tuser_out_port;
	wire [2:0] output_queue_s_axis2_tuser_in_vport;
	wire [7:0] output_queue_s_axis2_tuser_out_vport;
	wire output_queue_s_axis2_tvalid;
	wire output_queue_s_axis2_tlast;

	wire [255:0] arbiter_m_axis1_tdata;
	wire [31:0] arbiter_m_axis1_tkeep;
	wire [13:0] arbiter_m_axis1_tuser_packet_length;
	wire [2:0] arbiter_m_axis1_tuser_in_port;
	wire [7:0] arbiter_m_axis1_tuser_out_port;
	wire [2:0] arbiter_m_axis1_tuser_in_vport;
	wire [7:0] arbiter_m_axis1_tuser_out_vport;
	wire arbiter_m_axis1_tvalid;
	wire arbiter_m_axis1_tlast;
	wire output_queue_clk1;
	wire output_queue_s_axis1_tready;
  
	wire [255:0] arbiter_m_axis2_tdata;
	wire [31:0] arbiter_m_axis2_tkeep;
	wire [13:0] arbiter_m_axis2_tuser_packet_length;
	wire [2:0] arbiter_m_axis2_tuser_in_port;
	wire [7:0] arbiter_m_axis2_tuser_out_port;
	wire [2:0] arbiter_m_axis2_tuser_in_vport;
	wire [7:0] arbiter_m_axis2_tuser_out_vport;
	wire arbiter_m_axis2_tvalid;
	wire arbiter_m_axis2_tlast;
	wire output_queue_clk2;
	wire output_queue_s_axis2_tready;

  wire [0:9] rxp_1_i; 
  wire [0:9] rxn_1_i;     
  wire [0:9] txp_1_i; 
  wire [0:9] txn_1_i; 
  wire [0:9] rxp_2_i; 
  wire [0:9] rxn_2_i;     
  wire [0:9] txp_2_i; 
  wire [0:9] txn_2_i; 
  
  reg [0:9] rxp_1_i_error = 0; 
  reg [0:9] rxn_1_i_error = 0;     
  reg [0:9] txp_1_i_error = 0; 
  reg [0:9] txn_1_i_error = 0; 
  reg [0:9] rxp_2_i_error = 0; 
  reg [0:9] rxn_2_i_error = 0;     
  reg [0:9] txp_2_i_error = 0; 
  reg [0:9] txn_2_i_error = 0; 
  
  wire [7:0] error_count1;
  wire [7:0] error_count2;
  
  //_________________________Serial Connections________________   
  assign #LATENCY0  rxp_1_i[0]      =    txp_2_i[0] ^ txp_2_i_error[0];
  assign #LATENCY0  rxn_1_i[0]      =    txn_2_i[0] ^ txn_2_i_error[0];
  assign #LATENCY0  rxp_2_i[0]      =    txp_1_i[0] ^ txp_1_i_error[0];
  assign #LATENCY0  rxn_2_i[0]      =    txn_1_i[0] ^ txn_1_i_error[0];

  assign #LATENCY1  rxp_1_i[1]      =    txp_2_i[1] ^ txp_2_i_error[1];
  assign #LATENCY1  rxn_1_i[1]      =    txn_2_i[1] ^ txn_2_i_error[1];
  assign #LATENCY1  rxp_2_i[1]      =    txp_1_i[1] ^ txp_1_i_error[1];
  assign #LATENCY1  rxn_2_i[1]      =    txn_1_i[1] ^ txn_1_i_error[1];

  assign #LATENCY2  rxp_1_i[2]      =    txp_2_i[2] ^ txp_2_i_error[2];
  assign #LATENCY2  rxn_1_i[2]      =    txn_2_i[2] ^ txn_2_i_error[2];
  assign #LATENCY2  rxp_2_i[2]      =    txp_1_i[2] ^ txp_1_i_error[2];
  assign #LATENCY2  rxn_2_i[2]      =    txn_1_i[2] ^ txn_1_i_error[2];

  assign #LATENCY3  rxp_1_i[3]      =    txp_2_i[3] ^ txp_2_i_error[3];
  assign #LATENCY3  rxn_1_i[3]      =    txn_2_i[3] ^ txn_2_i_error[3];
  assign #LATENCY3  rxp_2_i[3]      =    txp_1_i[3] ^ txp_1_i_error[3];
  assign #LATENCY3  rxn_2_i[3]      =    txn_1_i[3] ^ txn_1_i_error[3];

  assign #LATENCY4  rxp_1_i[4]      =    txp_2_i[4] ^ txp_2_i_error[4];
  assign #LATENCY4  rxn_1_i[4]      =    txn_2_i[4] ^ txn_2_i_error[4];
  assign #LATENCY4  rxp_2_i[4]      =    txp_1_i[4] ^ txp_1_i_error[4];
  assign #LATENCY4  rxn_2_i[4]      =    txn_1_i[4] ^ txn_1_i_error[4];

  assign #LATENCY5  rxp_1_i[5]      =    txp_2_i[5] ^ txp_2_i_error[5];
  assign #LATENCY5  rxn_1_i[5]      =    txn_2_i[5] ^ txn_2_i_error[5];
  assign #LATENCY5  rxp_2_i[5]      =    txp_1_i[5] ^ txp_1_i_error[5];
  assign #LATENCY5  rxn_2_i[5]      =    txn_1_i[5] ^ txn_1_i_error[5];

  assign #LATENCY6  rxp_1_i[6]      =    txp_2_i[6] ^ txp_2_i_error[6];
  assign #LATENCY6  rxn_1_i[6]      =    txn_2_i[6] ^ txn_2_i_error[6];
  assign #LATENCY6  rxp_2_i[6]      =    txp_1_i[6] ^ txp_1_i_error[6];
  assign #LATENCY6  rxn_2_i[6]      =    txn_1_i[6] ^ txn_1_i_error[6];

  assign #LATENCY7  rxp_1_i[7]      =    txp_2_i[7] ^ txp_2_i_error[7];
  assign #LATENCY7  rxn_1_i[7]      =    txn_2_i[7] ^ txn_2_i_error[7];
  assign #LATENCY7  rxp_2_i[7]      =    txp_1_i[7] ^ txp_1_i_error[7];
  assign #LATENCY7  rxn_2_i[7]      =    txn_1_i[7] ^ txn_1_i_error[7];

  assign #LATENCY8  rxp_1_i[8]      =    txp_2_i[8] ^ txp_2_i_error[8];
  assign #LATENCY8  rxn_1_i[8]      =    txn_2_i[8] ^ txn_2_i_error[8];
  assign #LATENCY8  rxp_2_i[8]      =    txp_1_i[8] ^ txp_1_i_error[8];
  assign #LATENCY8  rxn_2_i[8]      =    txn_1_i[8] ^ txn_1_i_error[8];

  assign #LATENCY9  rxp_1_i[9]      =    txp_2_i[9] ^ txp_2_i_error[9];
  assign #LATENCY9  rxn_1_i[9]      =    txn_2_i[9] ^ txn_2_i_error[9];
  assign #LATENCY9  rxp_2_i[9]      =    txp_1_i[9] ^ txp_1_i_error[9];
  assign #LATENCY9  rxn_2_i[9]      =    txn_1_i[9] ^ txn_1_i_error[9];

	// Instantiate the Unit Under Test (UUT) (2x)
	nf10_upb_interconnect uut1 (
		.axi_aclk(axi_aclk), 
		.axi_resetn(axi_resetn), 
		.arbiter_m_axis_tdata(arbiter_m_axis1_tdata), 
		.arbiter_m_axis_tkeep(arbiter_m_axis1_tkeep), 
		.arbiter_m_axis_tuser_packet_length(arbiter_m_axis1_tuser_packet_length), 
		.arbiter_m_axis_tuser_in_port(arbiter_m_axis1_tuser_in_port), 
		.arbiter_m_axis_tuser_out_port(arbiter_m_axis1_tuser_out_port), 
		.arbiter_m_axis_tuser_in_vport(arbiter_m_axis1_tuser_in_vport), 
		.arbiter_m_axis_tuser_out_vport(arbiter_m_axis1_tuser_out_vport), 
		.arbiter_m_axis_tvalid(arbiter_m_axis1_tvalid), 
		.arbiter_m_axis_tready(arbiter_m_axis1_tready), 
		.arbiter_m_axis_tlast(arbiter_m_axis1_tlast), 
		.output_queue_clk(output_queue_clk1), 
		.output_queue_s_axis_tdata(output_queue_s_axis1_tdata), 
		.output_queue_s_axis_tkeep(output_queue_s_axis1_tkeep), 
		.output_queue_s_axis_tuser_packet_length(output_queue_s_axis1_tuser_packet_length), 
		.output_queue_s_axis_tuser_in_port(output_queue_s_axis1_tuser_in_port), 
		.output_queue_s_axis_tuser_out_port(output_queue_s_axis1_tuser_out_port), 
		.output_queue_s_axis_tuser_in_vport(output_queue_s_axis1_tuser_in_vport), 
		.output_queue_s_axis_tuser_out_vport(output_queue_s_axis1_tuser_out_vport), 
		.output_queue_s_axis_tvalid(output_queue_s_axis1_tvalid), 
		.output_queue_s_axis_tready(output_queue_s_axis1_tready), 
		.output_queue_s_axis_tlast(output_queue_s_axis1_tlast), 
		.RESET(RESET), 
    .INIT_CLK(reference_clk_1_p_r),
    .GT_RESET_IN(gsr_r),
    .GTXD8_P(reference_clk_1_p_r),
    .GTXD8_N(reference_clk_1_n_r),
		.RXP(rxp_1_i), 
		.RXN(rxn_1_i), 
		.TXP(txp_1_i), 
		.TXN(txn_1_i)
	);
	nf10_upb_interconnect uut2 (
		.axi_aclk(axi_aclk), 
		.axi_resetn(axi_resetn), 
		.arbiter_m_axis_tdata(arbiter_m_axis2_tdata), 
		.arbiter_m_axis_tkeep(arbiter_m_axis2_tkeep), 
		.arbiter_m_axis_tuser_packet_length(arbiter_m_axis2_tuser_packet_length), 
		.arbiter_m_axis_tuser_in_port(arbiter_m_axis2_tuser_in_port), 
		.arbiter_m_axis_tuser_out_port(arbiter_m_axis2_tuser_out_port), 
		.arbiter_m_axis_tuser_in_vport(arbiter_m_axis2_tuser_in_vport), 
		.arbiter_m_axis_tuser_out_vport(arbiter_m_axis2_tuser_out_vport), 
		.arbiter_m_axis_tvalid(arbiter_m_axis2_tvalid), 
		.arbiter_m_axis_tready(arbiter_m_axis2_tready), 
		.arbiter_m_axis_tlast(arbiter_m_axis2_tlast), 
		.output_queue_clk(output_queue_clk2), 
		.output_queue_s_axis_tdata(output_queue_s_axis2_tdata), 
		.output_queue_s_axis_tkeep(output_queue_s_axis2_tkeep), 
		.output_queue_s_axis_tuser_packet_length(output_queue_s_axis2_tuser_packet_length), 
		.output_queue_s_axis_tuser_in_port(output_queue_s_axis2_tuser_in_port), 
		.output_queue_s_axis_tuser_out_port(output_queue_s_axis2_tuser_out_port), 
		.output_queue_s_axis_tuser_in_vport(output_queue_s_axis2_tuser_in_vport), 
		.output_queue_s_axis_tuser_out_vport(output_queue_s_axis2_tuser_out_vport), 
		.output_queue_s_axis_tvalid(output_queue_s_axis2_tvalid), 
		.output_queue_s_axis_tready(output_queue_s_axis2_tready), 
		.output_queue_s_axis_tlast(output_queue_s_axis2_tlast), 
		.RESET(RESET), 
    .INIT_CLK(reference_clk_2_p_r),
    .GT_RESET_IN(gsr_r),
    .GTXD8_P(reference_clk_2_p_r),
    .GTXD8_N(reference_clk_2_n_r),
		.RXP(rxp_2_i), 
		.RXN(rxn_2_i), 
		.TXP(txp_2_i), 
		.TXN(txn_2_i)
	);
  
  // Instantiate the frame generators and checkers  
  frame_gen gen1 (
      .clk(output_queue_clk1), 
      .axi_resetn(axi_resetn), 
      .m_axis_tdata(output_queue_s_axis1_tdata), 
      .m_axis_tkeep(output_queue_s_axis1_tkeep), 
      .m_axis_tuser_packet_length(output_queue_s_axis1_tuser_packet_length), 
      .m_axis_tuser_in_port(output_queue_s_axis1_tuser_in_port), 
      .m_axis_tuser_out_port(output_queue_s_axis1_tuser_out_port), 
      .m_axis_tuser_in_vport(output_queue_s_axis1_tuser_in_vport), 
      .m_axis_tuser_out_vport(output_queue_s_axis1_tuser_out_vport), 
      .m_axis_tvalid(output_queue_s_axis1_tvalid), 
      .m_axis_tready(output_queue_s_axis1_tready), 
      .m_axis_tlast(output_queue_s_axis1_tlast)
  );
  frame_check check1 (
      .clk(axi_aclk), 
      .axi_resetn(axi_resetn), 
      .s_axis_tdata(arbiter_m_axis2_tdata), 
      .s_axis_tkeep(arbiter_m_axis2_tkeep), 
      .s_axis_tuser_packet_length(arbiter_m_axis2_tuser_packet_length), 
      .s_axis_tuser_in_port(arbiter_m_axis2_tuser_in_port), 
      .s_axis_tuser_out_port(arbiter_m_axis2_tuser_out_port), 
      .s_axis_tuser_in_vport(arbiter_m_axis2_tuser_in_vport), 
      .s_axis_tuser_out_vport(arbiter_m_axis2_tuser_out_vport), 
      .s_axis_tvalid(arbiter_m_axis2_tvalid), 
      .s_axis_tready(arbiter_m_axis2_tready), 
      .s_axis_tlast(arbiter_m_axis2_tlast)
  );
  
  frame_gen gen2 (
      .clk(output_queue_clk2), 
      .axi_resetn(axi_resetn), 
      .m_axis_tdata(output_queue_s_axis2_tdata), 
      .m_axis_tkeep(output_queue_s_axis2_tkeep), 
      .m_axis_tuser_packet_length(output_queue_s_axis2_tuser_packet_length), 
      .m_axis_tuser_in_port(output_queue_s_axis2_tuser_in_port), 
      .m_axis_tuser_out_port(output_queue_s_axis2_tuser_out_port), 
      .m_axis_tuser_in_vport(output_queue_s_axis2_tuser_in_vport), 
      .m_axis_tuser_out_vport(output_queue_s_axis2_tuser_out_vport), 
      .m_axis_tvalid(output_queue_s_axis2_tvalid), 
      .m_axis_tready(output_queue_s_axis2_tready), 
      .m_axis_tlast(output_queue_s_axis2_tlast)
  );
  frame_check check2 (
      .clk(axi_aclk), 
      .axi_resetn(axi_resetn), 
      .s_axis_tdata(arbiter_m_axis1_tdata), 
      .s_axis_tkeep(arbiter_m_axis1_tkeep), 
      .s_axis_tuser_packet_length(arbiter_m_axis1_tuser_packet_length), 
      .s_axis_tuser_in_port(arbiter_m_axis1_tuser_in_port), 
      .s_axis_tuser_out_port(arbiter_m_axis1_tuser_out_port), 
      .s_axis_tuser_in_vport(arbiter_m_axis1_tuser_in_vport), 
      .s_axis_tuser_out_vport(arbiter_m_axis1_tuser_out_vport), 
      .s_axis_tvalid(arbiter_m_axis1_tvalid), 
      .s_axis_tready(arbiter_m_axis1_tready), 
      .s_axis_tlast(arbiter_m_axis1_tlast)
  );
  
	initial begin
		// Initialize Inputs
		axi_aclk = 0;
		axi_resetn = 0;
		RESET = 0;

		// Wait for global reset to finish
		#(100 * CLOCKPERIOD_AXI / 2);
		axi_resetn = 1;
        
		// Add stimulus here

	end
  
  
  assign glbl.GSR = gsr_r;
  assign glbl.GTS = gts_r;

  initial
    begin
      gts_r = 1'b0;        
      gsr_r = 1'b1;
      #(16*CLOCKPERIOD_1);
      gsr_r = 1'b0;
    end
  
  always
    # (CLOCKPERIOD_AXI / 2) axi_aclk <= ~axi_aclk;
    
  initial
    reference_clk_1_n_r = 1'b0;
    
  always  
    #(CLOCKPERIOD_1 / 2) reference_clk_1_n_r = !reference_clk_1_n_r;
    
  assign reference_clk_1_p_r = !reference_clk_1_n_r;
  
  initial
    reference_clk_2_n_r = 1'b0;
    
  always  
    #(CLOCKPERIOD_2 / 2) reference_clk_2_n_r = !reference_clk_2_n_r;
    
  assign reference_clk_2_p_r = !reference_clk_2_n_r;
  
  initial begin
    #(10000 * CLOCKPERIOD_1 / 2);
    @(posedge reference_clk_2_p_r) begin
      rxp_1_i_error = 10'b0010000000; 
      rxn_1_i_error = 10'b0000000000;     
      txp_1_i_error = 10'b0000001000; 
      txn_1_i_error = 10'b0100000100; 
      rxp_2_i_error = 10'b0100000000; 
      rxn_2_i_error = 10'b0000000000;     
      txp_2_i_error = 10'b0100110000; 
      txn_2_i_error = 10'b0000000111; 
    end    
    #(CLOCKPERIOD_1);
    @(posedge reference_clk_2_p_r) begin
      rxp_1_i_error = 0; 
      rxn_1_i_error = 0;     
      txp_1_i_error = 0; 
      txn_1_i_error = 0; 
      rxp_2_i_error = 0; 
      rxn_2_i_error = 0;     
      txp_2_i_error = 0; 
      txn_2_i_error = 0; 
    end
  end
      
endmodule

