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

module nf10_upb_interconnect #( 
    parameter C_PORT_NUMBER = 0,
    parameter C_INPORT_WIDTH = 3,
    parameter C_OUTPORT_WIDTH = 8,
    parameter C_PACKET_LENGTH_WIDTH = 14,
    parameter C_MAX_PACKET_LENGTH = 10000, // in byte
    parameter C_MIN_PACKET_LENGTH = 33, // in byte
    parameter C_AXIS_DATA_WIDTH = 256,
    parameter SIM_GTXRESET_SPEEDUP = 1,
    parameter [31:0] C_AXI_BASE_ADDR = 32'h00000000,
    parameter [31:0] C_AXI_HIGH_ADDR = 32'hFFFFFFFF 
)
(
    // input wire arbiter side //
    input wire axi_aclk,
    input wire axi_resetn,    
    
    // Master Stream Ports
    output wire [C_AXIS_DATA_WIDTH-1:0] arbiter_m_axis_tdata,
    output wire [(C_AXIS_DATA_WIDTH/8)-1:0] arbiter_m_axis_tkeep, 
    output wire [C_PACKET_LENGTH_WIDTH-1:0] arbiter_m_axis_tuser_packet_length,
    output reg [C_INPORT_WIDTH-1:0] arbiter_m_axis_tuser_in_port = C_PORT_NUMBER,
    output reg [C_OUTPORT_WIDTH-1:0] arbiter_m_axis_tuser_out_port = 0,
    output reg [C_INPORT_WIDTH-1:0] arbiter_m_axis_tuser_in_vport = 0,
    output reg [C_OUTPORT_WIDTH-1:0] arbiter_m_axis_tuser_out_vport = 0,
    output wire arbiter_m_axis_tvalid,
    input  wire arbiter_m_axis_tready,
    output wire arbiter_m_axis_tlast,
	 
    // output queue side //
    output wire output_queue_clk,
    
    // Slave Stream Ports
    input wire [C_AXIS_DATA_WIDTH-1:0] output_queue_s_axis_tdata,
    input wire [(C_AXIS_DATA_WIDTH/8)-1:0] output_queue_s_axis_tkeep,
    input wire [C_PACKET_LENGTH_WIDTH-1:0] output_queue_s_axis_tuser_packet_length,
    input wire [C_INPORT_WIDTH-1:0] output_queue_s_axis_tuser_in_port,
    input wire [C_OUTPORT_WIDTH-1:0] output_queue_s_axis_tuser_out_port,
    input wire [C_INPORT_WIDTH-1:0] output_queue_s_axis_tuser_in_vport,
    input wire [C_OUTPORT_WIDTH-1:0] output_queue_s_axis_tuser_out_vport,
    input wire output_queue_s_axis_tvalid,
    output wire output_queue_s_axis_tready,
    input wire output_queue_s_axis_tlast,

    input wire RESET, 
    input wire INIT_CLK,
    input wire GT_RESET_IN,
    input wire GTXD8_P,
    input wire GTXD8_N,
    input wire [0:9] RXP,
    input wire [0:9] RXN,
    output wire [0:9] TXP,
    output wire [0:9] TXN,

    // AXI4Lite for reading statistics
    input wire s_axi_aclk,
    input wire s_axi_aresetn,
    input wire s_axi_arvalid,
    output reg s_axi_arready,
    input wire [31:0] s_axi_araddr,
    input wire [2:0] s_axi_arprot,
    output reg s_axi_rvalid,
    input wire s_axi_rready,
    output reg [31:0] s_axi_rdata,
    output reg [1:0] s_axi_rresp
    );
  `include "../../../../nf10_upb_lib/hdl/verilog/tkeep_coder.v"
  
  localparam AXIS_DATA_WIDTH_BYTE = log2(C_AXIS_DATA_WIDTH/8);
  
  wire axi_reset = !axi_resetn;
  wire clk = axi_aclk;  

  // pa fifo watermarks and flow ctrl signal
  wire pa_fifo_below_low;
  wire pa_fifo_above_high;
  wire flow_control_pause_req;
  reg pa_fifo_below_low_reg = 1;
  reg pa_fifo_above_high_reg = 0;
  reg pa_fifo_below_low_reg_delayed = 1;
  reg pa_fifo_above_high_reg_delayed = 0;
  
  // aurora output signals
  wire [C_AXIS_DATA_WIDTH-1:0] aurora_data;
  wire [(C_AXIS_DATA_WIDTH/8)-1:0] aurora_keep;
  wire [C_PACKET_LENGTH_WIDTH-1:0] aurora_pkt_length;
  wire aurora_valid;
  wire aurora_last;
  wire aurora_error;
  wire [AXIS_DATA_WIDTH_BYTE-1:0] aurora_keep_enc;
  assign aurora_keep_enc = encode(aurora_keep);
  
  // wide fifo input side signals
  wire [31:0] wide_fifo_dip;
  assign wide_fifo_dip = {11'b0, aurora_pkt_length, aurora_keep_enc, aurora_error, aurora_last};
  
  // wide fifo output side signals
  wire wide_fifo_almost_empty;
  wire wide_fifo_almost_full;
  wire wide_fifo_empty;
  wire wide_fifo_full;  
  wire [31:0] wide_fifo_dop;
  wire wide_fifo_error;
  wire [C_PACKET_LENGTH_WIDTH-1:0] wide_fifo_pkt_length;
  wire wide_fifo_last;
  wire [AXIS_DATA_WIDTH_BYTE-1:0] wide_fifo_keep_enc;
  assign {wide_fifo_pkt_length, wide_fifo_keep_enc, wide_fifo_error, wide_fifo_last} = wide_fifo_dop[AXIS_DATA_WIDTH_BYTE+C_PACKET_LENGTH_WIDTH+1:0];

  reg wide_fifo_almost_full_delayed;
  always @(posedge output_queue_clk) begin
    wide_fifo_almost_full_delayed <= wide_fifo_almost_full;
  end

  // Instantiate the aurora core wrapping module (incl. crc)
  nf10_upb_aurora_input aurora_input (
    .axi_resetn(axi_resetn), 
    .m_axis_tdata(aurora_data), 
    .m_axis_tkeep(aurora_keep), 
    .m_axis_tuser_packet_length(aurora_pkt_length), 
    .m_axis_tvalid(aurora_valid), 
    .m_axis_tlast(aurora_last), 
    .error(aurora_error), 
    .clk156(output_queue_clk), 
    .s_axis_tdata(output_queue_s_axis_tdata), 
    .s_axis_tkeep(output_queue_s_axis_tkeep), 
    .s_axis_tuser_packet_length(output_queue_s_axis_tuser_packet_length), 
    .s_axis_tvalid(output_queue_s_axis_tvalid), 
    .s_axis_tready(output_queue_s_axis_tready), 
    .s_axis_tlast(output_queue_s_axis_tlast), 
    .flow_control_pause_req(wide_fifo_almost_full_delayed != wide_fifo_almost_full), 
    .flow_control_pause_val(wide_fifo_almost_full), 
    .RESET(RESET), 
    .INIT_CLK(INIT_CLK), 
    .GT_RESET_IN(GT_RESET_IN), 
    .GTXD8_P(GTXD8_P), 
    .GTXD8_N(GTXD8_N), 
    .RXP(RXP), 
    .RXN(RXN), 
    .TXP(TXP), 
    .TXN(TXN)
  );
  
  // control signals
  reg dropping = 0; // indicate, if rest of current packet is dropped
  reg [C_PACKET_LENGTH_WIDTH-1:0] pkt_length = 0;
  wire [C_PACKET_LENGTH_WIDTH-1:0] next_pkt_length;
  wire length_check_ok = !(next_pkt_length > C_MAX_PACKET_LENGTH) && !(wide_fifo_last && next_pkt_length < C_MIN_PACKET_LENGTH);
  wire pa_fifo_full;
  wire pa_fifo_empty;
  wire pa_fifo_eop;
  assign next_pkt_length = pkt_length + {9'b0,wide_fifo_keep_enc} + {13'b0,1'b1};
  
  // control packet fifo input
  wire pa_fifo_commit;
  wire pa_fifo_revert;
  wire pa_fifo_wren;
  wire [AXIS_DATA_WIDTH_BYTE+C_PACKET_LENGTH_WIDTH-1:0] pa_fifo_mi; 
  wire [C_AXIS_DATA_WIDTH-1:0] pa_fifo_di;
  assign pa_fifo_wren = !dropping && !wide_fifo_empty && (!wide_fifo_almost_empty || !pa_fifo_full);
  assign pa_fifo_commit = pa_fifo_wren && wide_fifo_last && length_check_ok && !wide_fifo_error;
  assign pa_fifo_revert = pa_fifo_wren && (!length_check_ok || pa_fifo_full && !wide_fifo_almost_empty || wide_fifo_error);
  assign pa_fifo_mi = {next_pkt_length, wide_fifo_keep_enc}; 
  
  // control arbiter axi stream
  wire [AXIS_DATA_WIDTH_BYTE+C_PACKET_LENGTH_WIDTH-1:0] pa_fifo_mo;
  wire [AXIS_DATA_WIDTH_BYTE-1:0] arbiter_m_axis_tkeep_enc = pa_fifo_mo[AXIS_DATA_WIDTH_BYTE-1:0];
  assign arbiter_m_axis_tvalid = !pa_fifo_empty;
  assign arbiter_m_axis_tlast = pa_fifo_eop;
  assign arbiter_m_axis_tkeep = get_arbiter_tkeep(arbiter_m_axis_tkeep_enc, arbiter_m_axis_tlast); 
  assign arbiter_m_axis_tuser_packet_length = pa_fifo_mo[AXIS_DATA_WIDTH_BYTE+C_PACKET_LENGTH_WIDTH-1-:C_PACKET_LENGTH_WIDTH];  
  
  // check whether metadata info has to be used for arbiter tkeep
  function [(C_AXIS_DATA_WIDTH/8)-1:0] get_arbiter_tkeep;
  input [AXIS_DATA_WIDTH_BYTE-1:0] arbiter_m_axis_tkeep_enc;
  input arbiter_m_axis_tlast;
    if (arbiter_m_axis_tlast)
      get_arbiter_tkeep = decode(arbiter_m_axis_tkeep_enc);
    else
      get_arbiter_tkeep = {(C_AXIS_DATA_WIDTH/8){1'b1}};
  endfunction

  wire wide_fifo_rderr, wide_fifo_wrerr, pa_fifo_rderr, pa_fifo_wrerr;
   
  // instantiate an async fifo to change clock
  wide_fifo #(
   	.C_SIM_MODE("FAST"),
   	.C_ALMOST_FULL_OFFSET(13'd400), 
   	.C_ALMOST_EMPTY_OFFSET(13'd500), 
   	.C_EN_ECC_READ("FALSE"),
   	.C_EN_ECC_WRITE("FALSE"),
   	.C_EN_SYN("FALSE"),
   	.C_FIRST_WORD_FALL_THROUGH("TRUE"),
    .C_NUMBER_FIFOS(4)
  ) wide_fifo (
    .ALMOSTEMPTY(wide_fifo_almost_empty), 
    .ALMOSTFULL(wide_fifo_almost_full), 
    .DO(pa_fifo_di), 
    .DOP(wide_fifo_dop), 
    .EMPTY(wide_fifo_empty), 
    .FULL(wide_fifo_full), 
	.RDERR(wide_fifo_rderr),
    .WRERR(wide_fifo_wrerr),
    .DI(aurora_data), 
    .DIP(wide_fifo_dip), 
    .RDCLK(clk), 
    .RDEN(pa_fifo_wren || (dropping && !wide_fifo_empty)), 
    .RST(axi_reset), 
    .WRCLK(output_queue_clk), // from aurora
    .WREN(aurora_valid && !wide_fifo_full)
    );
  
  // Instantiate the packet aware fifo
  nf10_upb_packet_fifo #(
    .DATA_WIDTH(C_AXIS_DATA_WIDTH),
    .METADATA_WIDTH(AXIS_DATA_WIDTH_BYTE + C_PACKET_LENGTH_WIDTH),
    .DATA_DEPTH(9), // 16KB
    .METADATA_DEPTH(10),
    .LOW_THRESHOLD(100),
    .HIGH_THRESHOLD(400)
  ) pa_fifo (
    .CLK(clk),
    .DI(pa_fifo_di),
    .MI(pa_fifo_mi),
    .RDEN(arbiter_m_axis_tready && !pa_fifo_empty), // !empty => valid
    .WREN(pa_fifo_wren),
    .COMMIT(pa_fifo_commit),
    .REVERT(pa_fifo_revert),
    .RST(axi_reset),
    .DO(arbiter_m_axis_tdata),
    .MO(pa_fifo_mo),
    .EOP(pa_fifo_eop),
    .FULL(pa_fifo_full),
    .EMPTY(pa_fifo_empty),
    .RDERR(pa_fifo_rderr),
    .WRERR(pa_fifo_wrerr),
    .BELOW_LOW(pa_fifo_below_low),
    .ABOVE_HIGH(pa_fifo_above_high)
  );
  
  // count pkt length
  always @(posedge clk) begin
    if (pa_fifo_commit || pa_fifo_revert || axi_reset)
      pkt_length <= 0;
    else if (pa_fifo_wren)
      pkt_length <= next_pkt_length;
  end
  
  // ctrl dropping sig
  always @(posedge clk) begin
    if (dropping && wide_fifo_last || axi_reset)
      dropping <= 0;
    else if ((!length_check_ok || !wide_fifo_almost_empty && pa_fifo_full || wide_fifo_error) && pa_fifo_wren) begin
      dropping <= !wide_fifo_last;
    end 
  end

  // store packet fifo watermarks
  always @(posedge clk) begin
    if (!axi_reset) begin
  	pa_fifo_below_low_reg <= pa_fifo_below_low;
  	pa_fifo_above_high_reg <= pa_fifo_above_high;
  	pa_fifo_below_low_reg_delayed <= pa_fifo_below_low_reg;
  	pa_fifo_above_high_reg_delayed <= pa_fifo_above_high_reg;
    end else begin
  	pa_fifo_below_low_reg <= 1;
  	pa_fifo_above_high_reg <= 0;
  	pa_fifo_below_low_reg_delayed <= 1;
  	pa_fifo_above_high_reg_delayed <= 0;
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


  // collect statistics:
  localparam statistics_width = 32;
    
  reg [statistics_width-1:0] stat_rx_pkts = 0;
  reg [statistics_width-1:0] stat_rx_bytes = 0;
  reg [statistics_width-1:0] stat_rx_bad_pkts = 0;
  reg [statistics_width-1:0] stat_rx_too_long_pkts = 0;
  reg [statistics_width-1:0] stat_rx_congestion_dropped_pkts = 0;

  // delay signals for timing
  reg pa_fifo_commit_ff;
  reg pa_fifo_revert_ff;
  reg pa_fifo_wren_ff;
  reg pa_fifo_full_ff;
  reg dropping_ff;
  reg wide_fifo_error_ff;
  reg length_check_ok_ff;
  reg wide_fifo_almost_empty_ff;
  reg [AXIS_DATA_WIDTH_BYTE-1:0] wide_fifo_keep_enc_ff;
  reg output_queue_s_axis_tvalid_ff;
  reg output_queue_s_axis_tready_ff;
  reg output_queue_s_axis_tlast_ff;
  reg [(C_AXIS_DATA_WIDTH/8)-1:0] output_queue_s_axis_tkeep_ff;


  always @(posedge clk) begin
    pa_fifo_commit_ff <= pa_fifo_commit;
    pa_fifo_revert_ff <= pa_fifo_revert;
    pa_fifo_wren_ff <= pa_fifo_wren;
    pa_fifo_full_ff <= pa_fifo_full;
    dropping_ff <= dropping;
    wide_fifo_error_ff <= wide_fifo_error;
    length_check_ok_ff <= length_check_ok;
    wide_fifo_almost_empty_ff <= wide_fifo_almost_empty;
    wide_fifo_keep_enc_ff <= wide_fifo_keep_enc;
    output_queue_s_axis_tvalid_ff <= output_queue_s_axis_tvalid;
    output_queue_s_axis_tready_ff <= output_queue_s_axis_tready;
    output_queue_s_axis_tlast_ff <= output_queue_s_axis_tlast;
    output_queue_s_axis_tkeep_ff <= output_queue_s_axis_tkeep;

    if (pa_fifo_commit_ff || pa_fifo_revert_ff)
      stat_rx_pkts <= stat_rx_pkts + 1;
    if (pa_fifo_wren_ff || dropping_ff)
      stat_rx_bytes <= stat_rx_bytes + {9'b0,wide_fifo_keep_enc_ff} + {13'b0,1'b1}; 
    if (pa_fifo_revert_ff && wide_fifo_error_ff)
      stat_rx_bad_pkts <= stat_rx_bad_pkts + 1;
    if (pa_fifo_revert_ff && !length_check_ok_ff)
      stat_rx_too_long_pkts <= stat_rx_too_long_pkts + 1;
    if (pa_fifo_revert_ff && pa_fifo_full_ff && !wide_fifo_almost_empty_ff)
      stat_rx_congestion_dropped_pkts <= stat_rx_congestion_dropped_pkts + 1;
  end

        // AXI4Lite interface for statistics
        localparam RD_STATE_WAITING = 0, RD_STATE_PROCESSING = 1, RD_STATE_ANSWERING = 2;
        reg [1:0] rd_state = RD_STATE_WAITING;
        reg [1:0] rd_state_next;
        reg [31:0] rd_addr, rd_addr_next, rdata_next;
        wire s_axi_araddr_in_range = (s_axi_araddr & (C_AXI_BASE_ADDR ~^ C_AXI_HIGH_ADDR)) == C_AXI_BASE_ADDR;
        wire [31:0] s_axi_araddr_masked_32b_aligned = (s_axi_araddr & (C_AXI_BASE_ADDR ^ C_AXI_HIGH_ADDR)) >> 2; // reduce address lines from byte adressing to (32 bit) word adressing

        always @(posedge s_axi_aclk) begin
                rd_state <= rd_state_next;
                rd_addr <= rd_addr_next;
                s_axi_rdata <= rdata_next;
        end

        always @(*) begin
                s_axi_arready = 0;
                s_axi_rvalid = 0;
                s_axi_rresp = 0; // OK

                rdata_next = s_axi_rdata;
                rd_addr_next = rd_addr;
                rd_state_next = rd_state;

                case (rd_state)
                        RD_STATE_WAITING: begin
                                s_axi_arready = 1;
                                if (s_axi_arvalid && s_axi_araddr_in_range) begin
                                        rd_state_next = RD_STATE_PROCESSING;
                                        rd_addr_next = s_axi_araddr_masked_32b_aligned;
                                end
                        end

                        RD_STATE_PROCESSING: begin
                                rd_state_next = RD_STATE_ANSWERING;
                                case (rd_addr)
                                    0: begin
                                        rdata_next = stat_rx_pkts;
                                    end
                                    1: begin
                                        rdata_next = stat_rx_bytes;
                                    end
                                    2: begin
                                        rdata_next = stat_rx_bad_pkts;
                                    end
                                    3: begin
                                        rdata_next = stat_rx_too_long_pkts;
                                    end
                                    4: begin
                                        rdata_next = stat_rx_congestion_dropped_pkts;
                                    end
                                    default: begin
                                        rdata_next = 32'h80000000; // error signal
                                    end
                                endcase
                        end

                        RD_STATE_ANSWERING: begin
                                s_axi_rvalid = 1;
                                if (s_axi_rready)
                                        rd_state_next = RD_STATE_WAITING;
                        end
                endcase

                if (~s_axi_aresetn) begin
                        rd_state_next = RD_STATE_WAITING;
                end
        end
  
endmodule
