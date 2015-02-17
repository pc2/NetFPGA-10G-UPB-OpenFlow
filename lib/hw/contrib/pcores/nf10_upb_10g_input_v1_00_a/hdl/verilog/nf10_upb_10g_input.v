/*
 * nf10_upb_10g_input 
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
 *
 * Description: gets input from 10G mac, stores it in PA_FIFO and forwards it 
 *              to the input arbiter
 *              backwards it takes input from the output queue and forwards it
 *              to the 10G mac
 *              on congestion a pause frame is requested
 *
 *                  --------------------     -------------
 *              --> |Asynch. Width Mul.| --> |Packet FIFO| -->
 *                  --------------------     -------------
 *                                                 |
 *                  -----------                    |
 *              <-- |Flow Ctrl| <-------------------
 *                  -----------                
 *
 *                  ---------------                    
 *              <-- |Width Divider| <--
 *                  ---------------
 */


`default_nettype none

module nf10_upb_10g_input #( 
    parameter C_PORT_NUMBER = 0,
    parameter C_INPORT_WIDTH = 3,
    parameter C_OUTPORT_WIDTH = 8,
    parameter C_PACKET_LENGTH_WIDTH = 14,
    parameter C_MAX_PACKET_LENGTH = 10000, // in byte
    parameter C_AXIS_DATA_WIDTH = 256,
    parameter [31:0] C_AXI_BASE_ADDR = 32'h00000000,
    parameter [31:0] C_AXI_HIGH_ADDR = 32'hFFFFFFFF
)
(
    // MAC side //
    input wire clk156,
    output pause_req,
    output wire [15:0] pause_val,
    
    // Master Stream Ports
    output wire [63:0] mac_m_axis_tdata,
    output wire [7:0] mac_m_axis_tkeep, 
    output wire [0:0] mac_m_axis_tuser,
    output wire mac_m_axis_tvalid,
    input wire  mac_m_axis_tready,
    output wire mac_m_axis_tlast,

    // Slave Stream Ports
    input wire [63:0] mac_s_axis_tdata,
    input wire [7:0] mac_s_axis_tkeep,
    input wire [0:0] mac_s_axis_tuser,
    input wire mac_s_axis_tvalid,
    output reg mac_s_axis_tready = 1, // always able to receive
    input wire mac_s_axis_tlast,
    
    
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
  `include "../../../nf10_upb_lib/hdl/verilog/tkeep_coder.v"

  localparam AXIS_DATA_WIDTH_BYTE = log2(C_AXIS_DATA_WIDTH/8);

  wire clk;
  wire reset;
  assign clk = axi_aclk;
  assign reset = ~axi_resetn;
  assign output_queue_clk = clk156;
  
  // wires used between the asynch. width multiplier and the packet fifo
  wire [C_AXIS_DATA_WIDTH-1:0] axis_tdata_internal;
  wire [C_AXIS_DATA_WIDTH/8-1:0] axis_tkeep_internal;
  wire [AXIS_DATA_WIDTH_BYTE-1:0] axis_tkeep_internal_enc;
  wire [0:0] axis_tuser_internal;
  wire axis_tvalid_internal;
  wire axis_tready_internal;
  wire axis_tlast_internal;
  wire conv_must_read;
  wire conv_error;
  
  assign axis_tkeep_internal_enc = encode(axis_tkeep_internal);
  
  // Instantiate the asynchronous width converter
  AsyncWidthConverter width_mul (
    .clk156(clk156), 
    .s_axis_tdata(mac_s_axis_tdata), 
    .s_axis_tkeep(mac_s_axis_tkeep), 
    .s_axis_tuser(mac_s_axis_tuser), 
    .s_axis_tvalid(mac_s_axis_tvalid), 
    .s_axis_tready(), 
    .s_axis_tlast(mac_s_axis_tlast), 
    .axi_aclk(axi_aclk), 
    .axi_resetn(axi_resetn), 
    .must_read(conv_must_read), 
    .error(conv_error),
    .m_axis_tdata(axis_tdata_internal), 
    .m_axis_tkeep(axis_tkeep_internal), 
    .m_axis_tuser(axis_tuser_internal), 
    .m_axis_tvalid(axis_tvalid_internal), 
    .m_axis_tready(axis_tready_internal), 
    .m_axis_tlast(axis_tlast_internal)
  );
  
  reg dropping = 0; // indicate, if rest of current packet is dropped
  reg [C_PACKET_LENGTH_WIDTH-1:0] pkt_length = 0;
    
  // control packet fifo input
  wire pa_fifo_full;
  wire pa_fifo_empty;
  assign arbiter_m_axis_tvalid = !pa_fifo_empty;
  wire pa_fifo_wren = !dropping && axis_tvalid_internal && (conv_must_read || !pa_fifo_full);
  assign axis_tready_internal = pa_fifo_wren || dropping;
  wire pa_fifo_commit;
  wire pa_fifo_revert;
  assign pa_fifo_commit = pa_fifo_wren && axis_tlast_internal && (axis_tuser_internal == 1'b0) && (pkt_length + {9'b0,axis_tkeep_internal_enc} + {13'b0,1'b1} <= C_MAX_PACKET_LENGTH);
  assign pa_fifo_revert = pa_fifo_wren && ((pkt_length + axis_tkeep_internal_enc + 1 > C_MAX_PACKET_LENGTH) || pa_fifo_full && conv_must_read || axis_tlast_internal && (axis_tuser_internal == 1'b1));
  wire [AXIS_DATA_WIDTH_BYTE+C_PACKET_LENGTH_WIDTH-1:0] pa_fifo_mi;
  assign pa_fifo_mi = {pkt_length + {9'b0,axis_tkeep_internal_enc} + {13'b0,1'b1}, axis_tkeep_internal_enc};
  
  // control arbiter axi stream
  wire pa_fifo_eop;
  assign arbiter_m_axis_tlast = pa_fifo_eop;
  wire [AXIS_DATA_WIDTH_BYTE+C_PACKET_LENGTH_WIDTH-1:0] pa_fifo_mo;
  wire [AXIS_DATA_WIDTH_BYTE-1:0] arbiter_m_axis_tkeep_enc = pa_fifo_mo[AXIS_DATA_WIDTH_BYTE-1:0];
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
  
  wire below_low;
  wire above_high;
  
  // Instantiate the packet aware fifo
  nf10_upb_packet_fifo #(
    .DATA_WIDTH(C_AXIS_DATA_WIDTH),
    .METADATA_WIDTH(AXIS_DATA_WIDTH_BYTE + C_PACKET_LENGTH_WIDTH),
    .DATA_DEPTH(10), // 32KB
    .METADATA_DEPTH(10),
    .LOW_THRESHOLD(256),
    .HIGH_THRESHOLD(768)
  ) pa_fifo (
    .CLK(clk),
    .DI(axis_tdata_internal),
    .MI(pa_fifo_mi),
    .RDEN(arbiter_m_axis_tready && !pa_fifo_empty), // !empty => valid
    .WREN(pa_fifo_wren),
    .COMMIT(pa_fifo_commit),
    .REVERT(pa_fifo_revert),
    .RST(reset),
    .DO(arbiter_m_axis_tdata),
    .MO(pa_fifo_mo),
    .EOP(pa_fifo_eop),
    .FULL(pa_fifo_full),
    .EMPTY(pa_fifo_empty),
    .RDERR(),
    .WRERR(),
    .BELOW_LOW(below_low),
    .ABOVE_HIGH(above_high)
  );
  
  // count pkt length
  always @(posedge clk) begin
    if (pa_fifo_commit || pa_fifo_revert)
      pkt_length <= 0;
    else if (pa_fifo_wren)
      pkt_length <= pkt_length + {9'b0,axis_tkeep_internal_enc} + {13'b0,1'b1};
  end
  
  // ctrl dropping sig
  always @(posedge clk) begin
    if (dropping && axis_tlast_internal)
      dropping <= 0;
    else if (((pkt_length + {9'b0,axis_tkeep_internal_enc} + {13'b0,1'b1} > C_MAX_PACKET_LENGTH) || conv_must_read && pa_fifo_full) && pa_fifo_wren) begin
      dropping <= !axis_tlast_internal;
    end 
  end
  
  // Instantiate the flow control module
  flow_ctrl flow_ctrl (
    .reset(reset), 
    .clk(clk), 
    .below_low(below_low), 
    .above_high(above_high), 
    .clk156(clk156), 
    .pause_req(pause_req), 
    .pause_val(pause_val)
    );
  
  // Instantiate the width divider
  width_divider #(
    .OUTPUT_WIDTH(64),
    .DIVISOR(C_AXIS_DATA_WIDTH/64)
  ) width_div (
    .clk(clk156), 
    .reset(reset), 
    .m_axis_tdata(mac_m_axis_tdata), 
    .m_axis_tkeep(mac_m_axis_tkeep), 
    .m_axis_tvalid(mac_m_axis_tvalid), 
    .m_axis_tready(mac_m_axis_tready), 
    .m_axis_tuser(mac_m_axis_tuser), 
    .m_axis_tlast(mac_m_axis_tlast), 
    .s_axis_tdata(output_queue_s_axis_tdata), 
    .s_axis_tkeep(output_queue_s_axis_tkeep), 
    .s_axis_tvalid(output_queue_s_axis_tvalid), 
    .s_axis_tready(output_queue_s_axis_tready), 
    .s_axis_tuser(0), 
    .s_axis_tlast(output_queue_s_axis_tlast)
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
	
    localparam statistics_width = 32;
    
    reg [statistics_width-1:0] stat_rx_pkts = 0;
    reg [statistics_width-1:0] stat_rx_bytes = 0;
    reg [statistics_width-1:0] stat_rx_bad_pkts = 0;
    reg [statistics_width-1:0] stat_rx_too_long_pkts = 0;
    reg [statistics_width-1:0] stat_rx_congestion_dropped_pkts = 0;
//    reg [statistics_width-1:0] stat_rx_dropped_pkts;
    
//    reg [statistics_width-1:0] stat_tx_pkts = 0;
//    reg [statistics_width-1:0] stat_tx_bytes = 0;
//    
//    reg [statistics_width-1:0] stat_pause_reqs = 0;
//    reg [statistics_width-1:0] stat_conv_errors = 0;
    
    always @(posedge clk) begin
      if (pa_fifo_commit || pa_fifo_revert)
        stat_rx_pkts <= stat_rx_pkts + 1;
      if (axis_tvalid_internal && axis_tready_internal)
        stat_rx_bytes <= stat_rx_bytes + {9'b0,axis_tkeep_internal_enc} + {13'b0,1'b1}; 
      if (pa_fifo_revert && axis_tlast_internal && axis_tuser_internal)
        stat_rx_bad_pkts <= stat_rx_bad_pkts + 1;
      if ((pkt_length + {9'b0,axis_tkeep_internal_enc} + {13'b0,1'b1} > C_MAX_PACKET_LENGTH) && pa_fifo_wren)
        stat_rx_too_long_pkts <= stat_rx_too_long_pkts + 1;
      if ((conv_must_read && pa_fifo_full) && pa_fifo_wren)
        stat_rx_congestion_dropped_pkts <= stat_rx_congestion_dropped_pkts + 1;
//      if (pa_fifo_revert)
//        stat_rx_dropped_pkts <= stat_rx_dropped_pkts + 1;
//      if (conv_error)
//        stat_conv_errors <= stat_conv_errors + 1;
    end

    reg [(C_AXIS_DATA_WIDTH/8)-1:0] output_queue_s_axis_tkeep_ff = 0;
    reg output_queue_s_axis_tvalid_tready_ff = 0;
    reg output_queue_s_axis_tlast_ff = 0;
    
//    always @(posedge clk156) begin
//
//      output_queue_s_axis_tkeep_ff <= output_queue_s_axis_tkeep;
//      output_queue_s_axis_tvalid_tready_ff <= output_queue_s_axis_tvalid & output_queue_s_axis_tready;
//      output_queue_s_axis_tlast_ff <= output_queue_s_axis_tlast;
//
//      if (output_queue_s_axis_tvalid_tready_ff && output_queue_s_axis_tlast_ff)
//        stat_tx_pkts <= stat_tx_pkts + 1;
//      if (output_queue_s_axis_tvalid_tready_ff)
//        stat_tx_bytes <= stat_tx_bytes + {9'b0, encode(output_queue_s_axis_tkeep_ff)} + {13'b0,1'b1};
//      if (pause_req)
//        stat_pause_reqs <= stat_pause_reqs + 1;
//    end
	

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
