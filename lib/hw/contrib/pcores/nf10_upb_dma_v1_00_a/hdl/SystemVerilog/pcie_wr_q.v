/*******************************************************************************
 *
 *  NetFPGA-10G http://www.netfpga.org
 *
 *  File:
 *        pcie_wr_q.v
 *
 *  Library:
 *        hw/contrib/pcores/dma_v1_00_a
 *
 *  Module:
 *        dma
 *
 *  Author:
 *        Mario Flajslik
 *
 *        Modifications for the UPB OpenFlow Switch project:
 *         Jörg Niklas, osjsn@niklasfamily.de
 *         Project Group "On-the-Fly Networking for Big Data"
 *         Computer Engineering Group, University of Paderborn
 *
 *  Description:
 *        PCIe write queue and packet splitter. PCIe write requests (aka DMA
 *        writes) are put in the queue in a custom format. They are then
 *        split according to PCIe rules (maximum payload size) and each new
 *        packet is driven to the pcie_tx_wr module.
 *
 *  Copyright notice:
 *        Copyright (C) 2010, 2011 The Board of Trustees of The Leland Stanford
 *                                 Junior University
 *
 *        Modifications for the UPB OpenFlow Switch project:
 *          Copyright (c) 2014, 2015 Jörg Niklas, osjsn@niklasfamily.de
 *
 *  Licence:
 *        This file is part of the NetFPGA 10G development base package.
 *
 *        This file is free code: you can redistribute it and/or modify it under
 *        the terms of the GNU Lesser General Public License version 2.1 as
 *        published by the Free Software Foundation.
 *
 *        This package is distributed in the hope that it will be useful, but
 *        WITHOUT ANY WARRANTY; without even the implied warranty of
 *        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *        Lesser General Public License for more details.
 *
 *        You should have received a copy of the GNU Lesser General Public
 *        License along with the NetFPGA source package.  If not, see
 *        http://www.gnu.org/licenses/.
 *
 */

`include "dma_defs.vh"

module pcie_wr_q #(parameter IFACE_ID = 0)
  (
   // rx_ctrl interface
   input logic                         wr_q_enq_en,
   input logic [`WR_Q_WIDTH-1:0]       wr_q_enq_data,
   output logic                        wr_q_almost_full,
   output logic                        wr_q_full,

   // pcie_tx interface
   output logic                        wr_q_req_v,
   output logic [`PCIE_WR_Q_WIDTH-1:0] wr_q_req_data,
   input logic                         wr_q_req_grant,

   // pcie config
   input logic [9:0]                   max_payload_decoded,
    
   // misc
   input logic                         clk,
   input logic                         rst
   );
                                       
   // -----------------------------------
   // -- Write Queue
   // -----------------------------------
   // [90+:`MEM_ADDR_BITS] address
   // [89:26] host_address
   // [25:22] mem_select
   // [21:6]  byte_length
   // [5:1]   interrupt_di
   // [0]     '0' - write, '1' - interrupt
   
   logic                        wr_q_deq_en;
   logic [`WR_Q_WIDTH-1:0]      wr_q_deq_data;
   logic                        wr_q_empty;
   
   fifo #(.WIDTH(`WR_Q_WIDTH), .DEPTH(`WR_Q_DEPTH), .ALMOST_FULL(5)) 
   u_wr_q (
           .enq_en(wr_q_enq_en),
           .enq_data(wr_q_enq_data),
           .deq_en(wr_q_deq_en),
           .deq_data(wr_q_deq_data),
           .empty(wr_q_empty),
           .almost_full(wr_q_almost_full),
           .full(wr_q_full),
           .enq_clk(clk),
           .deq_clk(clk),
           .rst(rst));

   localparam WR_STATE_IDLE      = 0;
   localparam WR_STATE_START     = 1;
   localparam WR_STATE_BODY_PREP = 2;
   localparam WR_STATE_BODY      = 3;

   logic [1:0]                  wr_state, wr_state_nxt;

   logic                        wr_q_req_v_nxt;
   logic [`PCIE_WR_Q_WIDTH-1:0] wr_q_req_data_nxt;

   logic [3:0]                  wr_lastBE_reg,    wr_lastBE_reg_nxt;
   logic [13:0]                 wr_dw_len_reg,    wr_dw_len_reg_nxt; // increased width to 14 bits to support Jumbo Frames
   logic [`MEM_ADDR_BITS-1:0]   wr_addr_reg,      wr_addr_reg_nxt;
   logic [63:0]                 wr_host_addr_reg, wr_host_addr_reg_nxt;

   logic [9:0]                  wr_local_dw_size, wr_local_dw_size_nxt;
   
   always_comb begin
      logic [15:0]                 wr_local_byte_size;
      
      wr_state_nxt = wr_state;

      wr_q_req_data_nxt = wr_q_req_data;

      if(wr_q_req_grant)
        wr_q_req_v_nxt = 0;
      else
        wr_q_req_v_nxt = wr_q_req_v;
      
      wr_lastBE_reg_nxt    = wr_lastBE_reg;
      wr_dw_len_reg_nxt    = wr_dw_len_reg;
      wr_addr_reg_nxt      = wr_addr_reg;
      wr_host_addr_reg_nxt = wr_host_addr_reg;

      wr_local_dw_size_nxt = wr_local_dw_size;
      
      wr_q_deq_en = 0;
      
      case(wr_state)
        WR_STATE_IDLE: begin
           if(~wr_q_empty & (~wr_q_req_v | wr_q_req_grant)) begin
              if(wr_q_deq_data[0]) begin // interrupt
                 wr_q_req_data_nxt[0] = wr_q_deq_data[0];
                 wr_q_req_data_nxt[5:1] = wr_q_deq_data[5:1];
                 wr_q_req_data_nxt[`PCIE_WR_Q_WIDTH-1:6] = 0;
                 
                 wr_q_req_v_nxt = 1;
                 wr_q_deq_en = 1;
              end
              else begin // data write

                 wr_local_byte_size = wr_q_deq_data[21:6] + {14'b0, wr_q_deq_data[91:90]};
                 
                 if(wr_local_byte_size > {4'b0, max_payload_decoded, 2'b0}) begin // write bigger than max_payload
                    if((({1'b0, wr_q_deq_data[37:28]} + {1'b0, max_payload_decoded}) & 11'h400) != 0) begin // 4k hit (writing max payload would cross a 4k boundary in host address space)
                       wr_local_dw_size_nxt = 10'd1024 - wr_q_deq_data[37:28];
                    end
                    else begin // 4k ok
                       wr_local_dw_size_nxt = max_payload_decoded - ((max_payload_decoded - 10'd1) & wr_q_deq_data[37:28]);
                    end
                 end
                 else begin // smaller than max_payload
                    if(({1'b0, wr_q_deq_data[37:26]} + {1'b0, wr_q_deq_data[17:6]}) > 13'h1000) begin // 4k hit
                       wr_local_dw_size_nxt = 10'd1024 - wr_q_deq_data[37:28];
                    end
                    else begin // 4k ok
                       if(wr_local_byte_size[1:0] == 2'b00)
                         wr_local_dw_size_nxt = wr_local_byte_size[11:2];
                       else
                         wr_local_dw_size_nxt = wr_local_byte_size[11:2] + 1;
                    end                                   
                 end
                 
                 wr_state_nxt = WR_STATE_START;

              end
           end
        end
        WR_STATE_START: begin
           if(~wr_q_empty & (~wr_q_req_v | wr_q_req_grant)) begin
              wr_q_req_data_nxt[94+:`MEM_ADDR_BITS] = wr_q_deq_data[90+:`MEM_ADDR_BITS];
              wr_q_req_data_nxt[93:92] = IFACE_ID[1:0];
              wr_q_req_data_nxt[91:88] = wr_q_deq_data[25:22];
              wr_q_req_data_nxt[87:24] = wr_q_deq_data[89:26];
              wr_q_req_data_nxt[5:0] = 0;

              wr_local_byte_size = wr_q_deq_data[21:6] + {14'b0, wr_q_deq_data[91:90]};
              
              if(wr_local_byte_size > {4'b0, max_payload_decoded, 2'b0}) begin // write bigger than max_payload
                 // lastDW BE
                 if(wr_local_dw_size == 1)
                   wr_q_req_data_nxt[9:6] = 4'b0000;
                 else
                   wr_q_req_data_nxt[9:6] = 4'b1111;                    
                 wr_state_nxt = WR_STATE_BODY_PREP;
              end
              else begin // smaller than max_payload
                 if(({1'b0, wr_q_deq_data[37:26]} + {1'b0, wr_q_deq_data[17:6]}) > 13'h1000) begin // 4k hit
                    // lastDW BE
                    if(wr_local_dw_size == 1)
                      wr_q_req_data_nxt[9:6] = 4'b0000;
                    else
                      wr_q_req_data_nxt[9:6] = 4'b1111;
                    wr_state_nxt = WR_STATE_BODY_PREP;
                 end
                 else begin // 4k ok
                    // lastDW BE
                    if(wr_local_dw_size == 1)
                      wr_q_req_data_nxt[9:6] = 4'b0000;
                    else
                      case(wr_q_deq_data[91:90] + wr_q_deq_data[7:6])
                        2'b00: wr_q_req_data_nxt[9:6] = 4'b1111;
                        2'b01: wr_q_req_data_nxt[9:6] = 4'b0001;
                        2'b10: wr_q_req_data_nxt[9:6] = 4'b0011;
                        2'b11: wr_q_req_data_nxt[9:6] = 4'b0111;
                      endcase
                    wr_state_nxt = WR_STATE_IDLE;
                 end                                   
              end
              wr_q_req_data_nxt[23:14] = wr_local_dw_size;

              // first BE
              if(wr_q_deq_data[21:6] == 16'd1)
                case(wr_q_deq_data[91:90])
                  2'b00: wr_q_req_data_nxt[13:10] = 4'b0001;
                  2'b01: wr_q_req_data_nxt[13:10] = 4'b0010;
                  2'b10: wr_q_req_data_nxt[13:10] = 4'b0100;
                  2'b11: wr_q_req_data_nxt[13:10] = 4'b1000;
                endcase
              else if(wr_q_deq_data[21:6] == 16'd2)
                case(wr_q_deq_data[91:90])
                  2'b00: wr_q_req_data_nxt[13:10] = 4'b0011;
                  2'b01: wr_q_req_data_nxt[13:10] = 4'b0110;
                  2'b10: wr_q_req_data_nxt[13:10] = 4'b1100;
                  default: wr_q_req_data_nxt[13:10] = 4'b0000;
                endcase
              else if(wr_q_deq_data[21:6] == 16'd3)
                case(wr_q_deq_data[91:90])
                  2'b00: wr_q_req_data_nxt[13:10] = 4'b0111;
                  2'b01: wr_q_req_data_nxt[13:10] = 4'b1110;
                  default: wr_q_req_data_nxt[13:10] = 4'b0000;
                endcase
              else if(wr_q_deq_data[21:6] == 16'd0)
                case(wr_q_deq_data[91:90])
                  2'b00: wr_q_req_data_nxt[13:10] = 4'b1111;
                  default: wr_q_req_data_nxt[13:10] = 4'b0000;
                endcase
              else
                case(wr_q_deq_data[91:90])
                  2'b00: wr_q_req_data_nxt[13:10] = 4'b1111;
                  2'b01: wr_q_req_data_nxt[13:10] = 4'b1110;
                  2'b10: wr_q_req_data_nxt[13:10] = 4'b1100;
                  2'b11: wr_q_req_data_nxt[13:10] = 4'b1000;
                endcase
              
              // last BE
              case(wr_q_deq_data[91:90] + wr_q_deq_data[7:6])
                2'b00: wr_lastBE_reg_nxt = 4'b1111;
                2'b01: wr_lastBE_reg_nxt = 4'b0001;
                2'b10: wr_lastBE_reg_nxt = 4'b0011;
                2'b11: wr_lastBE_reg_nxt = 4'b0111;
              endcase

              if(wr_local_byte_size[1:0] == 2'b00)
                wr_dw_len_reg_nxt = wr_local_byte_size[15:2] - wr_local_dw_size;
              else
                wr_dw_len_reg_nxt = wr_local_byte_size[15:2] - wr_local_dw_size + 1;
              
              wr_addr_reg_nxt = wr_q_deq_data[90+:`MEM_ADDR_BITS] + {{(`MEM_ADDR_BITS-12){1'b0}}, wr_local_dw_size, 2'b0};
              wr_host_addr_reg_nxt = wr_q_deq_data[89:26] + {52'b0, wr_local_dw_size, 2'b0};
              
              wr_q_req_v_nxt = 1;
              wr_q_deq_en = 1;                 
           end
        end
        WR_STATE_BODY_PREP: begin

           if(wr_dw_len_reg > max_payload_decoded) begin // write bigger than max_payload
              if((({1'b0, wr_host_addr_reg[11:2]} + {1'b0, max_payload_decoded}) & 11'h400) != 0) begin // 4k hit
                 wr_local_dw_size_nxt = 10'd1024 - wr_host_addr_reg[11:2];
              end
              else begin // 4k ok
                 wr_local_dw_size_nxt = max_payload_decoded - ((max_payload_decoded - 10'd1) & wr_host_addr_reg[11:2]);
              end
           end
           else begin // smaller than max_payload
              if(({1'b0, wr_host_addr_reg[11:2]} + {1'b0, wr_dw_len_reg[9:0]}) > 11'h400) begin // 4k hit
                 wr_local_dw_size_nxt = 10'd1024 - wr_host_addr_reg[11:2];
              end
              else begin // 4k ok
                 wr_local_dw_size_nxt = wr_dw_len_reg;
              end                                   
           end

           wr_state_nxt = WR_STATE_BODY;

        end
        WR_STATE_BODY: begin
           if(~wr_q_req_v | wr_q_req_grant) begin

              wr_q_req_data_nxt[94+:`MEM_ADDR_BITS] = wr_addr_reg;
              wr_q_req_data_nxt[87:24] = wr_host_addr_reg;
              wr_q_req_data_nxt[5:0] = 0;
              
              if(wr_dw_len_reg > max_payload_decoded) begin // write bigger than max_payload
                 // lastDW BE
                 if(wr_local_dw_size == 1)
                   wr_q_req_data_nxt[9:6] = 4'b0000;
                 else
                   wr_q_req_data_nxt[9:6] = 4'b1111;                    
                 wr_state_nxt = WR_STATE_BODY_PREP;
              end
              else begin // smaller than max_payload
                 if(({1'b0, wr_host_addr_reg[11:2]} + {1'b0, wr_dw_len_reg[9:0]}) > 11'h400) begin // 4k hit
                    // lastDW BE
                    if(wr_local_dw_size == 1)
                      wr_q_req_data_nxt[9:6] = 4'b0000;
                    else
                      wr_q_req_data_nxt[9:6] = 4'b1111;
                    wr_state_nxt = WR_STATE_BODY_PREP;
                 end
                 else begin // 4k ok
                    wr_state_nxt = WR_STATE_IDLE;
                    // lastDW BE
                    if(wr_local_dw_size == 1)
                      wr_q_req_data_nxt[9:6] = 4'b0000;
                    else
                      wr_q_req_data_nxt[9:6] = wr_lastBE_reg;
                 end                                   
              end
              
              wr_q_req_data_nxt[23:14] = wr_local_dw_size;

              // first BE
              if(wr_local_dw_size == 1)
                wr_q_req_data_nxt[13:10] = wr_lastBE_reg;
              else
                wr_q_req_data_nxt[13:10] = 4'b1111;

              wr_dw_len_reg_nxt = wr_dw_len_reg - wr_local_dw_size;
              wr_addr_reg_nxt = wr_addr_reg + {{(`MEM_ADDR_BITS-12){1'b0}}, wr_local_dw_size, 2'b0};
              wr_host_addr_reg_nxt = wr_host_addr_reg + {52'b0, wr_local_dw_size, 2'b0};
              
              wr_q_req_v_nxt = 1;
              
           end
        end
      endcase      
   end
   
   always_ff @(posedge clk) begin
      if(rst) begin
         wr_state   <= WR_STATE_IDLE;
         wr_q_req_v <= 0;
      end
      else begin
         wr_state   <= wr_state_nxt;
         wr_q_req_v <= wr_q_req_v_nxt;
      end
      wr_q_req_data <= wr_q_req_data_nxt;

      wr_local_dw_size <= wr_local_dw_size_nxt;
      
      wr_lastBE_reg    <= wr_lastBE_reg_nxt;
      wr_dw_len_reg    <= wr_dw_len_reg_nxt;
      wr_addr_reg      <= wr_addr_reg_nxt;
      wr_host_addr_reg <= wr_host_addr_reg_nxt;
   end

endmodule