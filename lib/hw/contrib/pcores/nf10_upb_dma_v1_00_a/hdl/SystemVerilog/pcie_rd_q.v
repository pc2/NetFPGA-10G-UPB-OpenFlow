/*******************************************************************************
 *
 *  NetFPGA-10G http://www.netfpga.org
 *
 *  File:
 *        pcie_rd_q.v
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
 *        PCIe read queue and packet splitter. PCIe read requests (aka DMA
 *        reads) are put in the queue in a custom format. They are then
 *        split according to PCIe rules (maximum read request size) and each new
 *        packet is driven to the pcie_tx_rd module.
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

module pcie_rd_q #(parameter IFACE_ID = 0)
  (
   // tx_ctrl interface
   input logic                         rd_q_enq_en,
   input logic [`RD_Q_WIDTH-1:0]       rd_q_enq_data,
   output logic                        rd_q_full,

   // pcie_tx queue interface
   output logic                        rd_q_req_v,
   output logic [`PCIE_RD_Q_WIDTH-1:0] rd_q_req_data,
   input logic                         rd_q_req_grant,

   // pcie config
   input logic [9:0]                   max_read_decoded,

   // misc
   input logic                         clk,
   input logic                         rst
   );
  
   // -----------------------------------
   // -- Read Queue
   // -----------------------------------
   // [84+:`MEM_ADDR_BITS] address
   // [83:20] host_address
   // [19:16] mem_select
   // [15:0]  byte_length

   logic                   rd_q_deq_en;
   logic [`RD_Q_WIDTH-1:0] rd_q_deq_data;
   logic                   rd_q_empty;

   fifo #(.WIDTH(`RD_Q_WIDTH), .DEPTH(`RD_Q_DEPTH)) 
   u_rd_q (
           .enq_en(rd_q_enq_en),
           .enq_data(rd_q_enq_data),
           .deq_en(rd_q_deq_en),
           .deq_data(rd_q_deq_data),
           .empty(rd_q_empty),
           .almost_full(),
           .full(rd_q_full),
           .enq_clk(clk),
           .deq_clk(clk),
           .rst(rst));

   localparam RD_STATE_IDLE      = 0;
   localparam RD_STATE_START     = 1;
   localparam RD_STATE_BODY_PREP = 2;
   localparam RD_STATE_BODY      = 3;

   logic [1:0]                  rd_state, rd_state_nxt;
   
   logic                        rd_q_req_v_nxt;
   logic [`PCIE_RD_Q_WIDTH-1:0] rd_q_req_data_nxt;

   logic [3:0]                  rd_lastBE_reg,    rd_lastBE_reg_nxt;
   logic [13:0]                  rd_dw_len_reg,    rd_dw_len_reg_nxt; // increased to 14 bits to support Jumbo Frames
   logic [`MEM_ADDR_BITS-1:0]   rd_addr_reg,      rd_addr_reg_nxt;
   logic [63:0]                 rd_host_addr_reg, rd_host_addr_reg_nxt;
   
   logic [9:0]                  rd_local_dw_size, rd_local_dw_size_nxt;

   always_comb begin
      logic [15:0]   rd_local_byte_size;
   
      rd_state_nxt = rd_state;

      rd_q_req_data_nxt = rd_q_req_data;

      if(rd_q_req_grant)
        rd_q_req_v_nxt = 0;
      else
        rd_q_req_v_nxt = rd_q_req_v;
      
      rd_lastBE_reg_nxt    = rd_lastBE_reg;
      rd_dw_len_reg_nxt    = rd_dw_len_reg;
      rd_addr_reg_nxt      = rd_addr_reg;
      rd_host_addr_reg_nxt = rd_host_addr_reg;

      rd_local_dw_size_nxt = rd_local_dw_size;
      
      rd_q_deq_en = 0;
      
      case(rd_state)
        RD_STATE_IDLE: begin
           if(~rd_q_empty) begin
              
              rd_local_byte_size = rd_q_deq_data[15:0] + {14'b0, rd_q_deq_data[85:84]};
              
              if(rd_local_byte_size > {4'b0, max_read_decoded, 2'b0}) begin // read bigger than max_read_request
                 if((({1'b0, rd_q_deq_data[31:22]} + {1'b0, max_read_decoded}) & 11'h400) != 0) begin // 4k hit
                    rd_local_dw_size_nxt = 10'd1024 - rd_q_deq_data[31:22];
                 end
                 else begin // 4k ok
                    rd_local_dw_size_nxt = max_read_decoded - ((max_read_decoded - 10'd1) & rd_q_deq_data[31:22]);
                 end
              end
              else begin // smaller than max_read_request
                 if(({1'b0, rd_q_deq_data[31:20]} + {1'b0, rd_q_deq_data[11:0]}) > 13'h1000) begin // 4k hit
                    rd_local_dw_size_nxt = 10'd1024 - rd_q_deq_data[31:22];
                 end
                 else begin // 4k ok
                    if(rd_local_byte_size[1:0] == 2'b00)
                      rd_local_dw_size_nxt = rd_local_byte_size[11:2];
                    else
                      rd_local_dw_size_nxt = rd_local_byte_size[11:2] + 1;
                 end                                   
              end
              
              rd_state_nxt = RD_STATE_START;

           end
        end
        RD_STATE_START: begin
           if(~rd_q_empty & (~rd_q_req_v | rd_q_req_grant)) begin
              rd_q_req_data_nxt[88+:`MEM_ADDR_BITS] = rd_q_deq_data[84+:`MEM_ADDR_BITS];
              rd_q_req_data_nxt[87:86] = IFACE_ID[1:0];
              rd_q_req_data_nxt[85:82] = rd_q_deq_data[19:16];
              rd_q_req_data_nxt[81:18] = rd_q_deq_data[83:20];

              rd_local_byte_size = rd_q_deq_data[15:0] + {14'b0, rd_q_deq_data[85:84]};
              
              if(rd_local_byte_size > {4'b0, max_read_decoded, 2'b0}) begin // read bigger than max_read_request
                 // lastDW BE
                 if(rd_local_dw_size == 1)
                   rd_q_req_data_nxt[3:0] = 4'b0000;
                 else
                   rd_q_req_data_nxt[3:0] = 4'b1111;                    
                 rd_state_nxt = RD_STATE_BODY_PREP;
              end
              else begin // smaller than max_read_request
                 if(({1'b0, rd_q_deq_data[31:20]} + {1'b0, rd_q_deq_data[11:0]}) > 13'h1000) begin // 4k hit
                    // lastDW BE
                    if(rd_local_dw_size == 1)
                      rd_q_req_data_nxt[3:0] = 4'b0000;
                    else
                      rd_q_req_data_nxt[3:0] = 4'b1111;
                    rd_state_nxt = RD_STATE_BODY_PREP;
                 end
                 else begin // 4k ok
                    // lastDW BE
                    if(rd_local_dw_size == 1)
                      rd_q_req_data_nxt[3:0] = 4'b0000;
                    else
                      case(rd_q_deq_data[85:84] + rd_q_deq_data[1:0])
                        2'b00: rd_q_req_data_nxt[3:0] = 4'b1111;
                        2'b01: rd_q_req_data_nxt[3:0] = 4'b0001;
                        2'b10: rd_q_req_data_nxt[3:0] = 4'b0011;
                        2'b11: rd_q_req_data_nxt[3:0] = 4'b0111;
                      endcase
                    rd_state_nxt = RD_STATE_IDLE;
                 end                                   
              end
              rd_q_req_data_nxt[17:8] = rd_local_dw_size;

              // first BE
              if(rd_local_dw_size == 1) begin
                if(rd_q_deq_data[1:0] == 2'd1)
                  case(rd_q_deq_data[85:84])
                    2'b00: rd_q_req_data_nxt[7:4] = 4'b0001;
                    2'b01: rd_q_req_data_nxt[7:4] = 4'b0010;
                    2'b10: rd_q_req_data_nxt[7:4] = 4'b0100;
                    2'b11: rd_q_req_data_nxt[7:4] = 4'b1000;
                  endcase
                else if(rd_q_deq_data[1:0] == 2'd2)
                  case(rd_q_deq_data[85:84])
                    2'b00: rd_q_req_data_nxt[7:4] = 4'b0011;
                    2'b01: rd_q_req_data_nxt[7:4] = 4'b0110;
                    2'b10: rd_q_req_data_nxt[7:4] = 4'b1100;
                    default: rd_q_req_data_nxt[7:4] = 4'b0000;
                  endcase
                else if(rd_q_deq_data[1:0] == 2'd3)
                  case(rd_q_deq_data[85:84])
                    2'b00: rd_q_req_data_nxt[7:4] = 4'b0111;
                    2'b01: rd_q_req_data_nxt[7:4] = 4'b1110;
                    default: rd_q_req_data_nxt[7:4] = 4'b0000;
                  endcase
                else if(rd_q_deq_data[1:0] == 2'd0)
                  case(rd_q_deq_data[85:84])
                    2'b00: rd_q_req_data_nxt[7:4] = 4'b1111;
                    default: rd_q_req_data_nxt[7:4] = 4'b0000;
                  endcase
                else
                  case(rd_q_deq_data[85:84])
                    2'b00: rd_q_req_data_nxt[7:4] = 4'b1111;
                    2'b01: rd_q_req_data_nxt[7:4] = 4'b1110;
                    2'b10: rd_q_req_data_nxt[7:4] = 4'b1100;
                    2'b11: rd_q_req_data_nxt[7:4] = 4'b1000;
                  endcase
              end
              else begin
                 rd_q_req_data_nxt[7:4] = 4'b1111;
              end
                 
              // last BE
              case(rd_q_deq_data[85:84] + rd_q_deq_data[1:0])
                2'b00: rd_lastBE_reg_nxt = 4'b1111;
                2'b01: rd_lastBE_reg_nxt = 4'b0001;
                2'b10: rd_lastBE_reg_nxt = 4'b0011;
                2'b11: rd_lastBE_reg_nxt = 4'b0111;
              endcase

              if(rd_local_byte_size[1:0] == 2'b00)
                rd_dw_len_reg_nxt = rd_local_byte_size[15:2] - rd_local_dw_size;
              else
                rd_dw_len_reg_nxt = rd_local_byte_size[15:2] - rd_local_dw_size + 1;
              
              rd_addr_reg_nxt = rd_q_deq_data[84+:`MEM_ADDR_BITS] + {{(`MEM_ADDR_BITS-12){1'b0}}, rd_local_dw_size, 2'b0};
              rd_host_addr_reg_nxt = rd_q_deq_data[83:20] + {52'b0, rd_local_dw_size, 2'b0};
              
              rd_q_req_v_nxt = 1;
              rd_q_deq_en = 1;                 

           end
        end
        RD_STATE_BODY_PREP: begin

           if(rd_dw_len_reg > max_read_decoded) begin // write bigger than max_read_request
              if((({1'b0, rd_host_addr_reg[11:2]} + {1'b0, max_read_decoded}) & 11'h400) != 0) begin // 4k hit
                 rd_local_dw_size_nxt = 10'd1024 - rd_host_addr_reg[11:2];
              end
              else begin // 4k ok
                 rd_local_dw_size_nxt = max_read_decoded - (rd_host_addr_reg[11:2] & (max_read_decoded-10'd1));
              end
           end
           else begin // smaller than max_payload
              if(({1'b0, rd_host_addr_reg[11:2]} + {1'b0, rd_dw_len_reg[9:0]}) > 11'h400) begin // 4k hit
                 rd_local_dw_size_nxt = 10'd1024 - rd_host_addr_reg[11:2];
              end
              else begin // 4k ok
                 rd_local_dw_size_nxt = rd_dw_len_reg;
              end                                   
           end

           rd_state_nxt = RD_STATE_BODY;

        end
        RD_STATE_BODY: begin
           if(~rd_q_req_v | rd_q_req_grant) begin

              rd_q_req_data_nxt[88+:`MEM_ADDR_BITS] = rd_addr_reg;
              rd_q_req_data_nxt[81:18] = rd_host_addr_reg;

              if(rd_dw_len_reg > max_read_decoded) begin // write bigger than max_read_request
                 // lastDW BE
                 if(rd_local_dw_size == 1)
                   rd_q_req_data_nxt[3:0] = 4'b0000;
                 else
                   rd_q_req_data_nxt[3:0] = 4'b1111;                    
                 rd_state_nxt = RD_STATE_BODY_PREP;
              end
              else begin // smaller than max_payload
                 if(({1'b0, rd_host_addr_reg[11:2]} + {1'b0, rd_dw_len_reg[9:0]}) > 11'h400) begin // 4k hit
                    // lastDW BE
                    if(rd_local_dw_size == 1)
                      rd_q_req_data_nxt[3:0] = 4'b0000;
                    else
                      rd_q_req_data_nxt[3:0] = 4'b1111;
                    rd_state_nxt = RD_STATE_BODY_PREP;
                 end
                 else begin // 4k ok
                    rd_state_nxt = RD_STATE_IDLE;
                    // lastDW BE
                    if(rd_local_dw_size == 1)
                      rd_q_req_data_nxt[3:0] = 4'b0000;
                    else
                      rd_q_req_data_nxt[3:0] = rd_lastBE_reg;
                 end                                   
              end
              
              rd_q_req_data_nxt[17:8] = rd_local_dw_size;

              // first BE
              if(rd_local_dw_size == 1)
                rd_q_req_data_nxt[7:4] = rd_lastBE_reg;
              else
                rd_q_req_data_nxt[7:4] = 4'b1111;

              rd_dw_len_reg_nxt = rd_dw_len_reg - rd_local_dw_size;
              rd_addr_reg_nxt = rd_addr_reg + {{(`MEM_ADDR_BITS-12){1'b0}}, rd_local_dw_size, 2'b0};
              rd_host_addr_reg_nxt = rd_host_addr_reg + {52'b0, rd_local_dw_size, 2'b0};
              
              rd_q_req_v_nxt = 1;
           end
        end
      endcase      
   end

   always_ff @(posedge clk) begin
      if(rst) begin
         rd_state   <= RD_STATE_IDLE;
         rd_q_req_v <= 0;
      end
      else begin
         rd_state      <= rd_state_nxt;
         rd_q_req_v    <= rd_q_req_v_nxt;        
      end
      rd_q_req_data <= rd_q_req_data_nxt;

      rd_local_dw_size <= rd_local_dw_size_nxt;
      
      rd_lastBE_reg    <= rd_lastBE_reg_nxt;
      rd_dw_len_reg    <= rd_dw_len_reg_nxt;
      rd_addr_reg      <= rd_addr_reg_nxt;
      rd_host_addr_reg <= rd_host_addr_reg_nxt;
   end

endmodule
