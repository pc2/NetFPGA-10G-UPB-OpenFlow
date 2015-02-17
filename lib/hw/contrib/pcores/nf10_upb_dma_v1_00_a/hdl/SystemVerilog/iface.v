/*******************************************************************************
 *
 *  NetFPGA-10G http://www.netfpga.org
 *
 *  File:
 *        iface.v
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
 *        Top AXIS interface module that wraps the memories and TX/RX engines
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

module iface #(parameter IFACE_ID = 0)
   (
    // memory write interface (pcie_clk)
    input logic [1:0]                   wr_if_select,
    input logic [3:0]                   wr_mem_select, 
    input logic [`MEM_ADDR_BITS-1:0]    wr_addr_hi,
    input logic [31:0]                  wr_data_hi,
    input logic [3:0]                   wr_mask_hi,
    input logic                         wr_en_hi,
    input logic [`MEM_ADDR_BITS-1:0]    wr_addr_lo,
    input logic [31:0]                  wr_data_lo,
    input logic [3:0]                   wr_mask_lo,
    input logic                         wr_en_lo,
   
    // memory read interface (pcie_clk)
    input logic [1:0]                   rd_if_select,
    input logic [3:0]                   rd_mem_select, 
    input logic [`MEM_ADDR_BITS-1:0]    rd_addr_hi,
    output logic [31:0]                 rd_data_hi,
    input logic                         rd_en_hi,
    input logic [`MEM_ADDR_BITS-1:0]    rd_addr_lo,
    output logic [31:0]                 rd_data_lo,
    input logic                         rd_en_lo,

    // completion queue write interface (pcie_clk)
    input logic                         cm_q_wr_en,
    input logic [`CM_Q_WIDTH-1:0]       cm_q_data,
    output logic                        cm_q_almost_full,

    // pcie_tx queues interface (rx_clk)
    output logic                        wr_q_req_v,
    output logic [`PCIE_WR_Q_WIDTH-1:0] wr_q_req_data,
    input logic                         wr_q_req_grant,
    // pcie_tx queues interface (tx_clk)
    output logic                        rd_q_req_v,
    output logic [`PCIE_RD_Q_WIDTH-1:0] rd_q_req_data,
    input logic                         rd_q_req_grant,
    // pcie_tx queues interface (pcie_clk)
    output logic                        cm_q_req_v,
    output logic [`PCIE_CM_Q_WIDTH-1:0] cm_q_req_data,
    input logic                         cm_q_req_grant,

    // pcie misc (pcie_clk)
    input logic [2:0]                   max_read_req_size,
    input logic [2:0]                   max_payload_size,
    input logic                         read_completion_bundary,
    output logic                        iface_rdy,

    // AXI lite master (axi_clk)
    output logic [31:0]                 axi_rdwr_addr,
    input logic [31:0]                  axi_rd_data,
    output logic [31:0]                 axi_wr_data,
    output logic                        axi_rd_go,
    output logic                        axi_wr_go,
    input logic                         axi_rd_done,
    input logic                         axi_wr_done,
    input logic                         axi_error,
    output logic                        mem_cfg_rd_valid,
    
    // TX MAC (tx_clk)
    output logic [63:0]                 M_AXIS_TDATA,
    output logic [7:0]                  M_AXIS_TSTRB,
    output logic                        M_AXIS_TVALID,
    input logic                         M_AXIS_TREADY,
    output logic                        M_AXIS_TLAST,
    output logic [127:0]                M_AXIS_TUSER,

    // RX MAC (rx_clk)
    input logic [63:0]                  S_AXIS_TDATA,
    input logic [7:0]                   S_AXIS_TSTRB,
    input logic                         S_AXIS_TVALID,
    output logic                        S_AXIS_TREADY,
    input logic                         S_AXIS_TLAST,
    input logic [127:0]                 S_AXIS_TUSER,

`ifdef DEBUG_PCIE
    // debug (pcie_clk)
    input logic [63:0]                  debug_t_h1[15:0],
    input logic [63:0]                  debug_t_h2[15:0],
    input logic [63:0]                  debug_t_ts[15:0],
    input logic [63:0]                  debug_r_h1[15:0],
    input logic [63:0]                  debug_r_h2[15:0],
    input logic [63:0]                  debug_r_ts[15:0],
`endif
    
    // stats (pcie_clk)
    input logic [63:0]                  stat_pcie_rx_ts,
    input logic [31:0]                  stat_pcie_rx_word_cnt,
    input logic [31:0]                  stat_pcie_rx_wr_cnt,
    input logic [31:0]                  stat_pcie_rx_rd_cnt,
    input logic [31:0]                  stat_pcie_rx_cm_cnt,
    input logic [31:0]                  stat_pcie_rx_err_cnt,
    input logic [63:0]                  stat_pcie_tx_ts,
    input logic [31:0]                  stat_pcie_tx_word_cnt,
    input logic [31:0]                  stat_pcie_tx_wr_cnt,
    input logic [31:0]                  stat_pcie_tx_rd_cnt,
    input logic [31:0]                  stat_pcie_tx_cm_cnt,
    input logic [31:0]                  stat_pcie_tx_err_cnt,

    // misc
    input logic                         axi_clk,
    input logic                         tx_clk,
    input logic                         rx_clk,
    input logic                         pcie_clk,
    input logic                         rst,

    inout wire [35:0]                   chipscope_control_0
    );

   // ------------------------------------
   // -- Fancy reset that lasts 16 cycles
   // ------------------------------------
   localparam RST_STATE_IDLE = 0;
   localparam RST_STATE_WAIT = 1;
   logic                                rst_state;
   logic [3:0]                          rst_cnt;
   logic                                rst_reg_p, rst_reg_t, rst_reg_r;

   always_ff @(posedge pcie_clk) begin
      if(rst | soft_reset) begin
         rst_state <= RST_STATE_WAIT;
         rst_reg_p <= 1;
         rst_cnt <= 1;
      end
      else begin
         case(rst_state)
           RST_STATE_IDLE: begin
              rst_state <= RST_STATE_IDLE;
              rst_reg_p <= 0;
           end
           RST_STATE_WAIT: begin
              rst_cnt <= rst_cnt + 1;
              if(rst_cnt == 0) begin
                 rst_state <= RST_STATE_IDLE;
                 rst_reg_p <= 0;
              end
              else begin
                 rst_state <= RST_STATE_WAIT;
                 rst_reg_p <= 1;
              end
           end
         endcase
      end
   end
   x_signal x_rst_t(pcie_clk, rst_reg_p, tx_clk, rst_reg_t);
   x_signal x_rst_r(pcie_clk, rst_reg_p, rx_clk, rst_reg_r);
   assign iface_rdy = ~rst_reg_p;

   // -----------------------------------
   // -- Config registers
   // -----------------------------------
   logic [63:0]           tx_dsc_mask;
   logic [63:0]           tx_pkt_mask;
   logic [63:0]           tx_dne_mask;
   logic [63:0]           tx_dne_mask_r;
   logic [63:0]           rx_dsc_mask;
   logic [63:0]           rx_dsc_mask_t;
   logic [63:0]           rx_pkt_mask;
   logic [63:0]           rx_dne_mask;
   logic [63:0]           host_tx_dne_offset;
   logic [63:0]           host_tx_dne_mask;
   logic [63:0]           host_rx_dne_offset;
   logic [63:0]           host_rx_dne_mask;
   logic [15:0]           rx_byte_wait;
   logic                  tx_int_enable;
   logic                  rx_int_enable;
   logic                  soft_reset;

   // -----------------------------------
   // -- stats
   // -----------------------------------
   logic [63:0]           stat_mac_tx_ts;
   logic [31:0]           stat_mac_tx_word_cnt;
   logic [31:0]           stat_mac_tx_pkt_cnt;
   
   logic [63:0]           stat_mac_rx_ts;
   logic [31:0]           stat_mac_rx_word_cnt;
   logic [31:0]           stat_mac_rx_pkt_cnt;
   logic [31:0]           stat_mac_rx_err_cnt;


   // -----------------------------------
   // -- Memory valid bits
   // -----------------------------------
   logic [`MEM_ADDR_BITS-12:0] mem_vld_tx_dsc_wr_addr;
   logic [31:0]                mem_vld_tx_dsc_wr_mask;
   logic                       mem_vld_tx_dsc_wr_clear;
   logic                       mem_vld_tx_dsc_wr_stall;
   logic                       mem_vld_tx_dsc_rd_bit;
   
   logic [`MEM_ADDR_BITS-12:0] mem_vld_tx_pkt_wr_addr;
   logic [31:0]                mem_vld_tx_pkt_wr_mask;
   logic                       mem_vld_tx_pkt_wr_clear;
   logic                       mem_vld_tx_pkt_wr_stall;
   logic                       mem_vld_tx_pkt_rd_bit;
   
   logic [`MEM_ADDR_BITS-12:0] mem_vld_tx_dne_wr_addr;
   logic [31:0]                mem_vld_tx_dne_wr_mask;
   logic                       mem_vld_tx_dne_wr_clear;
   logic                       mem_vld_tx_dne_wr_stall;
   logic [`MEM_ADDR_BITS-12:0] mem_vld_tx_dne_rd_addr;
   logic [31:0]                mem_vld_tx_dne_rd_bits;
   
   logic [`MEM_ADDR_BITS-12:0] mem_vld_rx_dsc_wr_addr;
   logic [31:0]                mem_vld_rx_dsc_wr_mask;
   logic                       mem_vld_rx_dsc_wr_clear;
   logic                       mem_vld_rx_dsc_wr_stall;
   logic                       mem_vld_rx_dsc_rd_bit;
      
   logic [`MEM_ADDR_BITS-12:0] mem_vld_rx_dne_wr_addr;
   logic [31:0]                mem_vld_rx_dne_wr_mask;
   logic                       mem_vld_rx_dne_wr_clear;
   logic                       mem_vld_rx_dne_wr_stall;
   logic [`MEM_ADDR_BITS-12:0] mem_vld_rx_dne_rd_addr;
   logic [31:0]                mem_vld_rx_dne_rd_bits;

   // -----------------------------------
   // -- Memory read interfaces
   // -----------------------------------
   logic [`MEM_ADDR_BITS-1:0] mem_tx_dsc_rd_addr;
   logic [63:0]               mem_tx_dsc_rd_data;
   logic                      mem_tx_dsc_rd_en;

   logic [`MEM_ADDR_BITS-1:0] mem_tx_pkt_rd_addr;
   logic [63:0]               mem_tx_pkt_rd_data;
   logic                      mem_tx_pkt_rd_en;

   logic [`MEM_ADDR_BITS-1:0] mem_rx_dsc_rd_addr;
   logic [63:0]               mem_rx_dsc_rd_data;
   logic                      mem_rx_dsc_rd_en;

   // -----------------------------------
   // -- Memory write interfaces
   // -----------------------------------
   logic [`MEM_ADDR_BITS-1:0] mem_tx_dne_wr_addr;
   logic [63:0]               mem_tx_dne_wr_data;
   logic [7:0]                mem_tx_dne_wr_mask;
   logic                      mem_tx_dne_wr_en;

   logic [`MEM_ADDR_BITS-1:0] mem_rx_pkt_wr_addr;
   logic [63:0]               mem_rx_pkt_wr_data;
   logic [7:0]                mem_rx_pkt_wr_mask;
   logic                      mem_rx_pkt_wr_en;

   logic [`MEM_ADDR_BITS-1:0] mem_rx_dne_wr_addr;
   logic [63:0]               mem_rx_dne_wr_data;
   logic [7:0]                mem_rx_dne_wr_mask;
   logic                      mem_rx_dne_wr_en;
   
   // -----------------------------------
   // -- Instantiate memories
   // -----------------------------------
   logic [31:0]           rd_data_lo_array[7:0];
   logic [31:0]           rd_data_hi_array[7:0];

   logic [3:0]            rd_mem_select_d1;

   assign rd_data_lo_array[`ID_MEM_RX_DSC] = 32'hdeadbeef;
   assign rd_data_lo_array[`ID_MEM_TX_PKT] = 32'hdeadbeef;
   assign rd_data_lo_array[`ID_MEM_TX_DSC] = 32'hdeadbeef;
   assign rd_data_hi_array[`ID_MEM_RX_DSC] = 32'hdeadbeef;
   assign rd_data_hi_array[`ID_MEM_TX_PKT] = 32'hdeadbeef;
   assign rd_data_hi_array[`ID_MEM_TX_DSC] = 32'hdeadbeef;
   
   always_ff @(posedge pcie_clk) rd_mem_select_d1 <= rd_mem_select;
   assign rd_data_lo = rd_data_lo_array[rd_mem_select_d1[2:0]];
   assign rd_data_hi = rd_data_hi_array[rd_mem_select_d1[2:0]];

   cfg 
     u_cfg (.wr_mem_valid((wr_if_select == IFACE_ID[1:0]) && (wr_mem_select == `ID_MEM_CFG)),
            .rd_mem_valid((rd_if_select == IFACE_ID[1:0]) && (rd_mem_select == `ID_MEM_CFG)), 
            .rd_data_lo(rd_data_lo_array[`ID_MEM_CFG]),
            .rd_data_hi(rd_data_hi_array[`ID_MEM_CFG]),
            .tx_clk(tx_clk),
            .rx_clk(rx_clk),
            .pcie_clk(pcie_clk),
            .rst(rst_reg_p),
            .*);
   
   stats 
     u_stats (.wr_mem_valid((wr_if_select == IFACE_ID[1:0]) && (wr_mem_select == `ID_MEM_STAT)),
              .rd_mem_valid((rd_if_select == IFACE_ID[1:0]) && (rd_mem_select == `ID_MEM_STAT)), 
              .rd_data_lo(rd_data_lo_array[`ID_MEM_STAT]),
              .rd_data_hi(rd_data_hi_array[`ID_MEM_STAT]),
              .tx_clk(tx_clk),
              .rx_clk(rx_clk),
              .pcie_clk(pcie_clk),
              .rst(rst_reg_p),
              .*);
   
   mem #(.DEPTH(`MEM_N_TX_DSC), .WIDTH(16), .VALID_MODE(1), .HAS_WR_MASK(1)) 
   u_mem_tx_dsc (.wr_mem_valid((wr_if_select == IFACE_ID[1:0]) && (wr_mem_select == `ID_MEM_TX_DSC)),
                 .rd_mem_valid(1'b1),
                 .rd_addr_hi(mem_tx_dsc_rd_addr),
                 .rd_data_hi(mem_tx_dsc_rd_data[63:32]),
                 .rd_en_hi(mem_tx_dsc_rd_en),
                 .rd_addr_lo(mem_tx_dsc_rd_addr),
                 .rd_data_lo(mem_tx_dsc_rd_data[31:0]),
                 .rd_en_lo(mem_tx_dsc_rd_en),
                 .rd_vld_lo(mem_vld_tx_dsc_rd_bit),
                 .valid_wr_addr(mem_vld_tx_dsc_wr_addr),
                 .valid_wr_mask(mem_vld_tx_dsc_wr_mask),
                 .valid_wr_clear(mem_vld_tx_dsc_wr_clear),
                 .valid_wr_stall(mem_vld_tx_dsc_wr_stall),
                 .valid_rd_addr(),
                 .valid_rd_bits(),
                 .valid_rd_addr_x(),
                 .valid_rd_bits_x(),
                 .valid_rd_clk(),
                 .valid_wr_clk(tx_clk),
                 .wr_clk(pcie_clk),
                 .rd_clk(tx_clk),
                 .rst(rst_reg_p),
                 .*);
   mem #(.DEPTH(`MEM_N_TX_PKT), .WIDTH(64), .VALID_MODE(1), .HAS_WR_MASK(1)) 
   u_mem_tx_pkt (.wr_mem_valid((wr_if_select == IFACE_ID[1:0]) && (wr_mem_select == `ID_MEM_TX_PKT)),
                 .rd_mem_valid(1),
                 .rd_addr_hi(mem_tx_pkt_rd_addr),
                 .rd_data_hi(mem_tx_pkt_rd_data[63:32]),
                 .rd_en_hi(mem_tx_pkt_rd_en),
                 .rd_addr_lo(mem_tx_pkt_rd_addr),
                 .rd_data_lo(mem_tx_pkt_rd_data[31:0]),
                 .rd_en_lo(mem_tx_pkt_rd_en),
                 .rd_vld_lo(mem_vld_tx_pkt_rd_bit),
                 .valid_wr_addr(mem_vld_tx_pkt_wr_addr),
                 .valid_wr_mask(mem_vld_tx_pkt_wr_mask),
                 .valid_wr_clear(mem_vld_tx_pkt_wr_clear),
                 .valid_wr_stall(mem_vld_tx_pkt_wr_stall),
                 .valid_rd_addr(),
                 .valid_rd_bits(),
                 .valid_rd_addr_x(),
                 .valid_rd_bits_x(),
                 .valid_rd_clk(),
                 .valid_wr_clk(tx_clk),
                 .wr_clk(pcie_clk),
                 .rd_clk(tx_clk),
                 .rst(rst_reg_p),
                 .*);
   mem #(.DEPTH(`MEM_N_TX_DNE), .WIDTH(4), .VALID_MODE(2), .HAS_WR_MASK(0)) 
   u_mem_tx_dne (.wr_mem_valid(1),
                 .rd_mem_valid((rd_if_select == IFACE_ID[1:0]) && (rd_mem_select == `ID_MEM_TX_DNE)), 
                 .wr_addr_hi(mem_tx_dne_wr_addr),
                 .wr_data_hi(mem_tx_dne_wr_data[63:32]),
                 .wr_mask_hi(mem_tx_dne_wr_mask[7:4]),
                 .wr_en_hi(mem_tx_dne_wr_en),
                 .wr_addr_lo(mem_tx_dne_wr_addr),
                 .wr_data_lo(mem_tx_dne_wr_data[31:0]),
                 .wr_mask_lo(mem_tx_dne_wr_mask[3:0]),
                 .wr_en_lo(mem_tx_dne_wr_en),
                 .rd_data_lo(rd_data_lo_array[`ID_MEM_TX_DNE]),
                 .rd_data_hi(rd_data_hi_array[`ID_MEM_TX_DNE]),
                 .rd_vld_lo(),
                 .valid_wr_addr(mem_vld_tx_dne_wr_addr),
                 .valid_wr_mask(mem_vld_tx_dne_wr_mask),
                 .valid_wr_clear(mem_vld_tx_dne_wr_clear),
                 .valid_wr_stall(mem_vld_tx_dne_wr_stall),
                 .valid_rd_addr(mem_vld_tx_dne_rd_addr),
                 .valid_rd_bits(mem_vld_tx_dne_rd_bits),
                 .valid_rd_addr_x(),
                 .valid_rd_bits_x(),
                 .valid_rd_clk(rx_clk),
                 .valid_wr_clk(tx_clk),
                 .wr_clk(tx_clk),
                 .rd_clk(pcie_clk),
                 .rst(rst_reg_t),
                 .*);
   
   mem #(.DEPTH(`MEM_N_RX_DSC), .WIDTH(16), .VALID_MODE(1), .HAS_WR_MASK(1)) 
   u_mem_rx_dsc (.wr_mem_valid((wr_if_select == IFACE_ID[1:0]) && (wr_mem_select == `ID_MEM_RX_DSC)),
                 .rd_mem_valid(1),
                 .rd_addr_hi(mem_rx_dsc_rd_addr),
                 .rd_data_hi(mem_rx_dsc_rd_data[63:32]),
                 .rd_en_hi(mem_rx_dsc_rd_en),
                 .rd_addr_lo(mem_rx_dsc_rd_addr),
                 .rd_data_lo(mem_rx_dsc_rd_data[31:0]),
                 .rd_en_lo(mem_rx_dsc_rd_en),
                 .rd_vld_lo(mem_vld_rx_dsc_rd_bit),
                 .valid_wr_addr(mem_vld_rx_dsc_wr_addr),
                 .valid_wr_mask(mem_vld_rx_dsc_wr_mask),
                 .valid_wr_clear(mem_vld_rx_dsc_wr_clear),
                 .valid_wr_stall(mem_vld_rx_dsc_wr_stall),
                 .valid_rd_addr(),
                 .valid_rd_bits(),
                 .valid_rd_addr_x(),
                 .valid_rd_bits_x(),
                 .valid_rd_clk(),
                 .valid_wr_clk(rx_clk),
                 .wr_clk(pcie_clk),
                 .rd_clk(rx_clk),
                 .rst(rst_reg_p),
                 .*);
   mem #(.DEPTH(`MEM_N_RX_PKT), .WIDTH(64), .VALID_MODE(0), .HAS_WR_MASK(0)) 
   u_mem_rx_pkt (.wr_mem_valid(1),
                 .rd_mem_valid((rd_if_select == IFACE_ID[1:0]) && (rd_mem_select == `ID_MEM_RX_PKT)), 
                 .wr_addr_hi(mem_rx_pkt_wr_addr),
                 .wr_data_hi(mem_rx_pkt_wr_data[63:32]),
                 .wr_mask_hi(mem_rx_pkt_wr_mask[7:4]),
                 .wr_en_hi(mem_rx_pkt_wr_en),
                 .wr_addr_lo(mem_rx_pkt_wr_addr),
                 .wr_data_lo(mem_rx_pkt_wr_data[31:0]),
                 .wr_mask_lo(mem_rx_pkt_wr_mask[3:0]),
                 .wr_en_lo(mem_rx_pkt_wr_en),
                 .rd_data_lo(rd_data_lo_array[`ID_MEM_RX_PKT]),
                 .rd_data_hi(rd_data_hi_array[`ID_MEM_RX_PKT]),
                 .rd_vld_lo(),
                 .valid_wr_addr(),
                 .valid_wr_mask(),
                 .valid_wr_clear(),
                 .valid_wr_stall(),
                 .valid_rd_addr(),
                 .valid_rd_bits(),
                 .valid_rd_addr_x(),
                 .valid_rd_bits_x(),
                 .valid_rd_clk(),
                 .valid_wr_clk(),
                 .wr_clk(rx_clk),
                 .rd_clk(pcie_clk),
                 .rst(rst_reg_r),
                 .*);
   mem #(.DEPTH(`MEM_N_RX_DNE), .WIDTH(8), .VALID_MODE(2), .HAS_WR_MASK(0)) 
   u_mem_rx_dne (.wr_mem_valid(1),
                 .rd_mem_valid((rd_if_select == IFACE_ID[1:0]) && (rd_mem_select == `ID_MEM_RX_DNE)), 
                 .wr_addr_hi(mem_rx_dne_wr_addr),
                 .wr_data_hi(mem_rx_dne_wr_data[63:32]),
                 .wr_mask_hi(mem_rx_dne_wr_mask[7:4]),
                 .wr_en_hi(mem_rx_dne_wr_en),
                 .wr_addr_lo(mem_rx_dne_wr_addr),
                 .wr_data_lo(mem_rx_dne_wr_data[31:0]),
                 .wr_mask_lo(mem_rx_dne_wr_mask[3:0]),
                 .wr_en_lo(mem_rx_dne_wr_en),
                 .rd_data_lo(rd_data_lo_array[`ID_MEM_RX_DNE]),
                 .rd_data_hi(rd_data_hi_array[`ID_MEM_RX_DNE]),
                 .rd_vld_lo(),
                 .valid_wr_addr(mem_vld_rx_dne_wr_addr),
                 .valid_wr_mask(mem_vld_rx_dne_wr_mask),
                 .valid_wr_clear(mem_vld_rx_dne_wr_clear),
                 .valid_wr_stall(mem_vld_rx_dne_wr_stall),
                 .valid_rd_addr(mem_vld_rx_dne_rd_addr),
                 .valid_rd_bits(mem_vld_rx_dne_rd_bits),
                 .valid_rd_addr_x(),
                 .valid_rd_bits_x(),
                 .valid_rd_clk(rx_clk),
                 .valid_wr_clk(rx_clk),
                 .wr_clk(rx_clk),
                 .rd_clk(pcie_clk),
                 .rst(rst_reg_r),
                 .*);
   
   // -----------------------------------
   // -- Decode pcie settings
   // -----------------------------------
   logic [9:0]            max_payload_decoded, max_payload_decoded_reg_p, max_payload_decoded_reg_r;
   logic [9:0]            max_read_decoded, max_read_decoded_reg_t;
   always_comb begin
      unique case(max_payload_size)
        'b000: max_payload_decoded = 10'd32;
        'b001: max_payload_decoded = 10'd64;
        'b010: max_payload_decoded = 10'd128;
        'b011: max_payload_decoded = 10'd256;
        'b100: max_payload_decoded = 10'd512;
        'b101: max_payload_decoded = 10'd1024; // i.e. 'd0
        default: max_payload_decoded = 'd0;
      endcase
      unique case(max_read_req_size)
        'b000: max_read_decoded = 10'd32;
        'b001: max_read_decoded = 10'd64;
        'b010: max_read_decoded = 10'd128;
        'b011: max_read_decoded = 10'd256;
        'b100: max_read_decoded = 10'd512;
        'b101: max_read_decoded = 10'd1024; // i.e. 'd0
        default: max_read_decoded = 'd0;
      endcase
   end
   always_ff @(posedge pcie_clk) max_payload_decoded_reg_p <= max_payload_decoded;
   x_signal #(10) u_x_pcie_0(pcie_clk, max_payload_decoded, rx_clk, max_payload_decoded_reg_r);
   x_signal #(10) u_x_pcie_1(pcie_clk, max_read_decoded,    tx_clk, max_read_decoded_reg_t);


   // -----------------------------------
   // -- Packet RX and TX control
   // -----------------------------------
   logic                        wr_q_enq_en;
   logic [`WR_Q_WIDTH-1:0]      wr_q_enq_data;
   logic                        wr_q_almost_full;
   logic                        wr_q_full;
   
   logic                        rd_q_enq_en;
   logic [`RD_Q_WIDTH-1:0]      rd_q_enq_data;
   logic                        rd_q_full;

   rx_ctrl u_rx_ctrl (.tx_dne_mask(tx_dne_mask_r),
                      .rst(rst_reg_r),
                      .clk(rx_clk),
                      .chipscope_control_0(chipscope_control_0),
                      .*);
   
   tx_ctrl u_tx_ctrl (.rx_dsc_mask(rx_dsc_mask_t),
                      .rst(rst_reg_t), 
                      .clk(tx_clk),
                      .*);
   

   // ------------------------------------
   // -- PCIe queues and packet splitters
   // ------------------------------------

   pcie_wr_q #(.IFACE_ID(IFACE_ID)) u_pcie_wr_q(.max_payload_decoded(max_payload_decoded_reg_r),
                                                .rst(rst_reg_r), 
                                                .clk(rx_clk),
                                                .*);
   
   pcie_rd_q #(.IFACE_ID(IFACE_ID)) u_pcie_rd_q(.max_read_decoded(max_read_decoded_reg_t),
                                                .rst(rst_reg_t), 
                                                .clk(tx_clk),
                                                .*);
   
   pcie_cm_q u_pcie_cm_q(.max_payload_decoded(max_payload_decoded_reg_p),
                         .pcie_clk(pcie_clk),
                         .rx_clk(rx_clk),
                         .rst(rst_reg_p),
                         .*);

endmodule
