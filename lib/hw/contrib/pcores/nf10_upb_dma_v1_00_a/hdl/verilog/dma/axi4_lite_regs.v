/*******************************************************************************
 *
 *  NetFPGA-10G http://www.netfpga.org
 *
 *  File:
 *        axi4_lite_regs.v
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
 *  Description:
 *        Simple AXI slave interface to 8 test registers that can be read/written.
 *        These registers do not affect dma operation in any way, but can be used
 *        for test purposes.
 *
 *  Copyright notice:
 *        Copyright (C) 2010, 2011 The Board of Trustees of The Leland Stanford
 *                                 Junior University
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

module axi4_lite_regs_test
  #(
    // Master AXI Lite Data Width
    parameter DATA_WIDTH=32,
    parameter ADDR_WIDTH=32
    )
   (
    input                        ACLK,
    input                        ARESETN,
    
    input [ADDR_WIDTH-1: 0]      AWADDR,
    input                        AWVALID,
    output reg                   AWREADY,
    
    input [DATA_WIDTH-1: 0]      WDATA,
    input [DATA_WIDTH/8-1: 0]    WSTRB,
    input                        WVALID,
    output reg                   WREADY,
    
    output reg [1:0]             BRESP,
    output reg                   BVALID,
    input                        BREADY,
    
    input [ADDR_WIDTH-1: 0]      ARADDR,
    input                        ARVALID,
    output reg                   ARREADY,
    
    output reg [DATA_WIDTH-1: 0] RDATA, 
    output reg [1:0]             RRESP,
    output reg                   RVALID,
    input                        RREADY
    );

   localparam AXI_RESP_OK = 2'b00;
   localparam AXI_RESP_SLVERR = 2'b10;
   
   localparam WRITE_IDLE = 0;
   localparam WRITE_RESPONSE = 1;
   localparam WRITE_DATA = 2;

   localparam READ_IDLE = 0;
   localparam READ_RESPONSE = 1;

   localparam REG_TEST_0 = 4'h0;
   localparam REG_TEST_1 = 4'h1;
   localparam REG_TEST_2 = 4'h2;
   localparam REG_TEST_3 = 4'h3;
   localparam REG_TEST_4 = 4'h4;
   localparam REG_TEST_5 = 4'h5;
   localparam REG_TEST_6 = 4'h6;
   localparam REG_TEST_7 = 4'h7;

   reg [1:0]                     write_state, write_state_next;
   reg [1:0]                     read_state, read_state_next;
   reg [ADDR_WIDTH-1:0]          read_addr, read_addr_next;
   reg [ADDR_WIDTH-1:0]          write_addr, write_addr_next;
   reg [1:0]                     BRESP_next;

   reg [31:0]                    test_reg_0, test_reg_0_next;
   reg [31:0]                    test_reg_1, test_reg_1_next;
   reg [31:0]                    test_reg_2, test_reg_2_next;
   reg [31:0]                    test_reg_3, test_reg_3_next;
   reg [31:0]                    test_reg_4, test_reg_4_next;
   reg [31:0]                    test_reg_5, test_reg_5_next;
   reg [31:0]                    test_reg_6, test_reg_6_next;
   reg [31:0]                    test_reg_7, test_reg_7_next;
   
   always @(*) begin
      read_state_next = read_state;   
      ARREADY = 1'b1;
      read_addr_next = read_addr;
      RDATA = 0; 
      RRESP = AXI_RESP_OK;
      RVALID = 1'b0;
      
      case(read_state)
        READ_IDLE: begin
           if(ARVALID) begin
              read_addr_next = ARADDR;
              read_state_next = READ_RESPONSE;
           end
        end        
        
        READ_RESPONSE: begin
           RVALID = 1'b1;
           ARREADY = 1'b0;

           if(read_addr[3:0] == REG_TEST_0) begin
              RDATA = test_reg_0;
           end
           else if(read_addr[3:0] == REG_TEST_1) begin
              RDATA = test_reg_1;
           end
           else if(read_addr[3:0] == REG_TEST_2) begin
              RDATA = test_reg_2;
           end
           else if(read_addr[3:0] == REG_TEST_3) begin
              RDATA = test_reg_3;
           end
           else if(read_addr[3:0] == REG_TEST_4) begin
              RDATA = test_reg_4;
           end
           else if(read_addr[3:0] == REG_TEST_5) begin
              RDATA = test_reg_5;
           end
           else if(read_addr[3:0] == REG_TEST_6) begin
              RDATA = test_reg_6;
           end
           else if(read_addr[3:0] == REG_TEST_7) begin
              RDATA = test_reg_7;
           end
           else begin
              RRESP = AXI_RESP_SLVERR;
           end

           if(RREADY) begin
              read_state_next = READ_IDLE;
           end
        end
      endcase
   end
   
   always @(*) begin
      write_state_next = write_state;
      write_addr_next = write_addr;

      AWREADY = 1'b1;
      WREADY = 1'b0;
      BVALID = 1'b0;  
      BRESP_next = BRESP;

      test_reg_0_next = test_reg_0;
      test_reg_1_next = test_reg_1;
      test_reg_2_next = test_reg_2;
      test_reg_3_next = test_reg_3;
      test_reg_4_next = test_reg_4;
      test_reg_5_next = test_reg_5;
      test_reg_6_next = test_reg_6;
      test_reg_7_next = test_reg_7;

      case(write_state)
        WRITE_IDLE: begin
           write_addr_next = AWADDR;
           if(AWVALID) begin
              write_state_next = WRITE_DATA;
           end
        end
        WRITE_DATA: begin
           AWREADY = 1'b0;
           WREADY = 1'b1;
           if(WVALID) begin
              if (write_addr[2:0] == REG_TEST_0) begin
                 test_reg_0_next = WDATA;
                 BRESP_next = AXI_RESP_OK;
              end
              else if (write_addr[2:0] == REG_TEST_1) begin
                 test_reg_1_next = WDATA;
                 BRESP_next = AXI_RESP_OK;
              end
              else if (write_addr[2:0] == REG_TEST_2) begin
                 test_reg_2_next = WDATA;
                 BRESP_next = AXI_RESP_OK;
              end
              else if (write_addr[2:0] == REG_TEST_3) begin
                 test_reg_3_next = WDATA;
                 BRESP_next = AXI_RESP_OK;
              end
              else if (write_addr[2:0] == REG_TEST_4) begin
                 test_reg_4_next = WDATA;
                 BRESP_next = AXI_RESP_OK;
              end
              else if (write_addr[2:0] == REG_TEST_5) begin
                 test_reg_5_next = WDATA;
                 BRESP_next = AXI_RESP_OK;
              end
              else if (write_addr[2:0] == REG_TEST_6) begin
                 test_reg_6_next = WDATA;
                 BRESP_next = AXI_RESP_OK;
              end
              else if (write_addr[2:0] == REG_TEST_7) begin
                 test_reg_7_next = WDATA;
                 BRESP_next = AXI_RESP_OK;
              end
              else begin
                 BRESP_next = AXI_RESP_SLVERR;
              end
              write_state_next = WRITE_RESPONSE;
           end
        end
        WRITE_RESPONSE: begin
           AWREADY = 1'b0;
           BVALID = 1'b1;
           if(BREADY) begin                    
              write_state_next = WRITE_IDLE;
           end
        end
      endcase
   end

   always @(posedge ACLK) begin
      if(~ARESETN) begin
         write_state <= WRITE_IDLE;
         read_state <= READ_IDLE;
         read_addr <= 0;
         write_addr <= 0;
         BRESP <= AXI_RESP_OK;

         test_reg_0 <= 32'h00007700;
         test_reg_1 <= 32'h00007701;
         test_reg_2 <= 32'h00007702;
         test_reg_3 <= 32'h00007703;
         test_reg_4 <= 32'h00007704;
         test_reg_5 <= 32'h00007705;
         test_reg_6 <= 32'h00007706;
         test_reg_7 <= 32'h00007707;
      end
      else begin
         write_state <= write_state_next;
         read_state <= read_state_next;
         read_addr <= read_addr_next;
         write_addr <= write_addr_next;
         BRESP <= BRESP_next;

         test_reg_0 <= test_reg_0_next;
         test_reg_1 <= test_reg_1_next;
         test_reg_2 <= test_reg_2_next;
         test_reg_3 <= test_reg_3_next;
         test_reg_4 <= test_reg_4_next;
         test_reg_5 <= test_reg_5_next;
         test_reg_6 <= test_reg_6_next;
         test_reg_7 <= test_reg_7_next;

      end
   end
   

endmodule
