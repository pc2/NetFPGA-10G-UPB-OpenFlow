/*******************************************************************************
 *
 *  NetFPGA-10G http://www.netfpga.org
 *
 *  File:
 *        ipif_regs.v
 *
 *  Library:
 *        std/pcores/nf10_proc_common_v1_00_a
 *
 *  Module:
 *        ipif_regs
 *
 *  Author:
 *        Muhammad Shahbaz
 *
 *  Description:
 *       5/June/2013 Jong 
 *			Added IP2Bus_WrAck signal. This signal is required when trying to write
 *			a register value to read only registers.
 *        
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
 
 module ipif_regs 
 #(
   parameter C_S_AXI_DATA_WIDTH = 32,          
   parameter C_S_AXI_ADDR_WIDTH = 32,   
   parameter NUM_WO_REGS = 0, // Number of registers written by software and read by hardware only
   parameter NUM_RW_REGS = 0, // Number of registers written by software and read by both hardware and software
   parameter NUM_RO_REGS = 0  // Number of registers written by hardware and read by software only
   // Address Mapping
   //  ------  = base_address
   // |  WO  |         
   // |------|         |
   // |  RW  |         |
   // |------|         \/
   // |  RO  |
   //  ------  = high_address
 )
 (   
   // -- IPIF ports
   input                                               Bus2IP_Clk,
   input                                               Bus2IP_Resetn,
   input      [C_S_AXI_ADDR_WIDTH-1 : 0]               Bus2IP_Addr,
   input                                               Bus2IP_CS,
   input                                               Bus2IP_RNW,
   input      [C_S_AXI_DATA_WIDTH-1 : 0]               Bus2IP_Data,
   input      [C_S_AXI_DATA_WIDTH/8-1 : 0]             Bus2IP_BE,
   output     reg [C_S_AXI_DATA_WIDTH-1 : 0]           IP2Bus_Data,
   output     reg                                      IP2Bus_RdAck,
   output     reg                                      IP2Bus_WrAck,
   output                                              IP2Bus_Error,
   
   // -- Register ports
   output    [NUM_WO_REGS*C_S_AXI_DATA_WIDTH : 0]    wo_regs,
   output    [NUM_RW_REGS*C_S_AXI_DATA_WIDTH : 0]    rw_regs,
   input     [NUM_RO_REGS*C_S_AXI_DATA_WIDTH : 0]    ro_regs
 );

    function integer log2;
      input integer number;
      begin
         log2=0;
         while(2**log2<number) begin
            log2=log2+1;
         end
      end
   endfunction
   
   // -- internal parameters
   localparam addr_width = log2(NUM_WO_REGS+NUM_RW_REGS+NUM_RO_REGS);
   localparam addr_width_lsb = log2(C_S_AXI_ADDR_WIDTH/8);
   localparam addr_width_msb = addr_width+addr_width_lsb;
 
   // -- interal wire/regs
   genvar i;
   integer j;
   
   wire [C_S_AXI_DATA_WIDTH-1 : 0] reg_file_rd_port [0 : NUM_RW_REGS+NUM_RO_REGS-1];
   reg  [C_S_AXI_DATA_WIDTH-1 : 0] reg_file_wr_port [0 : NUM_WO_REGS+NUM_RW_REGS-1];
   
   generate	 
	 // Unpacking Write Only registers
	 if (NUM_WO_REGS > 0)       
	   for (i=0; i<NUM_WO_REGS; i=i+1) begin : WO
	     assign wo_regs[C_S_AXI_DATA_WIDTH*(i+1)-1 : C_S_AXI_DATA_WIDTH*i] = reg_file_wr_port[i];
	   end

	 // Unpacking Read Write registers
	 if (NUM_RW_REGS > 0)       
	   for (i=0; i<NUM_RW_REGS; i=i+1) begin : RW
	     assign rw_regs[C_S_AXI_DATA_WIDTH*(i+1)-1 : C_S_AXI_DATA_WIDTH*i] = reg_file_wr_port[NUM_WO_REGS+i];
		 assign reg_file_rd_port[i] = reg_file_wr_port[NUM_WO_REGS+i];
	   end

     // Unpacking Read Only registers
     if (NUM_RO_REGS > 0)       
	   for (i=0; i<NUM_RO_REGS; i=i+1) begin : RO
	     assign reg_file_rd_port[NUM_RW_REGS+i] = ro_regs[C_S_AXI_DATA_WIDTH*(i+1)-1 : C_S_AXI_DATA_WIDTH*i];
           end
   endgenerate
 
   // -- Implementation
   
   assign IP2Bus_Error = 1'b0;
   
   // SW writes
   always @ (posedge Bus2IP_Clk) begin
     if (~Bus2IP_Resetn) begin
	   for (j=0; j<(NUM_WO_REGS+NUM_RW_REGS); j=j+1) 
	     reg_file_wr_port[j] <= {C_S_AXI_DATA_WIDTH{1'b0}};
	   
	   IP2Bus_WrAck <= 1'b0;
	 end
	 else begin
	   IP2Bus_WrAck <= 1'b0;
	   
	   if (Bus2IP_CS && !Bus2IP_RNW && Bus2IP_Addr[addr_width_msb-1:addr_width_lsb] < (NUM_WO_REGS+NUM_RW_REGS)) begin
	     reg_file_wr_port[Bus2IP_Addr[addr_width_msb-1:addr_width_lsb]] <= Bus2IP_Data;
		 IP2Bus_WrAck <= 1'b1;
	   end
		//Bus waits Ack signal even a value is written to a read-only register. 
		else if (Bus2IP_CS && !Bus2IP_RNW && Bus2IP_Addr[addr_width_msb-1:addr_width_lsb] >= (NUM_WO_REGS+NUM_RW_REGS)) begin
			IP2Bus_WrAck	<= 1'b1;
		end
	 end
   end
   
   // SW reads
   always @ (posedge Bus2IP_Clk) begin
     if (~Bus2IP_Resetn) begin
	   IP2Bus_Data <= {C_S_AXI_DATA_WIDTH{1'b0}};
	   IP2Bus_RdAck <= 1'b0;
	 end
	 else begin
	   IP2Bus_RdAck <= 1'b0;
	   
	   if (Bus2IP_CS && Bus2IP_RNW && Bus2IP_Addr[addr_width_msb-1:addr_width_lsb] >= (NUM_WO_REGS)) begin
	     IP2Bus_Data <= reg_file_rd_port[Bus2IP_Addr[addr_width_msb-1:addr_width_lsb]-NUM_WO_REGS];
		 IP2Bus_RdAck <= 1'b1;
	   end
	 end
   end
    
 endmodule 
