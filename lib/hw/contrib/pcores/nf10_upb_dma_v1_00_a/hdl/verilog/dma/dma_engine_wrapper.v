/*******************************************************************************
 *
 *  NetFPGA-10G http://www.netfpga.org
 *
 *  File:
 *        dma_engine_wrapper.v
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
 *        Wrapper module for the dma_engine netlist that only defines the
 *        interface.
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
module dma_engine
  (
   // Tx
   output [63:0]  trn_td,
   output [7:0]   trn_trem_n,
   output         trn_tsof_n,
   output         trn_teof_n,
   output         trn_tsrc_rdy_n,
   output         trn_tsrc_dsc_n,
   input          trn_tdst_rdy_n,
   input          trn_tdst_dsc_n,
   output         trn_terrfwd_n,
   input [3:0]    trn_tbuf_av,
  
   // Rx
   input [63:0]   trn_rd,
   input [7:0]    trn_rrem_n,
   input          trn_rsof_n,
   input          trn_reof_n,
   input          trn_rsrc_rdy_n,
   input          trn_rsrc_dsc_n,
   output         trn_rdst_rdy_n,
   input          trn_rerrfwd_n,
   output         trn_rnp_ok_n,
   input [6:0]    trn_rbar_hit_n,
   input [7:0]    trn_rfc_nph_av,
   input [11:0]   trn_rfc_npd_av,
   input [7:0]    trn_rfc_ph_av,
   input [11:0]   trn_rfc_pd_av,
   output         trn_rcpl_streaming_n,
  
   // Host (CFG) Interface
   input [31:0]   cfg_do,
   input          cfg_rd_wr_done_n,
   output [31:0]  cfg_di,
   output [3:0]   cfg_byte_en_n,
   output [9:0]   cfg_dwaddr,
   output         cfg_wr_en_n,
   output         cfg_rd_en_n,
   output         cfg_err_cor_n,
   output         cfg_err_ur_n,
   output         cfg_err_ecrc_n,
   output         cfg_err_cpl_timeout_n,
   output         cfg_err_cpl_abort_n,
   output         cfg_err_cpl_unexpect_n,
   output         cfg_err_posted_n,
   output [47:0]  cfg_err_tlp_cpl_header,

   input          cfg_err_cpl_rdy_n,
   output         cfg_err_locked_n, 
   output         cfg_interrupt_n,
   input          cfg_interrupt_rdy_n,
   output         cfg_interrupt_assert_n,
   output [7:0]   cfg_interrupt_di,
   input [7:0]    cfg_interrupt_do,
   input [2:0]    cfg_interrupt_mmenable,
   input          cfg_interrupt_msienable,
   input          cfg_to_turnoff_n,
   output         cfg_pm_wake_n,
   input [2:0]    cfg_pcie_link_state_n,
   output         cfg_trn_pending_n,
   input [7:0]    cfg_bus_number,
   input [4:0]    cfg_device_number,
   input [2:0]    cfg_function_number,
   output [63:0]  cfg_dsn,
   input [15:0]   cfg_status,
   input [15:0]   cfg_command,
   input [15:0]   cfg_dstatus,
   input [15:0]   cfg_dcommand,
   input [15:0]   cfg_lstatus,
   input [15:0]   cfg_lcommand,
  
   // MAC tx
   output [63:0]  M_AXIS_TDATA,
   output [7:0]   M_AXIS_TSTRB,
   output         M_AXIS_TVALID,
   input          M_AXIS_TREADY,
   output         M_AXIS_TLAST,
   output [127:0] M_AXIS_TUSER,

   // MAC rx
   input [63:0]   S_AXIS_TDATA,
   input [7:0]    S_AXIS_TSTRB,
   input          S_AXIS_TVALID,
   output         S_AXIS_TREADY,
   input          S_AXIS_TLAST,
   input [127:0]  S_AXIS_TUSER,

   // AXI lite master core interface
   output         IP2Bus_MstRd_Req,
   output         IP2Bus_MstWr_Req,
   output [31:0]  IP2Bus_Mst_Addr,
   output [3:0]   IP2Bus_Mst_BE,
   output         IP2Bus_Mst_Lock,
   output         IP2Bus_Mst_Reset,
   input          Bus2IP_Mst_CmdAck,
   input          Bus2IP_Mst_Cmplt,
   input          Bus2IP_Mst_Error,
   input          Bus2IP_Mst_Rearbitrate,
   input          Bus2IP_Mst_Timeout,
   input [31:0]   Bus2IP_MstRd_d,
   input          Bus2IP_MstRd_src_rdy_n,
   output [31:0]  IP2Bus_MstWr_d,
   input          Bus2IP_MstWr_dst_rdy_n,
  
   // misc
   input          axi_clk,
   input          tx_clk,
   input          rx_clk,
   input          pcie_clk,
   input          rst,

   inout wire [35:0] chipscope_control_0,
   inout wire [35:0] chipscope_control_1
   );
   
endmodule
