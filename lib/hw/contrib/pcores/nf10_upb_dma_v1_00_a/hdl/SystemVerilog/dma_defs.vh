/*******************************************************************************
 *
 *  NetFPGA-10G http://www.netfpga.org
 *
 *  File:
 *        dma_defs.vh
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
 *        Top level include file for the DMA engine. It contains constant
 *        definitions that are used throughout the design.
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

// synthesise debug logic for PCIe (comment out for no debug logic)
//`define DEBUG_PCIE

// number of physical ports
`define NUM_PORTS 1

// max bit size of the memory address
`define MEM_ADDR_BITS 20

// depth of the tx pending queue
`define TX_PENDING_DEPTH 32

// interface write queue parameters
`define WR_Q_WIDTH (90 + `MEM_ADDR_BITS)
`define WR_Q_DEPTH 32
// interface read queue parameters
`define RD_Q_WIDTH (84 + `MEM_ADDR_BITS)
`define RD_Q_DEPTH 32
// interface completion queue parameters
`define CM_Q_WIDTH (75 + `MEM_ADDR_BITS)
`define CM_Q_DEPTH 8

// pcie tx queue parameters
`define PCIE_WR_Q_WIDTH (94 + `MEM_ADDR_BITS)
`define PCIE_WR_Q_DEPTH 8
`define PCIE_RD_Q_WIDTH (88 + `MEM_ADDR_BITS)
`define PCIE_RD_Q_DEPTH 8
`define PCIE_CM_Q_WIDTH (59 + `MEM_ADDR_BITS)
`define PCIE_CM_Q_DEPTH 8

// config cross clock domain fifo depths
`define CFG_X_AXI_DEPTH 64

// mem valid write cross domain fifo depth
`define MEM_VALID_X_DEPTH 4

// memory sizes (in number of cache lines)
`define MEM_N_TX_DSC 32
`define MEM_N_TX_PKT 512
`define MEM_N_TX_DNE 32
`define MEM_N_RX_DSC 32
`define MEM_N_RX_PKT 512
`define MEM_N_RX_DNE 32

// config memory address size
`define CFG_ADDR_BITS 14
// stats memory address size
`define STATS_ADDR_BITS 12 // (not increased to 14)

// bar0: cfg and stats 14 bits both (16k)
`define BAR0_MEM_SELECT_HI 14
`define BAR0_MEM_SELECT_LO 14
`define BAR0_ADDR_SELECT_HI 13
`define BAR0_ADDR_SELECT_LO 0

// bar2: up to 1MB memories (1MB addr space allocated)
`define BAR2_MEM_SELECT_HI 20
`define BAR2_MEM_SELECT_LO 20
`define BAR2_ADDR_SELECT_HI 19
`define BAR2_ADDR_SELECT_LO 0

// Number of bits to store port number
`define PORT_BITS ((`NUM_PORTS==1) ? 1 : $clog2(`NUM_PORTS))

// Memory IDs
`define ID_MEM_CFG 0
`define ID_MEM_STAT 1
`define ID_MEM_TX_DSC 2
`define ID_MEM_RX_DSC 3
`define ID_MEM_TX_DNE 4
`define ID_MEM_RX_DNE 5
`define ID_MEM_TX_PKT 6
`define ID_MEM_RX_PKT 7
