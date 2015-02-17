/*******************************************************************************
 *
 *  NetFPGA-10G http://www.netfpga.org
 *
 *  File:
 *        nf10driver.h
 *
 *  Project:
 *        nic
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
 *        Top level header file for the nic driver. Contains definitions for
 *        nic datastructures.
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

#ifndef NF10DRIVER_H
#define NF10DRIVER_H

#define PCI_VENDOR_ID_NF10 0x6f5e
#define PCI_DEVICE_ID_NF10 0xdead

#include "nf10config.h"
#include <linux/netdevice.h> 
#include <linux/cdev.h>
#include <asm/atomic.h>
#include <linux/workqueue.h>
#include <linux/spinlock_types.h>

#define MTU 9000

#define NF10_DISABLE_INT(card) \
{ \
	*(((uint64_t*)card->cfg_addr)+25) = 0; \
	mmiowb(); \
	*(((uint64_t*)card->cfg_addr)+26) = 0; \
	mmiowb(); \
}

#define NF10_ENABLE_INT(card) \
{ \
    *(((uint64_t*)card->cfg_addr)+25) = 1; \
    mmiowb(); \
    *(((uint64_t*)card->cfg_addr)+26) = 1; \
	mmiowb(); \
}

#define NF10_RX_INT(card) (((*(((uint64_t*)card->host_rx_dne_ptr) + (card->host_rx_dne.rd_ptr)/8 + 7) >> 48) & 0xffff) != 0xffff)
#define NF10_TX_INT(card) ((*(((uint32_t*)card->host_tx_dne_ptr) + (card->host_tx_dne.rd_ptr)/4) & 0xffff) == 1 )

struct nf10mem{
    uint64_t wr_ptr;
    uint64_t rd_ptr;
    atomic64_t cnt;
    uint64_t mask;
    uint64_t cl_size;
} __attribute__ ((aligned(64)));

struct nf10_card{

	struct napi_struct napi;

    volatile void *cfg_addr;   // kernel virtual address of the card BAR0 space
    volatile void *tx_dsc;     // kernel virtual address of the card tx descriptor space
    volatile void *rx_dsc;     // kernel virtual address of the card rx descriptor space

    uint64_t tx_dsc_mask;
    uint64_t rx_dsc_mask;
    uint64_t tx_pkt_mask;
    uint64_t rx_pkt_mask;
    uint64_t tx_dne_mask;
    uint64_t rx_dne_mask;

    void *host_tx_dne_ptr;         // virtual address
    uint64_t host_tx_dne_dma;      // physical address

    void *host_rx_dne_ptr;         // virtual address
    uint64_t host_rx_dne_dma;      // physical address
    
    struct pci_dev *pdev;
    struct cdev cdev; // char device structure (for /dev/nf10)

    struct net_device *ndev[VPORTS]; // network devices
    
    // memory buffers
    struct nf10mem mem_tx_dsc;
    struct nf10mem mem_tx_pkt;
    struct nf10mem mem_rx_dsc;
    struct nf10mem mem_rx_pkt;
    struct nf10mem host_tx_dne;
    struct nf10mem host_rx_dne;

    // tx book keeping
    struct sk_buff  **tx_bk_skb;
    uint64_t *tx_bk_dma_addr;
    uint64_t *tx_bk_size;
    uint64_t *tx_bk_port;
    struct sk_buff  **rx_bk_skb;
    uint64_t *rx_bk_dma_addr;
    uint64_t *rx_bk_size;
    uint16_t *rx_bk_id;
    uint16_t rx_id;

    spinlock_t tx_lock, axi_lock;

    int card_id;
    char card_name[6];

    dev_t devno;
    struct class *dev_class;

    int axi_write_buffer_level;
    atomic64_t axi_access_state;

    uint32_t mmap_ref_count;
};

#define AXI_ACCESS_UNASSIGNED 0
#define AXI_ACCESS_MMAP 1
#define AXI_ACCESS_IOCTL 2

struct nf10_ndev_priv{
    struct nf10_card *card;
    int port_num;
    int port_up;
};

int nf10_ael2005_phy_configuration(struct nf10_card *card);

void axi_wait_write_buffer_empty(struct nf10_card *card);
uint32_t axi_rd(struct nf10_card *card, uint32_t address);
void axi_wr(struct nf10_card *card, uint32_t address, uint32_t value);

#endif
