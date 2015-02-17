/*******************************************************************************
 *
 *  NetFPGA-10G http://www.netfpga.org
 *
 *  File:
 *        nf10priv.c
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
 *        These functions control the card tx/rx operation. 
 *        nf10priv_xmit -- gets called for every transmitted packet
 *                         (on any nf interface)
 *        work_handler  -- gets called when the interrupt handler puts work
 *                         on the queue
 *        nf10priv_send_rx_dsc -- allocates and sends a receive descriptor
 *                                to the nic
 *
 *        There also exists a LOOPBACK_MODE (enabled by defining constant
 *        LOOPBACK_MODE) that allows the driver to be tested on a single
 *        machine. This mode, at receive, flips the last bit in the second
 *        to last octet of the source and destination IP addresses. (e.g.
 *        address 192.168.2.1 is converted to 192.168.3.1 and vice versa).
 *
 *        An example configuration that has been tested with loopback between 
 *        interfaces 0 and 3 (must add static ARP entries, because ARPs
 *        aren't fixed by the LOOPBACK_MODE):
 *            ifconfig nf0 192.168.2.11;
 *            ifconfig nf3 192.168.3.12;
 *            arp -s 192.168.2.12 00:4E:46:31:30:03;
 *            arp -s 192.168.3.11 00:4E:46:31:30:00;
 *
 *            "ping 192.168.2.12" -- should now work with packets going over
 *                                   the wire.
 *
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

#include "nf10priv.h"
#include "nf10driver.h"
#include <linux/spinlock.h>
#include <linux/pci.h>
#include <linux/string.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/if_ether.h>
#include <net/ip.h>
#include <net/tcp.h>

#define SK_BUFF_ALLOC_SIZE (MTU + 6+6+4+2) // dest. MAC (6), src. MAC (6), opt. 802.1Q tag (4), Ethertype (2), no FCS (FCS is removed by Xilinx XGMAC)

netdev_tx_t nf10priv_xmit(struct nf10_card *card, struct sk_buff *skb, int port) {

    uint64_t pkt_addr = 0, pkt_addr_fixed = 0;
    uint64_t dsc_addr = 0, dsc_index = 0;
    uint64_t cl_size = (skb->len + 2*63) / 64; // need engouh room for data in any alignment case
    uint64_t port_decoded = 0;
    uint64_t dsc_l0, dsc_l1;
    uint64_t dma_addr;

    // meet minimum size requirement
    if(skb->len < 60){
        skb_pad(skb, 60 - skb->len);
        skb_put(skb, 60 - skb->len);
    }

    if(skb->len > 65535 || cl_size > card->mem_tx_pkt.cl_size) {
        printk(KERN_ERR "nf10: packet too big, dropping");
        dev_kfree_skb_any(skb);
        card->ndev[port]->stats.tx_errors++;
        return NETDEV_TX_OK;
    }

    // packet buffer management
    spin_lock(&card->tx_lock);

    // make sure we fit in the descriptor ring and packet buffer
    if( (atomic64_read(&card->mem_tx_dsc.cnt) + 1 <= card->mem_tx_dsc.cl_size) &&
        (atomic64_read(&card->mem_tx_pkt.cnt) + cl_size <= card->mem_tx_pkt.cl_size)) {
        
        pkt_addr = card->mem_tx_pkt.wr_ptr;
        card->mem_tx_pkt.wr_ptr = (pkt_addr + 64*cl_size) & card->mem_tx_pkt.mask;

        dsc_addr = card->mem_tx_dsc.wr_ptr;
        card->mem_tx_dsc.wr_ptr = (dsc_addr + 64) & card->mem_tx_dsc.mask;

        // get physical address of the data
        dma_addr = pci_map_single(card->pdev, skb->data, skb->len, PCI_DMA_TODEVICE);
        
        if(pci_dma_mapping_error(card->pdev, dma_addr)){

            printk(KERN_ERR "nf10: dma mapping error");

            card->ndev[port]->stats.tx_errors++;
            spin_unlock(&card->tx_lock);
            dev_kfree_skb_any(skb);
            return NETDEV_TX_OK;
        }

        atomic64_inc(&card->mem_tx_dsc.cnt);
        atomic64_add(cl_size, &card->mem_tx_pkt.cnt);

    } else { // no space in descriptor ring

    	card->ndev[port]->stats.tx_errors++;

    	if (atomic64_read(&card->mem_tx_dsc.cnt)) // there is a TX done interrupt pending
    		netif_stop_queue(card->ndev[port]); // stop the queue

   		spin_unlock(&card->tx_lock);
		return NETDEV_TX_BUSY;
    }
    
    // update stats
	card->ndev[port]->stats.tx_packets++;
	card->ndev[port]->stats.tx_bytes += skb->len;

    dsc_index = dsc_addr / 64;
    
    // figure out ports
	port_decoded = port & 7; // source vport

    // fix address for alignment issues
    pkt_addr_fixed = pkt_addr + (dma_addr & 0x3fULL);

    // prepare TX descriptor
    dsc_l0 = ((uint64_t)skb->len << 48) + ((uint64_t)port_decoded << 32) + (pkt_addr_fixed & 0xffffffff);
    dsc_l1 = dma_addr;

    // book keeping
    card->tx_bk_dma_addr[dsc_index] = dma_addr;
    card->tx_bk_skb[dsc_index] = skb;
    card->tx_bk_size[dsc_index] = cl_size;
    card->tx_bk_port[dsc_index] = port;

    // write to the card
    *(((uint64_t*)card->tx_dsc) + 8 * dsc_index + 0) = dsc_l0;
    mmiowb();
    *(((uint64_t*)card->tx_dsc) + 8 * dsc_index + 1) = dsc_l1;
    mmiowb();

    spin_unlock(&card->tx_lock);

    return NETDEV_TX_OK;
}

void process_rx_skb(struct nf10_card *card, int port, struct sk_buff *skb)
{
	if (((struct nf10_ndev_priv*)netdev_priv(card->ndev[port]))->port_up) {

		// update skb with port information
		skb->dev = card->ndev[port];
		skb->protocol = eth_type_trans(skb, card->ndev[port]);
		skb->ip_summed = CHECKSUM_NONE;

		// update stats
		card->ndev[port]->stats.rx_packets++;
		card->ndev[port]->stats.rx_bytes += skb->len;

		napi_gro_receive(&card->napi, skb);

	} else
		dev_kfree_skb_any(skb);
}

void nf10_handle_rx_irq(struct nf10_card *card, unsigned int *work_done, unsigned int work_to_do)
{
	uint64_t rx_int;
	uint64_t addr;
	uint64_t index;
	struct sk_buff *skb;
	uint64_t len;
	uint8_t out_vport;
	uint16_t rx_id;

	while ((((rx_int = *(((uint64_t*)card->host_rx_dne_ptr) + (card->host_rx_dne.rd_ptr)/8 + 7)) >> 48) & 0xffff) != 0xffff ) {

		if (*work_done >= work_to_do)
			break;
		(*work_done)++;

		// manage host completion buffer
		addr = card->host_rx_dne.rd_ptr;
		card->host_rx_dne.rd_ptr = (addr + 64) & card->host_rx_dne.mask;
		index = addr / 64;

		// invalidate host rx completion buffer
		*(((uint64_t*)card->host_rx_dne_ptr) + index * 8 + 7) = 0xffffffffffffffffULL;
		mb();

		// skb is now ready
		skb = card->rx_bk_skb[index];
		dma_unmap_single(&card->pdev->dev, card->rx_bk_dma_addr[index], SK_BUFF_ALLOC_SIZE, PCI_DMA_FROMDEVICE);
		atomic64_sub(card->rx_bk_size[index], &card->mem_rx_pkt.cnt);
		atomic64_dec(&card->mem_rx_dsc.cnt);

		// give the card a new RX descriptor
		nf10priv_send_rx_dsc(card);

		rx_id = (rx_int >> 32) & 0xffff;
		if (rx_id != card->rx_bk_id[index]) {

			printk(KERN_ERR "%s recv: DMA descriptors got out of sync. Received from HW %u, expected in SW %u\n", card->card_name, (uint32_t)rx_id, (uint32_t)card->rx_bk_id[index]);
			dev_kfree_skb_any(skb);

		} else {

			//printk(KERN_ERR "%s recv: Received data for descriptor index %u, packet ID %u\n", card->card_name, (uint32_t)index, (uint32_t)rx_id);

			// read data from the completion buffer
			len = rx_int & 0xffff;
			out_vport = (rx_int >> 16) & 0xff; // 8 bit out_vport (one hot)

			if (len >= 60 && len <= SK_BUFF_ALLOC_SIZE) { // FCS (CRC) is already removed

				skb_put(skb, len); // extend the buffer to the packet length

				switch (out_vport) {

					case 0:
						printk(KERN_ERR "nf10 receive: no out_vport bit set\n");
						dev_kfree_skb_any(skb);
						break;
					case 1:
						process_rx_skb(card, 0, skb);
						break;
					case 2:
						process_rx_skb(card, 1, skb);
						break;
					case 4:
						process_rx_skb(card, 2, skb);
						break;
					case 8:
						process_rx_skb(card, 3, skb);
						break;
					case 16:
						process_rx_skb(card, 4, skb);
						break;
					case 32:
						process_rx_skb(card, 5, skb);
						break;
					case 64:
						process_rx_skb(card, 6, skb);
						break;
					case 128:
						process_rx_skb(card, 7, skb);
						break;

					default: // more than one bit from the 8 bits is set => received a multicast
					{
						int8_t original_skb_goes_to_port = -1;

						int8_t i;
						for (i = 0; i < 8; i++) {

							if (out_vport & (1UL << i)) {

								if (i < VPORTS) { // valid port?

									if (original_skb_goes_to_port == -1) {

										original_skb_goes_to_port = i;

									} else {

										process_rx_skb(card, i, skb_copy(skb, GFP_ATOMIC));
									}
								}
							}
						}

						if (original_skb_goes_to_port != -1) {
							process_rx_skb(card, original_skb_goes_to_port, skb);
						} else {
							dev_kfree_skb_any(skb);
						}

					} break;
				}
			} else {
				printk(KERN_ERR "%s recv: Invalid packet size: %u. Destination port mask: 0x%02x. Dropping packet.\n", card->card_name, (uint32_t)len, (uint32_t)out_vport);
				dev_kfree_skb_any(skb);
			}
		}
	}
}

void nf10_handle_tx_irq(struct nf10_card *card)
{
	uint64_t addr;
	uint64_t index;
	int i;

	while (NF10_TX_INT(card)) {

		// manage host completion buffer
		addr = card->host_tx_dne.rd_ptr;
		card->host_tx_dne.rd_ptr = (addr + 64) & card->host_tx_dne.mask;
		index = addr / 64;

		// clean up the skb
		pci_unmap_single(card->pdev, card->tx_bk_dma_addr[index], card->tx_bk_skb[index]->len, PCI_DMA_TODEVICE);
		dev_kfree_skb_any(card->tx_bk_skb[index]);

		spin_lock(&card->tx_lock);

		atomic64_sub(card->tx_bk_size[index], &card->mem_tx_pkt.cnt);
		atomic64_dec(&card->mem_tx_dsc.cnt);

		// invalidate host tx completion buffer
		*(((uint32_t*)card->host_tx_dne_ptr) + index * 16) = 0xffffffff;
		mb();

		// restart queue if needed
		for(i = 0; i < VPORTS; i++){
			if(netif_queue_stopped(card->ndev[i]))
				netif_wake_queue(card->ndev[i]);
		}

		spin_unlock(&card->tx_lock);
	}
}

int nf10priv_send_rx_dsc(struct nf10_card *card){
    struct sk_buff *skb;
    dma_addr_t dma_addr;
    uint64_t pkt_addr = 0, pkt_addr_fixed = 0;
    uint64_t dsc_addr = 0, dsc_index = 0;
    uint64_t cl_size = (SK_BUFF_ALLOC_SIZE + 66) / 64;
    uint64_t dsc_l0, dsc_l1;

    // packet buffer management

    // make sure we fit in the descriptor ring and packet buffer
    if( (atomic64_read(&card->mem_rx_dsc.cnt) + 1 <= card->mem_rx_dsc.cl_size) &&
        (atomic64_read(&card->mem_rx_pkt.cnt) + cl_size <= card->mem_rx_pkt.cl_size)) {
        
        skb = dev_alloc_skb(SK_BUFF_ALLOC_SIZE + 2);
        if(!skb) {
            printk(KERN_ERR "nf10: skb alloc failed\n");
            return -1;
        }
        skb_reserve(skb, 2); /* align IP on 16B boundary */   

        pkt_addr = card->mem_rx_pkt.wr_ptr;
        card->mem_rx_pkt.wr_ptr = (pkt_addr + 64*cl_size) & card->mem_rx_pkt.mask;

        dsc_addr = card->mem_rx_dsc.wr_ptr;
        card->mem_rx_dsc.wr_ptr = (dsc_addr + 64) & card->mem_rx_dsc.mask;

        atomic64_inc(&card->mem_rx_dsc.cnt);
        atomic64_add(cl_size, &card->mem_rx_pkt.cnt);
        
    } 
    else{
        return -1;
    }

    // physical address
    dma_addr = dma_map_single(&card->pdev->dev, skb->data, SK_BUFF_ALLOC_SIZE, PCI_DMA_FROMDEVICE);
    if (dma_mapping_error(&card->pdev->dev, dma_addr))
    	return -1;

    dsc_index = dsc_addr / 64;

    // fix address for alignment issues
    pkt_addr_fixed = pkt_addr + (dma_addr & 0x3ULL);

    // prepare RX descriptor
    dsc_l0 = ((uint64_t)SK_BUFF_ALLOC_SIZE << 48) | ((uint64_t)card->rx_id << 32) | (pkt_addr_fixed & 0xffffffff);
    dsc_l1 = dma_addr;

    // book keeping
    card->rx_bk_dma_addr[dsc_index] = dma_addr;
    card->rx_bk_skb[dsc_index] = skb;
    card->rx_bk_size[dsc_index] = cl_size;
    card->rx_bk_id[dsc_index] = card->rx_id++;

	//printk(KERN_ERR "%s recv: Sending new DMA descriptor to slot %u, ID: %u\n", card->card_name, (uint32_t)dsc_index, (uint32_t)card->rx_bk_id[dsc_index]);

    // write to the card
	*(((uint64_t*)card->rx_dsc) + 8 * dsc_index + 0) = dsc_l0;
    mmiowb();
    *(((uint64_t*)card->rx_dsc) + 8 * dsc_index + 1) = dsc_l1;
    mmiowb();

    return 0;
}
