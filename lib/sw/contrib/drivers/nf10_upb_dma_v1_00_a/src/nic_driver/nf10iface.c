/*******************************************************************************
 *
 *  NetFPGA-10G http://www.netfpga.org
 *
 *  File:
 *        nf10iface.c
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
 *        This code creates and handles Ethernet interfaces presented to linux
 *        as nf0-nf3. Both interrupt handles (int_handler) and transmit
 *        function (nf10i_tx) delegate their work to functions in nf10priv.c
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

#include "nf10iface.h"
#include "nf10driver.h"
#include "nf10priv.h"
#include <asm/atomic.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/if_ether.h>


irqreturn_t int_handler(int irq, void *dev_id){
    struct pci_dev *pdev = dev_id;
    struct nf10_card *card = (struct nf10_card*)pci_get_drvdata(pdev);

    if (napi_schedule_prep(&card->napi)) {
    	//NF10_DISABLE_INT(card); // TODO: It seems we loose an interrupt when we enable this. Fix this!
    	__napi_schedule(&card->napi);
    }

    return IRQ_HANDLED;
}

int nf10_poll(struct napi_struct *napi, int budget)
{
	struct nf10_card *card = container_of(napi, struct nf10_card, napi);

	int work_done = 0;

	nf10_handle_tx_irq(card);
	nf10_handle_rx_irq(card, &work_done, budget);

	if (work_done < budget) {
		napi_complete(napi);
		NF10_ENABLE_INT(card); // enable interrupts - but maybe we already lost an interrupt !

		if (NF10_RX_INT(card) || NF10_TX_INT(card))
			napi_reschedule(napi); // maybe we lost an interrupt - reschedule

	}

	return work_done;
}

static netdev_tx_t nf10i_tx(struct sk_buff *skb, struct net_device *dev) {

    struct nf10_card* card = ((struct nf10_ndev_priv*)netdev_priv(dev))->card;
    int port = ((struct nf10_ndev_priv*)netdev_priv(dev))->port_num;

    return nf10priv_xmit(card, skb, port);
}

static int nf10i_ioctl(struct net_device *dev, struct ifreq *rq, int cmd){
    return 0;
}

static int nf10i_open(struct net_device *dev){
    ((struct nf10_ndev_priv*)netdev_priv(dev))->port_up = 1;
    return 0;    
}

static int nf10i_stop(struct net_device *dev){
    ((struct nf10_ndev_priv*)netdev_priv(dev))->port_up = 0;
    return 0;
}

static int nf10i_set_mac(struct net_device *dev, void *a){
    struct sockaddr *addr = (struct sockaddr *) a;
    
    if (!is_valid_ether_addr(addr->sa_data))
        return -EADDRNOTAVAIL;
    
    memcpy(dev->dev_addr, addr->sa_data, ETH_ALEN);
    
    return 0;
}

static struct net_device_stats *nf10i_stats(struct net_device *dev){
    return &dev->stats;
}

static const struct net_device_ops nf10_ops = {
    .ndo_open            = nf10i_open,
    .ndo_stop            = nf10i_stop,
    .ndo_do_ioctl        = nf10i_ioctl,
    .ndo_get_stats       = nf10i_stats,
    .ndo_start_xmit      = nf10i_tx,
    .ndo_set_mac_address = nf10i_set_mac
};

// init called by alloc_netdev
static void nf10iface_init(struct net_device *dev)
{
  ether_setup(dev); /* assign some of the fields */
  
  dev->netdev_ops      = &nf10_ops;
  dev->watchdog_timeo  = msecs_to_jiffies(5000);
  dev->mtu             = MTU;

}

int nf10iface_probe(struct pci_dev *pdev, struct nf10_card *card){
    int ret = -ENODEV;
    int i;

    struct net_device *netdev;
    
    char devname_virtual[9] = "nf10 v%d";
    char devname[8] = "nf10 %d";

    devname_virtual[4] = 'a' + (char)card->card_id;
    devname[4] = 'a' + (char)card->card_id;

    // Set up the network device...
    for (i = 0; i < VPORTS; i++){
        netdev = card->ndev[i] = alloc_netdev(
        	sizeof(struct nf10_ndev_priv),
            (i == VPORTS-1 ? devname : devname_virtual), // the last device will get nf10x0
            nf10iface_init
        );

        if(netdev == NULL){
            printk(KERN_ERR "nf10: Could not allocate ethernet device.\n");
            ret = -ENOMEM;
            goto err_out_free_dev;
        }
        netdev->irq = pdev->irq;

        ((struct nf10_ndev_priv*)netdev_priv(netdev))->card     = card;
        ((struct nf10_ndev_priv*)netdev_priv(netdev))->port_num = i;
        ((struct nf10_ndev_priv*)netdev_priv(netdev))->port_up  = 0;

        // assign some made up MAC adddr
        get_random_bytes(netdev->dev_addr, ETH_ALEN-1);
        netdev->dev_addr[0] &= 0xfe; // turn off broadcast/multicast bit
        netdev->dev_addr[ETH_ALEN-1] = ((card->card_id & 0xf) << 8) | (i & 0xf);

        if(register_netdev(netdev)){
            printk(KERN_ERR "nf10: register_netdev failed\n");
        }

        netif_start_queue(netdev);
    }

    // give some descriptors to the card
    for(i = 0; i < card->mem_rx_dsc.cl_size-2; i++){
        nf10priv_send_rx_dsc(card);
    }

    netif_napi_add(netdev, &card->napi, nf10_poll, 64);
    napi_enable(&card->napi);

    // request IRQ
    if(request_irq(pdev->irq, int_handler, 0, card->card_name, pdev) != 0){
        printk(KERN_ERR "nf10: request_irq failed\n");
        goto err_out_free_dev;
    }

    NF10_ENABLE_INT(card);

    // yay
    return 0;
    
    // fail
 err_out_free_dev:
    for (i = 0; i < VPORTS; i++){
        if(card->ndev[i]){
            unregister_netdev(card->ndev[i]);
            free_netdev(card->ndev[i]);
        }
    }

    return ret;
}

int nf10iface_remove(struct pci_dev *pdev, struct nf10_card *card){

	int i;

	napi_disable(&card->napi);
	NF10_DISABLE_INT(card);
	free_irq(pdev->irq, pdev);

    for (i = 0; i < VPORTS; i++){
        if(card->ndev[i]){
            unregister_netdev(card->ndev[i]);
            free_netdev(card->ndev[i]);
        }
    }

    return 0;
}
