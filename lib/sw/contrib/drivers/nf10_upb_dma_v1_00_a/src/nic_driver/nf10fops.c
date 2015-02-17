/*******************************************************************************
 *
 *  NetFPGA-10G http://www.netfpga.org
 *
 *  File:
 *        nf10fops.c
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
 *        This code creates a /dev/nf10 file that can be used to read/write
 *        AXI registers through ioctl calls. See sw/host/apps/rdaxi.c for
 *        an example of that.
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

#include "nf10fops.h"
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/pci.h>
#include <linux/sockios.h>
#include <linux/module.h>
#include <asm/uaccess.h>
#include <asm/tsc.h>
#include <linux/interrupt.h>
#include <asm/irq.h>
#include <linux/mm.h>

static struct file_operations nf10_fops={
    .owner = THIS_MODULE,
    .open = nf10fops_open,
    .unlocked_ioctl = nf10fops_ioctl,
    .release = nf10fops_release,
    .mmap = nf10_mmap
};


int nf10fops_open (struct inode *n, struct file *f){
    struct nf10_card *card = (struct nf10_card *)container_of(n->i_cdev, struct nf10_card, cdev);
    f->private_data = card;
    return 0;
}


long nf10fops_ioctl (struct file *f, unsigned int cmd, unsigned long arg) {

    struct nf10_card *card = (struct nf10_card *)f->private_data;

    switch(cmd) {

		case NF10_IOCTL_CMD_READ_STAT:
		{
			uint64_t addr, val;
			if(copy_from_user(&addr, (uint64_t*)arg, sizeof(addr)))
				return -EACCES;
			if(addr >= 4096/8)
				return -EINVAL;

			val = *(((uint64_t*)card->cfg_addr) + 16384/8 + addr);

			if(copy_to_user((uint64_t*)arg, &val, sizeof(val)))
				return -EACCES;
		} break;

		case NF10_IOCTL_CMD_WRITE_REG:

			if (atomic64_cmpxchg(&card->axi_access_state, AXI_ACCESS_UNASSIGNED, AXI_ACCESS_IOCTL) == AXI_ACCESS_UNASSIGNED) {

				struct ioctl_write cmd;
				if (copy_from_user(&cmd, (struct ioctl_write*)arg, sizeof(struct ioctl_write))) {
					atomic64_set(&card->axi_access_state, AXI_ACCESS_UNASSIGNED);
					return -EACCES;
				}

				spin_lock(&card->axi_lock);
				axi_wr(card, cmd.address, cmd.value);
				spin_unlock(&card->axi_lock);

				atomic64_set(&card->axi_access_state, AXI_ACCESS_UNASSIGNED);

			} else { // AXI bus is assigned to MMAP
				return -EFAULT;
			}
			break;

		case NF10_IOCTL_CMD_READ_REG:
		{
			if (atomic64_cmpxchg(&card->axi_access_state, AXI_ACCESS_UNASSIGNED, AXI_ACCESS_IOCTL) == AXI_ACCESS_UNASSIGNED) {
				uint64_t addr, val;

				if(copy_from_user(&addr, (uint64_t*)arg, sizeof(addr))) {

					atomic64_set(&card->axi_access_state, AXI_ACCESS_UNASSIGNED);
					return -EACCES;
				}

				spin_lock(&card->axi_lock);
				val = axi_rd(card, addr);
				spin_unlock(&card->axi_lock);

				if(copy_to_user((uint64_t*)arg, &val, sizeof(val))) {

					atomic64_set(&card->axi_access_state, AXI_ACCESS_UNASSIGNED);
					return -EACCES;
				}

				atomic64_set(&card->axi_access_state, AXI_ACCESS_UNASSIGNED);

			} else { // AXI bus is assigned to MMAP
				return -EFAULT;
			}
		} break;

		default:

			printk(KERN_ERR "nf10: unknown ioctl\n");
			return -EFAULT;
    }
    
    return 0;
}

int nf10fops_release (struct inode *n, struct file *f){
    f->private_data = NULL;
    return 0;
}

void nf10_mmap_open(struct vm_area_struct *vma)
{
	struct nf10_card *card = (struct nf10_card *)vma->vm_private_data;
	card->mmap_ref_count++;
}

void nf10_mmap_close(struct vm_area_struct *vma)
{
	struct nf10_card *card = (struct nf10_card *)vma->vm_private_data;

	if (--card->mmap_ref_count == 0)
		atomic64_cmpxchg(&card->axi_access_state, AXI_ACCESS_MMAP, AXI_ACCESS_UNASSIGNED);
}

struct vm_operations_struct nf10_mmap_ops = {
	.open =     nf10_mmap_open,
	.close =    nf10_mmap_close
};

int nf10_mmap(struct file *filp, struct vm_area_struct *vma)
{
	phys_addr_t io_address;
	unsigned long io_size, bar_size;

	struct nf10_card *card = (struct nf10_card *)filp->private_data;

	if (atomic64_cmpxchg(&card->axi_access_state, AXI_ACCESS_UNASSIGNED, AXI_ACCESS_MMAP) != AXI_ACCESS_UNASSIGNED) {

		printk(KERN_ERR "nf10: Could not open mmap - a ioctl is still in progress\n");
		return -EINVAL;
	}

	axi_wait_write_buffer_empty(card); // the user space gets an empty axi buffer
	card->mmap_ref_count = 1;

	vma->vm_ops = &nf10_mmap_ops;
	vma->vm_flags |= VM_IO | VM_DONTEXPAND | VM_DONTDUMP;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	vma->vm_private_data = filp->private_data;

	io_size = vma->vm_end - vma->vm_start; // user space size request in bytes (ignored)
	io_address = pci_resource_start(card->pdev, 0) + PAGE_SIZE * 2; // skip two pages at bar0 to get the axi regs
	bar_size = pci_resource_len(card->pdev, 0); // bar0 size in bytes (ignored)

	if (io_remap_pfn_range(
		vma,
		vma->vm_start,
		io_address >> PAGE_SHIFT, // if it does not match with user space request: error
		PAGE_SIZE * 2, // size in bytes
		vma->vm_page_prot
	)) {

		atomic64_set(&card->axi_access_state, AXI_ACCESS_UNASSIGNED);
		printk(KERN_ERR "nf10: Could not map IO memory to user space\n");
		return -EINVAL;
	}

	return 0;
}

int nf10fops_probe(struct pci_dev *pdev, struct nf10_card *card){
    int err;
    
    char devname[6] = "nf10 ";
    devname[4] = 'a' + (char)card->card_id;

    err = alloc_chrdev_region(&card->devno, 0, 1, card->card_name);
    if (err){
        printk(KERN_ERR "nf10: Error allocating chrdev\n");
        return err;
    }
    cdev_init(&card->cdev, &nf10_fops);
    card->cdev.owner = THIS_MODULE;
    card->cdev.ops = &nf10_fops;
    err = cdev_add(&card->cdev, card->devno, 1);
    if (err){
        printk(KERN_ERR "nf10: Error adding /dev/nf10\n");
        return err;
    }

    card->dev_class = class_create(THIS_MODULE, card->card_name);
    device_create(card->dev_class, NULL, card->devno, NULL, card->card_name);
    return 0;
}

int nf10fops_remove(struct pci_dev *pdev, struct nf10_card *card){
    device_destroy(card->dev_class, card->devno);
    class_unregister(card->dev_class);
    class_destroy(card->dev_class);
    cdev_del(&card->cdev);
    unregister_chrdev_region(card->devno, 1);
    return 0;
}
