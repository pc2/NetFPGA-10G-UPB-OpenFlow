/*******************************************************************************
 *
 *  NetFPGA-10G http://www.netfpga.org
 *
 *  File:
 *        nf10fops.h
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
 *        Function prototypes for nf10fops.c
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

#ifndef NF10FOPS_H
#define NF10FOPS_H

#include <linux/fs.h>
#include "nf10driver.h"

#define NF10_IOCTL_CMD_READ_STAT (SIOCDEVPRIVATE+0)
#define NF10_IOCTL_CMD_WRITE_REG (SIOCDEVPRIVATE+1)
#define NF10_IOCTL_CMD_READ_REG (SIOCDEVPRIVATE+2)

struct ioctl_write {
	uint32_t value;
	uint32_t address;
};

int nf10fops_open (struct inode *n, struct file *f);
long nf10fops_ioctl (struct file *f, unsigned int cmd, unsigned long arg);
int nf10fops_release (struct inode *n, struct file *f);

int nf10_mmap(struct file *filp, struct vm_area_struct *vma);

int nf10fops_probe(struct pci_dev *pdev, struct nf10_card *card);
int nf10fops_remove(struct pci_dev *pdev, struct nf10_card *card);

#endif
