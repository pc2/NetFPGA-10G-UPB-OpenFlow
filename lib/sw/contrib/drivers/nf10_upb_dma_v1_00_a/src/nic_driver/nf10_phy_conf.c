/*******************************************************************************
 *
 *  NetFPGA-10G http://www.netfpga.org
 *
 *  File:
 *        nf10_phy_conf.c
 *
 *  Project:
 *        nic
 *
 *  Author:
 *        M. Forconesi (Cambridge Team)
 *
 *        Modifications for the UPB OpenFlow Switch project:
 *         Jörg Niklas, osjsn@niklasfamily.de
 *         Project Group "On-the-Fly Networking for Big Data"
 *         Computer Engineering Group, University of Paderborn
 *
 *  Description:
 *        Top level file for the configuration of the AEL2005 PHY chips on the
 *        NetFPGA-10G board.
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

#include <linux/kernel.h>
#include "nf10driver.h"
#include "nf10_phy_conf.h"

static inline void ael2005_sleep(int ms) {
    //msleep(ms);
    usleep_range(ms*20,ms*200); // speed up by factor 50...
}

static int XEmacLite_PhyRead(struct nf10_card *card, uint32_t PhyAddress, uint32_t RegNum, uint32_t opcode, uint32_t clause, uint16_t *PhyDataPtr) {
    uint32_t PhyAddrReg;
    uint32_t rd_address;
    uint32_t wr_address;
    uint32_t rd_value;

    //Verify MDIO master status.
    rd_address = MDIO_BASE_ADDR + XEL_MDIOCNTR_OFFSET;
    if ((axi_rd(card, rd_address) & XEL_MDIOCNTR_STATUS_MASK)  != 0) {
        rd_value = axi_rd(card, rd_address);
        return -1;
    }

    PhyAddrReg = ((((PhyAddress << XEL_MDIO_ADDRESS_SHIFT) & XEL_MDIO_ADDRESS_MASK) | RegNum) | opcode | clause);
    wr_address = MDIO_BASE_ADDR + XEL_MDIOADDR_OFFSET;
    axi_wr(card, wr_address, PhyAddrReg);

    //Enable MDIO and start the transfer.
    rd_address = MDIO_BASE_ADDR + XEL_MDIOCNTR_OFFSET;
    rd_value = axi_rd(card, rd_address);
    wr_address = MDIO_BASE_ADDR + XEL_MDIOCNTR_OFFSET;
    axi_wr(card, wr_address, rd_value | XEL_MDIOCNTR_STATUS_MASK | XEL_MDIOCNTR_ENABLE_MASK);

    //Wait till the completion of transfer.
    rd_address = MDIO_BASE_ADDR + XEL_MDIOCNTR_OFFSET;
    ael2005_sleep(2);
    while ((axi_rd(card, rd_address) & XEL_MDIOCNTR_STATUS_MASK) != 0);

    //Read data from MDIO read data register.
    rd_address = MDIO_BASE_ADDR + XEL_MDIORD_OFFSET;
    *PhyDataPtr = (uint16_t)axi_rd(card, rd_address);

    //Disable the MDIO.
    rd_address = MDIO_BASE_ADDR + XEL_MDIOCNTR_OFFSET;
    rd_value = axi_rd(card, rd_address);
    wr_address = MDIO_BASE_ADDR + XEL_MDIOCNTR_OFFSET;
    axi_wr(card, wr_address, rd_value & ~XEL_MDIOCNTR_ENABLE_MASK);

    return 0;
}

static int XEmacLite_PhyWrite(struct nf10_card *card, uint32_t PhyAddress, uint32_t RegNum, uint32_t opcode, uint32_t clause, uint16_t PhyData) {
    uint32_t PhyAddrReg;
    uint32_t rd_address;
    uint32_t wr_address;
    uint32_t rd_value;

    //Verify MDIO master status.
    rd_address = MDIO_BASE_ADDR + XEL_MDIOCNTR_OFFSET;
    if ((axi_rd(card, rd_address) & XEL_MDIOCNTR_STATUS_MASK)  != 0) {
        printk(KERN_ERR "nf10: AEL2005 Device Busy\n");
        return -1;
    }

    PhyAddrReg = ((((PhyAddress << XEL_MDIO_ADDRESS_SHIFT) & XEL_MDIO_ADDRESS_MASK) | RegNum) | opcode | clause);
    wr_address = MDIO_BASE_ADDR + XEL_MDIOADDR_OFFSET;
    axi_wr(card, wr_address, PhyAddrReg);

    //Write data to MDIO write data register
    wr_address = MDIO_BASE_ADDR + XEL_MDIOWR_OFFSET;
    axi_wr(card, wr_address, (uint32_t)PhyData);

    //Enable MDIO and start the transfer
    rd_address = MDIO_BASE_ADDR + XEL_MDIOCNTR_OFFSET;
    rd_value = axi_rd(card, rd_address);
    wr_address = MDIO_BASE_ADDR + XEL_MDIOCNTR_OFFSET;
    axi_wr(card, wr_address, rd_value | XEL_MDIOCNTR_STATUS_MASK | XEL_MDIOCNTR_ENABLE_MASK);

    //Wait till the completion of transfer.
    rd_address = MDIO_BASE_ADDR + XEL_MDIOCNTR_OFFSET;
    ael2005_sleep(2);
    while ((axi_rd(card, rd_address) & XEL_MDIOCNTR_STATUS_MASK) != 0);

    //Disable the MDIO.
    rd_address = MDIO_BASE_ADDR + XEL_MDIOCNTR_OFFSET;
    rd_value = axi_rd(card, rd_address);
    wr_address = MDIO_BASE_ADDR + XEL_MDIOCNTR_OFFSET;
    axi_wr(card, wr_address, rd_value & ~XEL_MDIOCNTR_ENABLE_MASK);
    
    return 0;
}

static int ael2005_read (struct nf10_card *card, uint32_t PhyAddr, uint32_t PhyDev, uint16_t address, uint16_t *data){
    XEmacLite_PhyWrite(card, PhyAddr, PhyDev, XEL_MDIO_OP_45_ADDRESS, XEL_MDIO_CLAUSE_45, address);
    XEmacLite_PhyRead(card, PhyAddr, PhyDev, XEL_MDIO_OP_45_READ, XEL_MDIO_CLAUSE_45, data);
    ael2005_sleep(5);
    return 0;
}

static int ael2005_write (struct nf10_card *card, uint32_t PhyAddr, uint32_t PhyDev, uint16_t address, uint16_t data){
    XEmacLite_PhyWrite(card, PhyAddr, PhyDev, XEL_MDIO_OP_45_ADDRESS, XEL_MDIO_CLAUSE_45, address);
    XEmacLite_PhyWrite(card, PhyAddr, PhyDev, XEL_MDIO_OP_45_WRITE, XEL_MDIO_CLAUSE_45, data);
    ael2005_sleep(5);
    return 0;
}

static int ael2005_i2c_read (struct nf10_card *card, uint32_t PhyAddress, uint16_t dev_addr, uint16_t word_addr, uint16_t *data){
    uint16_t stat;
    int i;
    ael2005_write(card, PhyAddress, 1, AEL_I2C_CTRL, (dev_addr << 8) | (1 << 8) | word_addr);
    for (i = 0; i < 20; i++){
        ael2005_sleep(2);
        ael2005_read  (card, PhyAddress, 1, AEL_I2C_STAT, &stat);
        if ((stat & 3) == 1){
            ael2005_read(card, PhyAddress, 1, AEL_I2C_DATA, &stat);
            *data = stat >> 8;
            return 0;
        }
    }
    return -1;
}

static int ael2005_initialize(struct nf10_card *card, uint32_t dev, int mode){
    int size, i;

    // Step 1
    size = sizeof(reset) / sizeof(uint16_t);
    for(i = 0; i < size; i+=2) ael2005_write(card, dev, PMA_MDIO_DEVICE_ADDRESS, reset[i], reset[i+1]);
    ael2005_sleep(5);

    // Step 2
    if(mode == MODE_SR){
        size = sizeof(sr_edc) / sizeof(uint16_t);
        for(i = 0; i < size; i+=2) ael2005_write(card, dev, PMA_MDIO_DEVICE_ADDRESS, sr_edc[i], sr_edc[i+1]);
    }
    else if (mode == MODE_TWINAX) {
        size = sizeof(twinax_edc) / sizeof(uint16_t);
        for(i = 0; i < size; i+=2) ael2005_write(card, dev, PMA_MDIO_DEVICE_ADDRESS, twinax_edc[i], twinax_edc[i+1]);
    }

    // Step 3
    size = sizeof(regs1) / sizeof(uint16_t);
    for(i = 0; i < size; i+=2) ael2005_write(card, dev, PMA_MDIO_DEVICE_ADDRESS, regs1[i], regs1[i+1]);
    ael2005_sleep(5);

    return 0;
}

int nf10_ael2005_phy_configuration(struct nf10_card *card){
	int port, dev;
    uint16_t value = 0;
    char port_mode;

    // Check if we need initialization
    ael2005_read(card, 2, PMA_MDIO_DEVICE_ADDRESS, AEL_MICRO_CONTROLLER_CTL_ADDRESS, &value);
    //printk(KERN_INFO "nf10: 0xc04a: %04x\n",value);
    if (value & 0x8000) {   // uC held in reset
        printk(KERN_INFO "nf10: Programming the AEL2005 PHY chips...\n");

        for(port = 0; port < 4; port ++) {
            if(port == 0) dev = 2;
            if(port == 1) dev = 1;
            if(port == 2) dev = 0;
            if(port == 3) dev = 3;
            value = 0;

            // Check if we have a 10GBASE-SR cable
            ael2005_i2c_read (card, dev, MODULE_DEV_ADDR, 0x3, &value);
            if((value >> 4) == 1) port_mode = MODE_SR;
            else port_mode = MODE_TWINAX;

            ael2005_initialize(card, dev, port_mode);
        }

        return 0;
    }

    return -1;
}
