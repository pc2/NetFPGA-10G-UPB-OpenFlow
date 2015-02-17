/*******************************************************************************
 *
 *  NetFPGA-10G http://www.netfpga.org
 *
 *  File:
 *        wraxi.c
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
 *        Example C application that shows how to use the driver to write AXI
 *        registers on the NetFPGA card.
 *        Usage example: ./wraxi 0x7d400000 0xa5a5
 *
 *        NIC project contains 8 AXI registers that can be read/written for
 *        test purposes (they have no effect on the NIC). Check hw/system.mhs
 *        file to find the the AXI addresses (under dma section look for
 *        C_BASEADDR; the addresses are C_BASEADDR...C_BASEADDR+7) 
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

#define __STDC_FORMAT_MACROS
#include <inttypes.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#define NF10_IOCTL_CMD_READ_STAT (SIOCDEVPRIVATE+0)
#define NF10_IOCTL_CMD_WRITE_REG (SIOCDEVPRIVATE+1)
#define NF10_IOCTL_CMD_READ_REG (SIOCDEVPRIVATE+2)

int main(int argc, char* argv[]){
    int f;
    uint64_t v;
    uint64_t addr;
    uint64_t val;

    if(argc < 4){
        printf("usage: rdaxi netfpga_device reg_addr(in hex) reg_val(in_hex)\n\n");
        return 0;
    }
    else{
        sscanf(argv[2], "%" PRIx64, &addr);
        sscanf(argv[3], "%" PRIx64, &val);
    }

    //----------------------------------------------------
    //-- open nf10 file descriptor for all the fun stuff
    //----------------------------------------------------
    f = open(argv[1], O_RDWR);
    if(f < 0){
        perror("Error opening device");
        return 0;
    }
    
    printf("\n");

    // High 32 bits are the AXI address,
    // low 32 bits are the value written to that address
    v = (addr << 32) + val;
    if(ioctl(f, NF10_IOCTL_CMD_WRITE_REG, &v) < 0){
        perror("nf10 ioctl failed");
        return 0;
    }
    printf("\n");

    close(f);
    
    return 0;
}
