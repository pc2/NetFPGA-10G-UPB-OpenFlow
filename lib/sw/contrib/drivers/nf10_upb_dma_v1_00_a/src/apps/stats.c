/*******************************************************************************
 *
 *  NetFPGA-10G http://www.netfpga.org
 *
 *  File:
 *        stats.c
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
 *        Example C application that shows how to read stats off of the NetFPGA
 *        card. All counters are 32 bit wide (64bit counters created some timing
 *        closure issues), even though 8 bytes of address space are reserved 
 *        for each counter. Lower 128 counters are reserved, and the reset are
 *        listed here. (For more details look at DMA SystemVerilog source code
 *        file stats.v)
 *
 *        128+ 1: pcie rx number of 8-byte words
 *        128+ 2: pcie rx number of writes
 *        128+ 3: pcie rx number of reads
 *        128+ 4: pcie rx number of completions
 *        128+ 5: pcie rx number of errors
 *        128+ 8: pcie tx number of 8-byte words
 *        128+ 9: pcie tx number of writes
 *        128+10: pcie tx number of reads
 *        128+11: pcie tx number of completions
 *        128+12: pcie tx number of errors                 
 *        128+17: mac tx number of 8-byte words
 *        128+18: mac tx number of packets
 *        128+21: mac rx number of 8-byte words
 *        128+22: mac rx number of packets
 *        128+23: mac rx number of errors
 *
 *        NOTE: reading the stats is a PCIe event and that itself will affect
 *              the pcie stats
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

	if (argc != 2) {
		printf("Usage: stats netfpga_device\n");
		return 0;
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

    v = 128+2;
    if(ioctl(f, NF10_IOCTL_CMD_READ_STAT, &v) < 0){
        perror("nf10 ioctl failed");
        return 0;
    }
    printf("PCIe RX: number of WRITE pkts:      %" PRIu64 "\n", v);

    v = 128+3;
    if(ioctl(f, NF10_IOCTL_CMD_READ_STAT, &v) < 0){
        perror("nf10 ioctl failed");
        return 0;
    }
    printf("PCIe RX: number of READ pkts:       %" PRIu64 "\n", v);

    v = 128+4;
    if(ioctl(f, NF10_IOCTL_CMD_READ_STAT, &v) < 0){
        perror("nf10 ioctl failed");
        return 0;
    }
    printf("PCIe RX: number of COMPLETION pkts: %" PRIu64 "\n", v);

    v = 128+5;
    if(ioctl(f, NF10_IOCTL_CMD_READ_STAT, &v) < 0){
        perror("nf10 ioctl failed");
        return 0;
    }
    printf("PCIe RX: number of ERRORS:          %" PRIu64 "\n", v);

    printf("\n");

    v = 128+9;
    if(ioctl(f, NF10_IOCTL_CMD_READ_STAT, &v) < 0){
        perror("nf10 ioctl failed");
        return 0;
    }
    printf("PCIe TX: number of WRITE pkts:      %" PRIu64 "\n", v);

    v = 128+10;
    if(ioctl(f, NF10_IOCTL_CMD_READ_STAT, &v) < 0){
        perror("nf10 ioctl failed");
        return 0;
    }
    printf("PCIe TX: number of READ pkts:       %" PRIu64 "\n", v);

    v = 128+11;
    if(ioctl(f, NF10_IOCTL_CMD_READ_STAT, &v) < 0){
        perror("nf10 ioctl failed");
        return 0;
    }
    printf("PCIe TX: number of COMPLETION pkts: %" PRIu64 "\n", v);

    v = 128+12;
    if(ioctl(f, NF10_IOCTL_CMD_READ_STAT, &v) < 0){
        perror("nf10 ioctl failed");
        return 0;
    }
    printf("PCIe TX: number of ERRORS:          %" PRIu64 "\n", v);

    printf("\n");

    v = 128+18;
    if(ioctl(f, NF10_IOCTL_CMD_READ_STAT, &v) < 0){
        perror("nf10 ioctl failed");
        return 0;
    }
    printf("AXIS TX: number of packets: %" PRIu64 "\n", v);

    v = 128+17;
    if(ioctl(f, NF10_IOCTL_CMD_READ_STAT, &v) < 0){
        perror("nf10 ioctl failed");
        return 0;
    }
    printf("AXIS TX: number of words:   %" PRIu64 "\n", v);

    v = 128+22;
    if(ioctl(f, NF10_IOCTL_CMD_READ_STAT, &v) < 0){
        perror("nf10 ioctl failed");
        return 0;
    }
    printf("AXIS RX: number of packets: %" PRIu64 "\n", v);

    v = 128+21;
    if(ioctl(f, NF10_IOCTL_CMD_READ_STAT, &v) < 0){
        perror("nf10 ioctl failed");
        return 0;
    }
    printf("AXIS RX: number of words:   %" PRIu64 "\n", v);

    v = 128+23;
    if(ioctl(f, NF10_IOCTL_CMD_READ_STAT, &v) < 0){
        perror("nf10 ioctl failed");
        return 0;
    }
    printf("AXIS RX: number of errors:  %" PRIu64 "\n", v);
    
    printf("\n");

    close(f);
    
    return 0;
}
