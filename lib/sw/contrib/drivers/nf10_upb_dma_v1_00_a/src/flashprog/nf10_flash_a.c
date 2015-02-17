/*******************************************************************************
 *
 *  NetFPGA-10G http://www.netfpga.org
 *
 *  File:
 *        nf10_flash_a.c
 *
 *  Library:
 *       
 *
 *  Project:
 *        
 *
 *  Author:
 *        Jong Han
 *
 *        Modifications for the UPB OpenFlow Switch project:
 *         Jörg Niklas, osjsn@niklasfamily.de
 *         Project Group "On-the-Fly Networking for Big Data"
 *         Computer Engineering Group, University of Paderborn
 *
 *  Description:
 *        
 *
 *  Copyright notice:
 *        Copyright (C) 2013 University of Cambridge
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
 *        This file was developed by SRI International and the University of
 *        Cambridge Computer Laboratory under DARPA/AFRL contract (FA8750-10-C-0237)
 *        ("CTSRD"), as part of the DARPA CRASH research programme.
 */

#include "reg_lib.h"
#include "emc_flash_lib.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#define NF10_IOCTL_CMD_WRITE_REG (SIOCDEVPRIVATE+1)
#define NF10_IOCTL_CMD_READ_REG (SIOCDEVPRIVATE+2)

int main(int argc, char* argv[])
{
	int f;
	uint64_t v, addr, val;

	addr = 0x40000000;
	val=0x2;

	if (argc != 2) {
		printf("Usage: nf10_flash_a netfpga_device\n");
		return 0;
	}

	// Open device handle
    f = open(argv[1], O_RDWR);
    if(f < 0) {
        perror("Error opening device");
        return 0;
    }

	v = addr;

	if(ioctl(f, NF10_IOCTL_CMD_READ_REG, &v) < 0){
		perror("nf10 ioctl failed");
	return 0;
	}
	
	v &= 0xffffffff;

	if (v != 0x3) {
		printf ("Check HW or Driver, cannot load Flash A!");
	}
	else {
		printf("GPIO initial 0x%lx 0x%lx \n", addr, v);
		v = (addr << 32) + val;
		if(ioctl(f, NF10_IOCTL_CMD_WRITE_REG, &v) < 0){
		        perror("nf10 ioctl failed");
		        return 0;
		}
		printf("FPGA is programmed with Flash A!\n");
	}

	close(f);

	return 0;
}

