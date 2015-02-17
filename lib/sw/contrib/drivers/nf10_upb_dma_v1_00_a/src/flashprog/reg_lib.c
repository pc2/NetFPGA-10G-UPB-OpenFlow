/*******************************************************************************
 *
 *  NetFPGA-10G http://www.netfpga.org
 *
 *  File:
 *        reg_lib.c
 *
 *  Project:
 *        flash_configuration
 *
 *  Author:
 *        Muhammad Shahbaz
 *
 *  Description:
 *        Set of definitions for the NF10 register access library.
 *
 *  Copyright notice:
 *        Copyright (C) 2010, 2011 University of Cambridge
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
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#define IOCTL_CMD_READ_STAT (SIOCDEVPRIVATE+0)
#define IOCTL_CMD_WRITE_REG (SIOCDEVPRIVATE+1)
#define IOCTL_CMD_READ_REG (SIOCDEVPRIVATE+2)

inline uint32_t reg_rd(int dev, uint64_t addr)
{
    if(ioctl(dev, IOCTL_CMD_READ_REG, &addr) < 0){
        perror("ioctl failed");
        return 0;
    }
    return addr & 0xffffffff;
}

inline int reg_wr(int dev, uint64_t addr, uint32_t val)
{
    addr = (addr << 32) + val;
    if(ioctl(dev, IOCTL_CMD_WRITE_REG, &addr) < 0){
        perror("ioctl failed");
        return 0;
    }   
    return -1;
}

/* 
// Code for opening nf10 device
int dev_open()
{
    int dev = open("/dev/nf10", O_RDWR);
    if(dev < 0){
        perror("/dev/nf10");
        return 0;
    }
    return dev;
}
*/
