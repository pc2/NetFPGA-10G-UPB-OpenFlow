/*******************************************************************************
 *
 *  NetFPGA-10G http://www.netfpga.org
 *
 *  File:
 *        reg_lib.h
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

#ifndef _REG_LIB_H_
#define _REG_LIB_H_

#include <stdio.h>
#include <stdint.h>

inline uint32_t reg_rd(int dev, uint64_t addr);
inline int reg_wr(int dev, uint64_t addr, uint32_t val);

#endif
