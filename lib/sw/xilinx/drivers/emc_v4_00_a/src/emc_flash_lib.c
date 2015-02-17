/*******************************************************************************
 *
 *  NetFPGA-10G http://www.netfpga.org
 *
 *  File:
 *        emc_flash_lib.c
 *
 *  Project:
 *        flash_configuration
 *
 *  Author:
 *        Muhammad Shahbaz
 *
 *  Description:
 *        For the Flash Controller Project.
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
#include "emc_flash_lib.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*************************************************************
  Function body
 *************************************************************/

void prog_flash(int dev, char* bin_file, char flash_id)
{
	FILE *file;
	unsigned int addr;
	unsigned int base_addr;
	unsigned int elec_sig = 0;
	int i;

	printf("Programming flash...\r\n");

	// Open bin_file
	file = fopen(bin_file, "rb");
	if (!file)
	{
		fprintf(stderr, "Unable to open file %s.\r\n", bin_file);
		return;
	}

	// Select flash
	if (flash_id == 'a' || flash_id == 'A')
		base_addr = XFL_CONFIG_BASE_ADDR_A;
	else if (flash_id == 'b' || flash_id == 'B')
		base_addr = XFL_CONFIG_BASE_ADDR_B;
	else
	{
		fprintf(stderr, "Invalid target flash '%c'!\r\n", flash_id);
		return;
	}

	clr_status_reg(dev, base_addr);

	printf("Flash image: %s\r\n", bin_file);

	elec_sig = rd_elec_sig(dev, base_addr);
	printf("Manufacturer ID: %x\r\n", elec_sig);

	if (elec_sig != XFL_ELEC_SIG)
	{
		fprintf(stderr, "Invalid electronic signature! Please verify that the CPLD is configured properly.\r\n");
		return;
	}

	clr_status_reg(dev, base_addr);

	printf("Programming flash '%c'.\r\n", flash_id);
	printf("[");

	for (i = 0; i < XFL_CONFIG_TOTAL_BLKS; i++)
	{
		addr = base_addr + (XFL_CONFIG_BLOCK_SIZE * i);
		printf("."); fflush(stdout);
		unlock_single_block(dev, addr);
		erase_single_block(dev, addr);
	}

	clr_status_reg(dev, base_addr);

	flash_wr_binfile_B(dev, base_addr, file);

	printf("]\r\n");
	
        // Close bin_file
        fclose(file);
}

void flash_wr_binfile_B(int dev, unsigned int base_addr, FILE* file)
{
	char *buffer;
	unsigned long fileLen;
	int i;

	// Get file length
	fseek(file, 0, SEEK_END);
	fileLen=ftell(file);
	fseek(file, 0, SEEK_SET);

	// Allocate memory
	buffer=(char *)malloc(fileLen+1);
	if (!buffer)
	{
		fprintf(stderr, "Memory error!\r\n");
		fclose(file);
		return;
	}

	// Read file contents into buffer
	fread(buffer, fileLen, 1, file);

	// Program Flash
	for (i = 0; i < fileLen; i++)
	{
		if (wr_data_B(dev, (base_addr + i), buffer[i]) != 0)
		{
			printf("Writing flash image failed!\r\n");
			return;
		}

		if ((i%XFL_CONFIG_BLOCK_SIZE) == 0)
		{
			printf("."); fflush(stdout);
		}
	}

	// Verify
	printf("(verifying)");
	for (i = 0; i < fileLen; i++)
	{
		char read_byte = rd_cmd(dev, base_addr + i, XFL_CMD_READ_ARRAY);
		if (buffer[i] != read_byte) {
			printf("Verify error at address 0x%08x\r\n", i);
		}
	}

	free(buffer);
}

void unlock_single_block(int dev, unsigned int addr)
{
	char status;

	do
	{
		wr_cmd(dev, addr, XFL_CMD_BLOCK_LOCK_SETUP);
		wr_cmd(dev, addr, XFL_CMD_CONFIRM);
		status = rd_cmd(dev, addr + 2, XFL_CMD_READ_ELEC_SIG);
	}
	while (status == 1);
}

unsigned int erase_single_block(int dev, unsigned int addr)
{
	wr_cmd(dev, addr, XFL_CMD_BLOCK_ERASE_SETUP);
	wr_cmd(dev, addr, XFL_CMD_CONFIRM);

	return flash_wt_rdy(dev, addr);
}

void lock_single_block(int dev, unsigned int addr)
{
	char status;

	do
	{
		wr_cmd(dev, addr, XFL_CMD_BLOCK_LOCK_SETUP);
		wr_cmd(dev, addr, XFL_CMD_BLOCK_LOCK_CONFIRM);
		status = rd_cmd(dev, addr + 2, XFL_CMD_READ_ELEC_SIG);
	}
	while (status == 0);
}

unsigned int wr_data_B(int dev, unsigned int addr, char data)
{
	wr_cmd(dev, addr, XFL_CMD_PROGRAM_SETUP);
	wr_byte(dev, addr, data);

	return flash_wt_rdy(dev, addr);
}

unsigned int rd_elec_sig(int dev, unsigned int addr)
{
	return rd_cmd(dev, addr, XFL_CMD_READ_ELEC_SIG);
}

void clr_status_reg(int dev, unsigned int addr)
{
	char status;
	int count = 0;

	wr_cmd(dev, addr, XFL_CMD_CLEAR_STATUS_REG);

	do
	{
		status = rd_cmd(dev, addr, XFL_CMD_READ_STATUS_REG);
		if (count++ == 1000000) {
			printf("Error: Ready bit not set in status register. Status register: 0x%08x\r\n", (uint8_t)status);
			count = 0;
			wr_cmd(dev, addr, XFL_CMD_CLEAR_STATUS_REG);
		}
	}
	while ((status & (char)XFL_STATUS_READY) != (char)XFL_STATUS_READY);
}

void set_read_array_mode(int dev, unsigned int addr)
{
	wr_cmd(dev, addr, XFL_CMD_READ_ARRAY);
}

unsigned int flash_wt_rdy(int dev, unsigned int addr)
{
	char status;

	// Query the device until its status indicates that it is ready
	do
	{
		status = rd_cmd(dev, addr, XFL_CMD_READ_STATUS_REG);
	}
	while (status != (char)XFL_STATUS_READY);

	return 0;
}

char rd_cmd(int dev, unsigned int addr, char cmd_code)
{
	wr_cmd(dev, addr, cmd_code);
	return rd_byte(dev, addr);
}

void wr_cmd(int dev, unsigned int addr, char cmd_code)
{
	wr_byte(dev, addr, cmd_code);
}

void wr_byte(int dev, unsigned int addr, char data)
{
	reg_wr(dev, XFL_BASE_ADDR + (addr << 2), data & 0x000000FF);
}

char rd_byte(int dev, unsigned int addr)
{
	unsigned int val;
	val = reg_rd(dev, XFL_BASE_ADDR + (addr << 2));

    	return (char)(val & 0x000000FF);
}

