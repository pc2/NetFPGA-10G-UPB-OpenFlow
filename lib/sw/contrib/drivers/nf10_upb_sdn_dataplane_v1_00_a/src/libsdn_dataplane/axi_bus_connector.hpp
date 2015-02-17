/*
 * This file is part of the NetFPGA 10G UPB OpenFlow Switch project
 *
 * Copyright (c) 2014, 2015 Jörg Niklas, osjsn@niklasfamily.de
 *
 * Project Group "On-the-Fly Networking for Big Data"
 * SFB 901 "On-The-Fly Computing"
 *
 * University of Paderborn
 * Computer Engineering Group
 * Pohlweg 47 - 49
 * 33098 Paderborn
 * Germany
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <limits.h>
#include <string>

#define axi_read(x) axi_bus->read(axi_address + (uint32_t)(intptr_t)&((memory_layout*)0)->x)
#define axi_write(x,y) axi_bus->write(axi_address + (uint32_t)(intptr_t)&((memory_layout*)0)->x, y)

namespace upb {

/**
 * This class provides the connection to the AXI-4 Lite on-chip bus through the NIC device driver.
 */
class axi_bus_connector {

private:

	int fd; // the file descriptor

	const static uint32_t io_mem_map_size = 4096 * 2; // has to match with kernel driver, otherwise it will generate an error

	enum class axi_fifo_status : uint32_t {

		EMPTY = 1,
		ALMOST_FULL = 2,
		FULL = 4
	};

	struct axi_access_regs_t {

		uint64_t wr_addr_hi_data_lo;
		union {
			uint64_t wr_rd_addr_hi;
			uint64_t rd_rd_data;
		};
		uint32_t axi_status_bits;
	};

	volatile axi_access_regs_t* axi_access_regs;

public:
	/**
	 * The constructor opens a connection to the NetFPGA device and maps the AXI-4-Lite Bus registers to the user space
	 * @param card_id Device id of the NetFPGA card. 0 corresponds to /dev/nf10a, 1 corresponds to /dev/nf10b, ...
	 */
	axi_bus_connector(uint32_t card_id);

	/**
	 * The constructor opens a connection to the NetFPGA device and maps the AXI-4-Lite Bus registers to the user space
	 * @param device Device of the NetFPGA card (e.g. /dev/nf10a)
	 */
	axi_bus_connector(const char *device);

	/**
	 * The destructor closes the connection to the NetFPGA
	 */
	~axi_bus_connector();

private:
	void open(const char *device);

public:

	/**
	 * Read a dword (32 bits) from the AXI-4-Lite bus on the FPGA
	 * @param axi_address The (byte-) address that should be accessed. Only 32 bit aligned accesses are allowed.
	 * @return The read value from the AXI bus. If a wrong address was read (AXI bus timeout) you get a 0xdeadbeef
	 */
	uint32_t read(uint32_t axi_address);

	/**
	 * Write a dword (32 bits) to the AXI-4-Lite bus on the FPGA
	 * @param axi_address The (byte-) address that should be accessed. Only 32 bit aligned accesses are allowed.
	 * @param data The 32 bit data that should be written
	 */
	void write(uint32_t axi_address, uint32_t data);

private:
	std::string id;

public:
	/**
	 * Returns the device name (e.g. "/dev/nf10a")
	 */
	const char *netfpga_id();

private:

	uint32_t axi_write_buffer_level = (uint32_t)-1;

	void poll_write_buffer_empty();

};

}
