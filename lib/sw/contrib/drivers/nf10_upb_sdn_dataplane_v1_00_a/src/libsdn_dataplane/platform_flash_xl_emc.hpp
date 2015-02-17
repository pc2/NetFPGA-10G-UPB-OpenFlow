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

#include "config_platform_flash.hpp"
#include <stdint.h>
#include <stddef.h>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/shared_array.hpp>
#include <boost/log/trivial.hpp>

namespace upb {

namespace b = boost;

class axi_bus_connector;

/**
 * This class can write bitstreams to the Platform Flash chips
 */
class platform_flash_xl_emc : config_platform_flash {

public:
	/**
	 * Xilinx Platform Flash XL (connected to emc controller) support
	 * @param axi_bus shared pointer to an AXI bus instance
	 * @param axi_address AXI bus address of the EMC
	 */
	platform_flash_xl_emc(b::shared_ptr<axi_bus_connector> axi_bus, uint32_t axi_address);

private:
	b::shared_ptr<axi_bus_connector> axi_bus;
	uint32_t axi_address;

	struct block_t {
		uint32_t size, offset;
	};
	block_t blocks[flash_blocks];

public:
	/**
	 * Currently configured number of flash chips (there are 2 on the NetFPGA)
	 */
	const static uint32_t flash_chip_count = flash_chip_count;

	/**
	 * Write data to the flash chip. If the data is smaller than the size of the chip, the remaining space will be set to all 1's.
	 * @param flash_id The flash id. Currently this is 0 for flash A and 1 for flash B.
	 * @param data shared_array with the data
	 * @param size Size of the data to be programmed
	 */
	void write(uint32_t flash_id, b::shared_array<char> data, uint32_t size);

	/**
	 * Write data to the flash chip. If the data is smaller than the size of the chip, the remaining space will be set to all 1's.
	 * @param flash_id The flash id. Currently this is 0 for flash A and 1 for flash B.
	 * @param filename File name of the file to program
	 */
	void write(uint32_t flash_id, std::string filename);

private:

	struct memory_layout {
		uint32_t nor_flash_array[flash_chip_count][flash_size_words];
	};

	void reset_flash(uint32_t flash_id);
	bool poll_flash_ready(uint32_t flash_id, uint32_t bank_address);
	void unlock_erase_block(uint32_t flash_id, uint32_t address);
	void write_buffer(uint32_t flash_id, uint32_t address, char *data, uint32_t size);
	char read(uint32_t flash_id, uint32_t address);

};

}
