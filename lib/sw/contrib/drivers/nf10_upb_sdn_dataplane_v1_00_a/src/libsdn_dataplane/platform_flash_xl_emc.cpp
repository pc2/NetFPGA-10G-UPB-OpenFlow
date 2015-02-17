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

#include "platform_flash_xl_emc.hpp"
#include <algorithm>
#include <assert.h>
#include <iostream>
#include <fstream>
#include <boost/date_time.hpp>
#include "axi_bus_connector.hpp"
#include "errors.hpp"

namespace upb {

namespace bpt = boost::posix_time;

platform_flash_xl_emc::platform_flash_xl_emc(b::shared_ptr<axi_bus_connector> axi_bus, uint32_t axi_address) :
	axi_bus(axi_bus),
	axi_address(axi_address)
{
	uint32_t i = 0, offset = 0;
	for (; i < main_blocks; i++, offset+=main_block_size_words) {
		blocks[i].size = main_block_size_words;
		blocks[i].offset = offset;
	}
	for (; i < flash_blocks; i++, offset+=parameter_block_size_words) {
		blocks[i].size = parameter_block_size_words;
		blocks[i].offset = offset;
	}
}

void platform_flash_xl_emc::write(uint32_t flash_id, std::string filename)
{
	std::ifstream file(filename, std::ios::in|std::ios::binary|std::ios::ate);
	if (file.is_open()) {

		size_t size = file.tellg();
		b::shared_array<char> data(new char[size]);
		file.seekg (0, std::ios::beg);
		file.read(data.get(), size);
		file.close();
		write(flash_id, data, size);
	} else {

		throw errors::ERROR_COULD_NOT_OPEN_FILE_FOR_FLASH_PROG;
	}
}

void platform_flash_xl_emc::write(uint32_t flash_id, b::shared_array<char> data, uint32_t size)
{
	assert(flash_id < flash_chip_count);

	BOOST_LOG_TRIVIAL(info) << "Programming flash " << (char)('A' + flash_id);
	reset_flash(flash_id);

	uint32_t write_pos = 0;
	for (uint32_t block = 0; block < flash_blocks; block++) { // over all flash blocks

		unlock_erase_block(flash_id, blocks[block].offset); // erase one block

		for (uint32_t block_pos = 0; block_pos < blocks[block].size / buffer_size_words; block_pos++) {

			uint32_t bufsize = buffer_size_words;
			uint32_t words_to_write_now = std::min(bufsize, size - write_pos);

			if (words_to_write_now) { // there is still something to write

				// write up to 32 bytes
				write_buffer(flash_id, write_pos, &data[write_pos], words_to_write_now);

				// verify
				for (uint32_t buf_pos = 0; buf_pos < words_to_write_now; buf_pos++) {
					if (read(flash_id, write_pos + buf_pos) != data[write_pos + buf_pos])
						throw errors::ERROR_VERIFYING_FLASH_CONTENTS;
				}

				write_pos += words_to_write_now;
			}
		}

		BOOST_LOG_TRIVIAL(info) << "Written flash block " << block+1 << "/" << flash_blocks << " successfully";
	}
}

void platform_flash_xl_emc::reset_flash(uint32_t flash_id)
{
	// put all banks to "read array" mode
	for (int i = 0; i < banks; i++)
		for (int j = 0; j < 33; j++) // this ensures that every "buffer program" command is finished (aborted)
			axi_write(nor_flash_array[flash_id][i * bank_size], FLASH_CMD_READ_ARRAY);

	// clear status register
	axi_write(nor_flash_array[0], FLASH_CMD_CLEAR_STATUS_REG);
}

bool platform_flash_xl_emc::poll_flash_ready(uint32_t flash_id, uint32_t bank_address)
{
	// assume we are in "read status register" mode
	uint8_t status;

	bpt::ptime start = bpt::microsec_clock::local_time();
	do {
		status = (uint8_t)axi_read(nor_flash_array[flash_id][bank_address]);

		bpt::time_duration dur = bpt::microsec_clock::local_time() - start;
		if (dur.total_milliseconds() > 10000) {

			axi_write(nor_flash_array[flash_id][bank_address], FLASH_CMD_READ_ARRAY); // go back to "read array" mode
			throw errors::FLASH_TIMEOUT;
		}

	} while (!(status & FLASH_STATUS_PEC_READY));

	return (status & FLASH_STATUS_OK_MASK) == 0;
}

void platform_flash_xl_emc::unlock_erase_block(uint32_t flash_id, uint32_t address)
{
	// first unlock a potentially locked block
	axi_write(nor_flash_array[flash_id][address], FLASH_CMD_LOCK_SETUP);
	axi_write(nor_flash_array[flash_id][address], FLASH_CMD_WRITE_CONFIRM);

	// erase the block
	axi_write(nor_flash_array[flash_id][address], FLASH_CMD_BLOCK_ERASE);
	axi_write(nor_flash_array[flash_id][address], FLASH_CMD_WRITE_CONFIRM);

	if (!poll_flash_ready(flash_id, address))
		throw errors::ERROR_ERASING_FLASH_BLOCK;

	// we stay in "status register" mode here!
}

void platform_flash_xl_emc::write_buffer(uint32_t flash_id, uint32_t address, char *data, uint32_t size)
{
	assert(size != 0);
	assert(size <= 32);

	axi_write(nor_flash_array[flash_id][address], FLASH_CMD_BUFFER_PROGRAM);
	poll_flash_ready(flash_id, address);
	axi_write(nor_flash_array[flash_id][address], size-1); // set the burst size (max 32)

	for (int i = 0; i < size; i++) {
		axi_write(nor_flash_array[flash_id][address+i], data[i]);
	}

	axi_write(nor_flash_array[flash_id][address], FLASH_CMD_WRITE_CONFIRM);
	bool success = poll_flash_ready(flash_id, address);
	axi_write(nor_flash_array[flash_id][address], FLASH_CMD_READ_ARRAY); // always go back to "read array" mode
	if (!success)
		throw errors::ERROR_PROGRAMMING_FLASH_BUFFER;
}

char platform_flash_xl_emc::read(uint32_t flash_id, uint32_t address)
{
	// we assume that we are always in "read array" mode
	return (char)axi_read(nor_flash_array[flash_id][address]);
}

}
