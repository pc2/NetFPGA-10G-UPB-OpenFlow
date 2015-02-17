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

namespace upb {

class config_platform_flash {

public:

	constexpr const static uint32_t flash_chip_count = 2;

	// Flash parameters (taken from Xilinx DS617)

	constexpr const static uint32_t main_blocks = 127;
	constexpr const static uint32_t main_block_size_words = 0x10000;
	constexpr const static uint32_t parameter_blocks = 4;
	constexpr const static uint32_t parameter_block_size_words = 0x4000;
	constexpr const static uint32_t flash_blocks = main_blocks + parameter_blocks;
	constexpr const static uint32_t buffer_size_words = 0x20;
	constexpr const static uint32_t blocks_per_bank = 8;
	constexpr const static uint32_t bank_size = main_block_size_words * blocks_per_bank;
	constexpr const static uint32_t banks = 16;
	constexpr const static uint32_t flash_size_words = bank_size * banks;
	constexpr const static uint32_t bank_mask = ~(bank_size - 1);

	enum {
		FLASH_CMD_BLOCK_ERASE = 0x20,
		FLASH_CMD_CLEAR_STATUS_REG = 0x50,
		FLASH_CMD_LOCK_SETUP = 0x60,
		FLASH_CMD_READ_ELEC_SIG = 0x90,
		FLASH_CMD_WRITE_CONFIRM = 0xd0,
		FLASH_CMD_BUFFER_PROGRAM = 0xe8,
		FLASH_CMD_READ_ARRAY = 0xff
	};

	constexpr const static uint8_t FLASH_STATUS_OK_MASK = 0x7f;

	enum {
		FLASH_STATUS_PEC_READY = 0x80
	};
};

}
