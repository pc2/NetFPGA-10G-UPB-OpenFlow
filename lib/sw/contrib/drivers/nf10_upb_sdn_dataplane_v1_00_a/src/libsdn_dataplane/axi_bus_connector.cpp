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

#include "axi_bus_connector.hpp"

#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <boost/log/trivial.hpp>
#include "errors.hpp"

namespace upb {

axi_bus_connector::axi_bus_connector(uint32_t card_id)
{
	char device[11] = "/dev/nf10x";
	device[9] = 'a' + card_id;

	open(device);
}

axi_bus_connector::axi_bus_connector(const char *device)
{
	open(device);
}

void axi_bus_connector::open(const char *device)
{
	fd = ::open(device, O_RDWR);
	if (fd < 0)
		throw errors::COULD_NOT_OPEN_NETFPGA_DEVICE;

	axi_access_regs = (volatile axi_access_regs_t*)mmap(NULL, io_mem_map_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	if (axi_access_regs == MAP_FAILED) {
		BOOST_LOG_TRIVIAL(error) << "Could not map to user space. Error text: " << strerror(errno);
		close(fd);
		throw errors::COULD_NOT_MAP_TO_USER_SPACE;
	}

	poll_write_buffer_empty();

	id = device;
}

axi_bus_connector::~axi_bus_connector()
{
	munmap((void*)axi_access_regs, io_mem_map_size);
	close(fd);
}

void axi_bus_connector::poll_write_buffer_empty()
{
	while (axi_access_regs->axi_status_bits != (volatile uint32_t)axi_fifo_status::EMPTY);
}

uint32_t axi_bus_connector::read(uint32_t axi_address)
{
	axi_access_regs->wr_rd_addr_hi = (uint64_t)axi_address << 32;
	__sync_synchronize();
	return (uint32_t)axi_access_regs->rd_rd_data;
}

void axi_bus_connector::write(uint32_t axi_address, uint32_t data)
{
	/*
	 * Basically same code as in nf10driver.c (NIC kernel mode driver)
	 */
	if (axi_write_buffer_level >= 64) {

		do { // wait until there is some write buffer space available

			uint32_t level_bits = axi_access_regs->axi_status_bits;

			if(level_bits & (uint32_t)axi_fifo_status::EMPTY){ // buffer empty
				axi_write_buffer_level = 0;
				break;
			} else if(level_bits & (uint32_t)axi_fifo_status::ALMOST_FULL) { // buffer not almost full => at most 48 entries may be in the fifo
				axi_write_buffer_level = 48;
				break;
			} else if(level_bits & (uint32_t)axi_fifo_status::FULL) { // buffer not full => at most 63 entries may be in the fifo
				axi_write_buffer_level = 63;
				break;
			}

		} while(1);
	}

	axi_access_regs->wr_addr_hi_data_lo = ((uint64_t)axi_address << 32) | data;
	__sync_synchronize();
	axi_write_buffer_level++;
}

const char *axi_bus_connector::netfpga_id()
{
	return id.c_str();
}


}
