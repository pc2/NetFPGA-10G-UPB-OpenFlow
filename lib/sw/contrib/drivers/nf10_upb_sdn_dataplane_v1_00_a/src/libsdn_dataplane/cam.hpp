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
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/signals2.hpp>
#include <boost/crc.hpp>
#include <hw_flow_table_base.hpp>
#include <axi_bus_connector.hpp>
#include <flow.hpp>
#include <hw_flow.hpp>
#include <flow_statistics.hpp>

#define div_round_up(x,y) (x/y + (x%y?1:0))

namespace upb {

namespace b = boost;
namespace bs2 = boost::signals2;

/**
 * This class controls the Content Addressable Memory (CAM) core on the FPGA
 */
class cam : public hw_flow_table_base, public b::enable_shared_from_this<cam> {

private:
	hw_priority_t hw_priority_;

	b::shared_ptr<axi_bus_connector> axi_bus;
	uint32_t axi_address;

	struct memory_layout {

		union {
			struct {
				uint32_t module_name;
				uint32_t module_version;
				uint32_t cam_entries;
			};
			uint32_t cam[0x2000000][8];
		};
	};

	b::shared_ptr<flow_statistics> stat;
	uint32_t stat_offset;

	struct cam_table_entry {
		flow *flow_ptr = NULL;
		bs2::connection flow_delete_signal_connection;
		uint32_t key_last_word = 0;
	};

	std::vector<cam_table_entry> cam_table;
	uint32_t cam_address_mask;
	hw_flow::hw_tuple cam_compatible_mask;

	b::crc_basic<32> crc_generator;
	uint32_t used_entries = 0;

public:

	/**
	 * The constructor connects to the CAM core on the FPGA, resets the table's contents and does some basic initialization
	 * @param axi_bus Shared pointer to the axi_bus_connector class
	 * @param axi_address The address of the CAM on the AXI bus (look at the FPGA project's .mhs file for the address)
	 * @param hw_priority Hardware priority of the CAM (by default the CAM has priority over the TCAM so the default value is '1' here)
	 */
	cam(const b::shared_ptr<axi_bus_connector> &axi_bus, uint32_t axi_address, hw_priority_t hw_priority);
	/**
	 * The destructor clears the CAM's table on the FPGA
	 */
	~cam();

	void set_flow_statistics(const b::shared_ptr<flow_statistics> &stat, uint32_t stat_offset);
	table_size_t table_size();
	hw_priority_t hw_priority();
	bool add_flow(const b::shared_ptr<flow> &new_flow);

private:

	void delete_flow(uint32_t flow_pos);
	void update_action(uint32_t action_hardware_representation, uint32_t flow_pos_cam_table);
	void activate_flow_hw(uint32_t flow_pos_cam_table);
	void deactivate_flow_hw(uint32_t flow_pos_cam_table);
	void reset_table();

};

}

#undef div_round_up

