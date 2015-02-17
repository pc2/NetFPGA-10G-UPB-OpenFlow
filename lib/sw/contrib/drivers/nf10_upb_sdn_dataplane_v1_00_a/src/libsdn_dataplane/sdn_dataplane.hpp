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

#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/mem_fun.hpp>
#include <boost/signals2.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <axi_bus_connector.hpp>
#include <flow.hpp>
#include <hw_flow_table_base.hpp>
#include <config_fpga_addresses.hpp>

namespace upb {

namespace b = boost;
namespace bmi = boost::multi_index;
namespace bs2 = boost::signals2;

/**
 * The main class representing the dataplane on the NetFPGA
 */
class sdn_dataplane : public b::enable_shared_from_this<sdn_dataplane>, config_fpga_addresses {

private:
	b::shared_ptr<axi_bus_connector> axi_bus;
	std::string dataplane_name;
	// mutex for the whole sdn_dataplane
	b::shared_ptr<b::recursive_mutex> data_plane_mutex;

	/*
	 * The flow table
	 */

	enum {
		INDEX_FLOWS_BY_PRIORITY
	};

	typedef bmi::multi_index_container <

		flow*,
		bmi::indexed_by<

			// index 0: sort flows by priority (descending)
			bmi::ordered_non_unique<
				bmi::mem_fun<flow, flow::priority_t, &flow::get_priority>,
				std::greater<flow::priority_t> // reverse sorting order
			>
		>

	> flow_table_t;
	flow_table_t flow_table;
	typedef flow_table_t::nth_index<INDEX_FLOWS_BY_PRIORITY>::type index_flows_by_priority;

	/*
	 * The table of hardware flow tables
	 */

	enum {
		INDEX_HW_TABLES_BY_SIZE_DESCENDING,
		INDEX_HW_TABLES_BY_PRIORITY_DESCENDING
	};

	typedef bmi::multi_index_container <

		b::shared_ptr<hw_flow_table_base>,
		bmi::indexed_by<

			// index 0: sort hw tables by table size (descending - largest (= "cheapest") table first)
			bmi::ordered_non_unique<
				bmi::mem_fun<hw_flow_table_base, hw_flow_table_base::table_size_t, &hw_flow_table_base::table_size>,
				std::greater<hw_flow_table_base::table_size_t> // reverse sorting order
			>,

			// index 1: sort hw tables by hardware priority (descending - highest priority table first)
			bmi::ordered_non_unique<
				bmi::mem_fun<hw_flow_table_base, hw_flow_table_base::hw_priority_t, &hw_flow_table_base::hw_priority>,
				std::greater<hw_flow_table_base::hw_priority_t> // reverse sorting order
			>
		>
	> hw_flow_tables_table_t;
	hw_flow_tables_table_t hw_flow_tables_table;
	typedef hw_flow_tables_table_t::nth_index<INDEX_HW_TABLES_BY_SIZE_DESCENDING>::type index_hw_tables_by_size_descending;
	typedef hw_flow_tables_table_t::nth_index<INDEX_HW_TABLES_BY_PRIORITY_DESCENDING>::type index_hw_tables_by_priority_descending;

public:
	/**
	 * Constructs an instance of the class sdn_dataplane
	 * @param card_id The device ID of the NetFPGA card, e.g. '0' for /dev/nf10a, '1' for /dev/nf10b
	 * @param dataplane_name Optional name for the dataplane
	 */
	sdn_dataplane(uint32_t card_id, const char *dataplane_name = "");
	/**
	 * Constructs an instance of the class sdn_dataplane
	 * @param device_name Device name of the NetFPGA card (e.g. /dev/nf10a)
	 * @param dataplane_name Optional name for the dataplane
	 */
	sdn_dataplane(const char *device_name, const char *dataplane_name = "");
	/**
	 * Constructs an instance of the class sdn_dataplane
	 * @param axi_bus A shared pointer to a axi_bus_connector object
	 * @param dataplane_name Optional name for the dataplane
	 */
	sdn_dataplane(const b::shared_ptr<axi_bus_connector> &axi_bus, const char *dataplane_name = "");

public:
	/**
	 * Adds a flow entry to the automatic flow entry management of sdn_dataplane
	 * @param new_flow A shared pointer to the flow object
	 */
	void add_flow(const b::shared_ptr<flow> &new_flow);

private:
	bool higher_priority_entries_are_all_bound(const index_flows_by_priority::iterator &it);
	bool bind_flow(const index_flows_by_priority::iterator &it);
	void try_bind_flows();
	void delete_flow_handler(index_flows_by_priority::iterator &it);

};

}
