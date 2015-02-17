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

#include <stddef.h>
#include <stdint.h>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/intrusive/slist.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/signals2.hpp>
#include <axi_bus_connector.hpp>
#include <poll_thread.hpp>

namespace upb {

namespace b = boost;
namespace bi = boost::intrusive;
namespace bpt = boost::posix_time;
namespace bs2 = boost::signals2;

/**
 * This class is associated with the statistics core on the FPGA.
 * All active statistics counters, which are implemented as 32 bit counters on the FPGA, are increased to 64 bits by doing periodic polling of the values.
 * This class makes use of a shared polling thread which periodically calls the poll() function of this class.
 */
class flow_statistics {


public:
	/**
	 * Constructs a flow_statistics class instance
	 * @param axi_bus Shared pointer to the axi_bus_connector class
	 * @param axi_address The address of the statistics core on the AXI bus (look at the FPGA project's .mhs file for the address)
	 * @param statistics_entries Number of statistics sets which are managed by the core
	 * @param clock_frequency_hz Clock frequency of the OpenFlow core on the FPGA in Hz
	 * @param thread A shared pointer to an instance of the poll_thread class. This class is used for providing a thread.
	 */
	flow_statistics(
		const b::shared_ptr<axi_bus_connector> &axi_bus,
		uint32_t axi_address,
		uint32_t statistics_entries,
		float clock_frequency_hz,
		const b::shared_ptr<poll_thread> &thread
	);

private:

	struct memory_layout {
		union {

			struct {
				uint32_t packet_count;
				uint32_t byte_count;
				uint32_t timestamp;
				uint32_t empty;
			} counters[];

			uint32_t word_access[];
		};
	};

	b::shared_ptr<axi_bus_connector> axi_bus;
	uint32_t axi_address;
	uint32_t statistics_entries;
	float clock_cycle_time_ms;
	b::shared_ptr<poll_thread> thread;

	struct counters_t : public bi::slist_base_hook<> {
		uint32_t packet_count, byte_count, timestamp;

		uint64_t packet_count_64;
		uint64_t byte_count_64;
		uint64_t timestamp_64; // 0xffffffffffffffff if no packet yet arrived
	};

	std::vector<counters_t> counters;

	bs2::scoped_connection poll_thread_conn;

	typedef bi::slist<counters_t> active_flows_list_t;
	active_flows_list_t active_flows_list;

	// timestamp handling
	uint32_t timestamp_axi_address;
	uint64_t timestamp = 0;

	void poll_timestamp();
	void poll();

public:
	/**
	 * Resets the statistics counters (packet, bytes, timestamp) at the given position
	 * @param pos The position of the statistics counter set
	 */
	void reset_counters(uint32_t pos);

private:
	void poll_counters(uint32_t pos);

public:
	/**
	 * Activates the statistics counters at the given position
	 * @param pos The position of the statistics counter set
	 */
	void activate_counters(uint32_t pos);
	/**
	 * Deactivates the statistics counters at the given position
	 * @param pos The position of the statistics counter set
	 */
	void deactivate_counters(uint32_t pos);
	/**
	 * Moves the statistics counters to a new position. The values the the counters are copied.
	 * @param old_pos The old position of the statistics counter set
	 * @param new_pos The new position of the statistics counter set
	 */
	void move_counters_values(uint32_t old_pos, uint32_t new_pos);
	/**
	 * Read out a set of statistics counters
	 * @param pos The position of the statistics counter set
	 * @param packet_count The amount of packets matching the associated flow table entry
	 * @param byte_count The amount of bytes transferred through the associated flow table entry
	 * @param utc_last_packet A UTC time stamp of the last received packet. 'bpt::not_a_date_time' if no packet was yet received
	 * @param reset true: Reset the counters after reading them out
	 */
	void get_flow_statistics(uint32_t pos, uint64_t &packet_count, uint64_t &byte_count, bpt::ptime &utc_last_packet, bool reset);
};


}
