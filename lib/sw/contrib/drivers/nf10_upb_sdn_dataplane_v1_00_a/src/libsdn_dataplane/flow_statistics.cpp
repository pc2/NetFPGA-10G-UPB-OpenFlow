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

#include "flow_statistics.hpp"
#include <assert.h>
#include <math.h>
#include <boost/bind.hpp>

namespace upb {

flow_statistics::flow_statistics(
	const b::shared_ptr<axi_bus_connector> &axi_bus,
	uint32_t axi_address,
	uint32_t statistics_entries,
	float clock_frequency_hz,
	const b::shared_ptr<poll_thread> &thread
) :
	axi_bus(axi_bus),
	axi_address(axi_address),
	statistics_entries(statistics_entries),
	clock_cycle_time_ms(1e3f / clock_frequency_hz),
	thread(thread),
	counters(statistics_entries),
	poll_thread_conn(thread->get_signal().connect(b::bind(&flow_statistics::poll, this)))
{
	uint32_t c_match_addr_width = ceilf(log2f(statistics_entries));
	timestamp_axi_address = axi_address | ((((uint32_t)1 << c_match_addr_width) + 1) << 2); // not << 4 here because we use word access instead of byte access
	poll_timestamp();
}

void flow_statistics::poll_timestamp()
{
	uint32_t read_timestamp = axi_read(word_access[timestamp_axi_address]);
	timestamp += read_timestamp - (uint32_t)timestamp;
}

void flow_statistics::poll()
{
	poll_timestamp();

	for (active_flows_list_t::iterator it = active_flows_list.begin(); it != active_flows_list.end(); it++) {

		uint32_t pos = (uint32_t)(&*it - &*counters.begin());

		poll_counters(pos);
	}
}

void flow_statistics::reset_counters(uint32_t pos)
{
	assert(counters[pos].is_linked()); // this statistics entry has to be activated

	counters[pos].packet_count = axi_read(counters[pos].packet_count);
	counters[pos].byte_count = axi_read(counters[pos].byte_count);

	counters[pos].packet_count_64 = 0;
	counters[pos].byte_count_64 = 0;
	counters[pos].timestamp_64 = (uint64_t)-1; // marker for "no packet yet"
}

void flow_statistics::poll_counters(uint32_t pos)
{
	uint32_t read_packet_count = axi_read(counters[pos].packet_count);

	// only do something if the packet_count changed
	if (counters[pos].packet_count != read_packet_count) {

		counters[pos].packet_count_64 += read_packet_count - counters[pos].packet_count;
		counters[pos].packet_count = read_packet_count;

		uint32_t read_byte_count = axi_read(counters[pos].byte_count);
		counters[pos].byte_count_64 +=  read_byte_count - counters[pos].byte_count;
		counters[pos].byte_count = read_byte_count;

		uint32_t read_timestamp = axi_read(counters[pos].timestamp);
		counters[pos].timestamp_64 = timestamp + ((int32_t)read_timestamp - (int32_t)timestamp);
	}
}

void flow_statistics::activate_counters(uint32_t pos)
{
	assert(!counters[pos].is_linked()); // this statistics entry must not be activated

	active_flows_list.push_front(counters[pos]);
	reset_counters(pos);
}

void flow_statistics::deactivate_counters(uint32_t pos)
{
	assert(counters[pos].is_linked()); // this statistics entry has to be activated

	active_flows_list.erase(active_flows_list.iterator_to(counters[pos]));
}

void flow_statistics::get_flow_statistics(uint32_t pos, uint64_t &packet_count, uint64_t &byte_count, bpt::ptime &utc_last_packet, bool reset)
{
	poll_counters(pos);

	packet_count = counters[pos].packet_count_64;
	byte_count = counters[pos].byte_count_64;

	if (counters[pos].timestamp_64 != (uint64_t)-1) {

		// get the current time since FPGA boot up
		poll_timestamp();

		bpt::time_duration td(bpt::milliseconds((uint64_t)((timestamp - counters[pos].timestamp_64) * clock_cycle_time_ms + 0.5f)));
		utc_last_packet = bpt::microsec_clock::universal_time() - td;

	} else {
		utc_last_packet = bpt::not_a_date_time;
	}

	if (reset) {
		counters[pos].packet_count_64 = 0;
		counters[pos].byte_count_64 = 0;
		counters[pos].timestamp_64 = (uint64_t)-1;
	}
}

void flow_statistics::move_counters_values(uint32_t old_pos, uint32_t new_pos)
{
	assert(counters[new_pos].is_linked()); // the new statistics entry has to be activated

	poll_counters(old_pos);

	counters[new_pos].packet_count_64 = counters[old_pos].packet_count_64;
	counters[new_pos].byte_count_64 = counters[old_pos].byte_count_64;
	counters[new_pos].timestamp_64 = counters[old_pos].timestamp_64;
}

} // namespace upb
