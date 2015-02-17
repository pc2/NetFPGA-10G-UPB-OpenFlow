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

#include "sdn_dataplane.hpp"
#include <assert.h>
#include <boost/make_shared.hpp>
#include <boost/log/trivial.hpp>
#include <tcam.hpp>
#include <cam.hpp>
#include <boost/log/trivial.hpp>
#include <poll_thread.hpp>

namespace upb {

sdn_dataplane::sdn_dataplane(uint32_t card_id, const char *dataplane_name) :
	sdn_dataplane(b::make_shared<axi_bus_connector>(card_id), dataplane_name)
{}

sdn_dataplane::sdn_dataplane(const char *device_name, const char *dataplane_name) :
	sdn_dataplane(b::make_shared<axi_bus_connector>(device_name), dataplane_name)
{}

sdn_dataplane::sdn_dataplane(const b::shared_ptr<axi_bus_connector> &axi_bus, const char *dataplane_name) :
	axi_bus(axi_bus),
	dataplane_name(dataplane_name)
{
	data_plane_mutex = b::make_shared<b::recursive_mutex>();

	/*
	 * Instantiation of hardware flow tables
	 */
	b::shared_ptr<hw_flow_table_base> tcam_p = b::make_shared<tcam>(
		axi_bus,
		tcam_address,
		tcam_hw_priority
	);
	bool success = hw_flow_tables_table.get<INDEX_HW_TABLES_BY_SIZE_DESCENDING>().insert(tcam_p).second;
	assert(success);

	b::shared_ptr<hw_flow_table_base> cam_p = b::make_shared<cam>(
		axi_bus,
		cam_address,
		cam_hw_priority
	);
	success = hw_flow_tables_table.get<INDEX_HW_TABLES_BY_SIZE_DESCENDING>().insert(cam_p).second;
	assert(success);

	b::shared_ptr<poll_thread> thread = b::make_shared<poll_thread>(data_plane_mutex, default_statistics_poll_interval_ms);

	b::shared_ptr<flow_statistics> stat = b::make_shared<flow_statistics>(
		axi_bus,
		statistics_module_address,
		action_processor_static_lookup_entries + tcam_p->table_size() + cam_p->table_size() + action_processor_vport_lookup_entries,
		clock_frequency_hz_statistics_counters,
		thread
	);

	tcam_p->set_flow_statistics(stat, action_processor_static_lookup_entries);
	cam_p->set_flow_statistics(stat, action_processor_static_lookup_entries + tcam_p->table_size());

	BOOST_LOG_TRIVIAL(info) << "Created dataplane \"" << this->dataplane_name << "\" instance for NetFPGA " << axi_bus->netfpga_id();
}

void sdn_dataplane::add_flow(const b::shared_ptr<flow> &new_flow)
{
	b::recursive_mutex::scoped_lock lock(*data_plane_mutex); // make this function thread-safe

	index_flows_by_priority &flow_table_index = flow_table.get<INDEX_FLOWS_BY_PRIORITY>();

	std::pair<index_flows_by_priority::iterator, bool> result;
	result = flow_table_index.insert(new_flow.get());

	if (!result.second)
		throw errors::COULD_NOT_INSERT_INTO_FLOW_TABLE; // should NEVER happen!

	new_flow->set_mutex(data_plane_mutex);
	new_flow->add_flow_delete_handler(b::bind(&sdn_dataplane::delete_flow_handler, shared_from_this(), result.first));

	if (!new_flow->is_supported())
		BOOST_LOG_TRIVIAL(info) << axi_bus->netfpga_id() << ": sdn_dataplane: Added unsupported flow: " << new_flow->to_string();

	bool successfully_pushed_flow_to_hardware =
			new_flow->is_supported() // the flow is supported
		&&	higher_priority_entries_are_all_bound(result.first) // all higher priority entries are bound
		&&	bind_flow(result.first) // the flow was added into a hardware table (CAM/TCAM)
	;

	if (!successfully_pushed_flow_to_hardware) {

		// we could not push this flow into hardware

		// deactivate (unbind) all the following flows with lower priorities

		uint32_t deactivated_flow_counter = 0;
		index_flows_by_priority::iterator it;
		for (it = result.first, it++; it != flow_table_index.end(); it++) {

			if ((*it)->is_bound() && (*it)->get_priority() < new_flow->get_priority()) {

				// this flow has lower priority than the new flow
				(*it)->unbind();
				deactivated_flow_counter++;
			}
		}

		if (deactivated_flow_counter)
			BOOST_LOG_TRIVIAL(info) << axi_bus->netfpga_id() << ": sdn_dataplane: Deactivated hardware acceleration for " << deactivated_flow_counter << " flows";
	}
}

bool sdn_dataplane::higher_priority_entries_are_all_bound(const index_flows_by_priority::iterator &it)
{
	index_flows_by_priority &flow_table_index = flow_table.get<INDEX_FLOWS_BY_PRIORITY>();

	/*
	 * Check if a previous entry with a higher priority is unbound
	 * If so: This flow can also not be bound to a hardware table
	 */
	for (index_flows_by_priority::iterator prev = it; prev != flow_table_index.begin();) {

		prev--;

		if (!(*prev)->is_bound() && (*prev)->get_priority() > (*it)->get_priority()) {

			// the previous flow is unbound
			return false; // stop here
		}
	}

	return true;
}

bool sdn_dataplane::bind_flow(const index_flows_by_priority::iterator &it)
{
	index_flows_by_priority &flow_table_index = flow_table.get<INDEX_FLOWS_BY_PRIORITY>();

	/*
	 * Determine which hw tables are suitable for the new flow's priority
	 * If there is a neighbor entry that has exactly our flow priority we are locked to that priority
	 */
	hw_flow_table_base::hw_priority_t highest_allowed_hw_table_priority = hw_flow_table_base::highest_priority;
	hw_flow_table_base::hw_priority_t lowest_allowed_hw_table_priority = hw_flow_table_base::lowest_priority;

	bool already_locked_to_hw_table_priority = false;

	// check if there are (possibly higher priority) flows before this flow that are bound to a hw table
	for (index_flows_by_priority::iterator prev = it; prev != flow_table_index.begin();) {

		prev--;

		if ((*prev)->is_bound()) {

			// there is an bound entry that potentially limits our upper hw table priority
			highest_allowed_hw_table_priority = (*prev)->get_hw_flow_table()->hw_priority();

			if ((*it)->get_priority() == (*prev)->get_priority()) {

				// this flow entry has exactly our priority -> we are locked to the hardware table of that flow
				lowest_allowed_hw_table_priority = highest_allowed_hw_table_priority;
				already_locked_to_hw_table_priority = true;
			}

			break;
		}
	}

	if (!already_locked_to_hw_table_priority) {

		// check if there are (possibly lower priority) flows after this flow that are bound to a hw table

		index_flows_by_priority::iterator next;
		for (next = it, next++; next != flow_table_index.end(); next++) {

			if ((*next)->is_bound()) {

				// there is an bound entry that potentially limits our lower hw table priority
				lowest_allowed_hw_table_priority = (*next)->get_hw_flow_table()->hw_priority();

				if ((*it)->get_priority() == (*next)->get_priority()) {

					// this flow entry has exactly our priority -> we are locked to the hardware table of that flow
					highest_allowed_hw_table_priority = lowest_allowed_hw_table_priority;
				}

				break;
			}
		}
	}

	// iterate through the table of hw tables (ordered by size (costs))
	index_hw_tables_by_size_descending &hw_tables_by_size = hw_flow_tables_table.get<INDEX_HW_TABLES_BY_SIZE_DESCENDING>();
	for (
			index_hw_tables_by_size_descending::iterator hw_tables_it = hw_tables_by_size.begin();
			hw_tables_it != hw_tables_by_size.end();
			hw_tables_it++
	) {

		if (hw_tables_it->get()->hw_priority() >= lowest_allowed_hw_table_priority && hw_tables_it->get()->hw_priority() <= highest_allowed_hw_table_priority) {

			// this table's hw priority is suitable
			if (hw_tables_it->get()->add_flow((*it)->shared_from_this())) {

				// success in adding flow into that hw table
				return true;
			}
		}
	}

	return false;
}

void sdn_dataplane::try_bind_flows()
{
	index_flows_by_priority &flow_table_index = flow_table.get<INDEX_FLOWS_BY_PRIORITY>();

	bool not_successful = false;
	flow::priority_t last_priority;

	for (index_flows_by_priority::iterator it = flow_table_index.begin(); it != flow_table_index.end(); it++) {

		if (not_successful && (*it)->get_priority() != last_priority) {
			/*
			 * There were already unsuccessful attempts to bind flows to hardware
			 * and we are now at a different priority (not allowed) -> abort
			 */
			break;
		}
		if (!(*it)->is_bound()) { // flow is not bound

			if (
					!(*it)->is_supported()	// flow is not supported...
				||	!bind_flow(it)			// ...or could could be successfully bound
			) {
				not_successful = true;
				last_priority = (*it)->get_priority();
			}
		}
	}
}

void sdn_dataplane::delete_flow_handler(index_flows_by_priority::iterator &it)
{
	index_flows_by_priority &flow_table_index = flow_table.get<INDEX_FLOWS_BY_PRIORITY>();

	// delete the current entry
	flow_table_index.erase(it);

	/*
	 * After a flow was deleted: Try to bind other still unbound flows to hw tables
	 */
	try_bind_flows();
}

}
