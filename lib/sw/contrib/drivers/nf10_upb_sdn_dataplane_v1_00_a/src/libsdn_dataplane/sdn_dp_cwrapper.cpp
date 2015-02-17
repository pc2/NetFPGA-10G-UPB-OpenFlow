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

#include "sdn_dp_cwrapper.hpp"
#include "sdn_dp_cwrapper.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <map>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/log/trivial.hpp>
#include <boost/atomic.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <errors.hpp>
#include <errors.h>
#include <logging.hpp>
#include <port.hpp>
#include <flow.hpp>
#include <sdn_dataplane.hpp>

namespace upb {

namespace b = boost;

template <typename T>
::errors c_exception_handling(bool quiet, const T &f)
{
	try {
		return (::errors)f();
	}
	catch (const upb::errors &e) {
		if (!quiet) // some calls may often produce an error, 'quiet' skips an error message
			BOOST_LOG_TRIVIAL(error) << "Error: " << errors_text::error_text[(uint32_t)e];
		return (::errors)e;
	}
	catch(const std::exception& e) {
		BOOST_LOG_TRIVIAL(error) << "Error: " << errors_text::error_text[(uint32_t)UPB_ERROR_CPP_EXCEPTION];
		BOOST_LOG_TRIVIAL(error) << "Error text of C++ exception: " << e.what();
		return UPB_ERROR_CPP_EXCEPTION;
	}
	catch(...) {
		BOOST_LOG_TRIVIAL(error) << "Error: " << errors_text::error_text[(uint32_t)UPB_ERROR_UNKNOWN_EXCEPTION];
		return UPB_ERROR_UNKNOWN_EXCEPTION;
	}
}

b::mutex dp_map_mutex;
std::map<uint32_t, b::shared_ptr<sdn_dataplane>> dp_map;
b::mutex flow_map_mutex;
typedef std::map<uint64_t, b::shared_ptr<flow>> flow_map_t;
flow_map_t flow_map;
b::atomic<uint64_t> flow_references(1);


} // end namespace upb

extern "C" {

using namespace upb;

void upb_init(void)
{
	logging::init_logging();
	logging::log_level_debug();
}

void upb_log_error(const char *text)
{
	BOOST_LOG_TRIVIAL(error) << text;
}


::errors upb_create_data_plane(uint32_t data_plane_id, const char *data_plane_name)
{
	b::mutex::scoped_lock lock(dp_map_mutex);
	return c_exception_handling(false, [&]()
	{
		// check if this NetFPGA already has an instance of the sdn_dataplane class
		if (dp_map.find(data_plane_id) == dp_map.end()) {

			// No? Create instance.
			dp_map[data_plane_id] = b::make_shared<sdn_dataplane>(data_plane_id, data_plane_name);
		} else {
			return UPB_ERROR_DATAPLANE_ALREADY_EXISTS;
		}

		return UPB_ERROR_OK;
	});
}

::errors upb_add_egress_port_bits(const char *port_name, uint16_t *egress_port_bits, uint32_t *dataplane_id)
{
	return c_exception_handling(true, [&]() // no error printing
	{
		uint32_t card_id, port_id;
		if (port::parse_port_name(port_name, card_id, port_id)) {
			*egress_port_bits |= port::egress_port(port_id);
			*dataplane_id = card_id;
		} else {
			return UPB_ERROR_INVALID_PORT_NAME;
		}
		return UPB_ERROR_OK;
	});
}

::errors upb_get_ingress_port(const char *port_name, uint16_t *ingress_port, uint32_t *dataplane_id)
{
	return c_exception_handling(true, [&]() // no error printing
	{
		uint32_t card_id, port_id;
		if (port::parse_port_name(port_name, card_id, port_id)) {
			*ingress_port = port::ingress_port(port_id);
			*dataplane_id = card_id;
		} else {
			return UPB_ERROR_INVALID_PORT_NAME;
		}
		return UPB_ERROR_OK;
	});
}

::errors upb_get_data_plane_id(const char *port_name, uint32_t *data_plane_id)
{
	uint32_t port_id;
	if (port::parse_port_name(port_name, *data_plane_id, port_id)) {
		return UPB_ERROR_OK;
	} else {
		return UPB_ERROR_INVALID_PORT_NAME;
	}
}

void upb_clear_flow(upb_flow_t *flow)
{
	memset(flow, 0, sizeof(upb_flow_t));
}

void upb_set_action_(const b::shared_ptr<flow> &f, enum upb_action_t action, uint16_t egress_port)
{

	switch (action) {

		case UPB_ACTION_UNSET:
			assert(0); // not allowed at this point
			break;

		case UPB_ACTION_FORWARD:
			f->set_action(flow::action_types::FORWARD, egress_port);
			break;

		case UPB_ACTION_BACK_TO_IN_PORT:
			f->set_action(flow::action_types::BACK_TO_IN_PORT, egress_port);
			break;

		case UPB_ACTION_BROADCAST:
			f->set_action(flow::action_types::BROADCAST, egress_port);
			break;

		case UPB_ACTION_SEND_TO_CONTROLLER:
			f->set_action(flow::action_types::SEND_TO_CONTROLLER, egress_port);
			break;
	}
}

::errors upb_add_flow(
	uint32_t data_plane_id,
	uint64_t *flow_p,
	bool unsupported,
	struct upb_flow_t *key,
	struct upb_flow_t *mask,
	upb_priority_t priority,
	enum upb_action_t action,
	uint16_t egress_port
) {
	return c_exception_handling(false, [&]()
	{
		// create a new flow
		b::shared_ptr<flow> f = b::make_shared<flow>();
		{
			b::mutex::scoped_lock lock(flow_map_mutex);
			flow_map[*flow_p = flow_references.fetch_add(1, b::memory_order_relaxed)] = f;
		}

		// set dirty tag
		if (unsupported) f->set_unsupported();

		// copy the flow's fields
		f->set_ingress_ports(	key->ingress_port,	mask->ingress_port);

		// (ok, c++11 is sometimes ugly)
		flow::mac_t key_src_mac, mask_src_mac, key_dst_mac, mask_dst_mac;
		std::copy(key->src_mac,		key->src_mac+6,		key_src_mac.begin());
		std::copy(mask->src_mac,	mask->src_mac+6,	mask_src_mac.begin());
		std::copy(key->dst_mac,		key->dst_mac+6,		key_dst_mac.begin());
		std::copy(mask->dst_mac,	mask->dst_mac+6,	mask_dst_mac.begin());

		f->set_src_mac(			key_src_mac, 		mask_src_mac);
		f->set_dst_mac(			key_dst_mac, 		mask_dst_mac);
		f->set_ethertype(		key->ethertype, 	mask->ethertype);
		f->set_vlan_id(			key->vlan_id,		mask->vlan_id);
		f->set_vlan_pcp(		key->vlan_pcp,		mask->vlan_pcp);
		f->set_src_ip(			key->src_ip,		mask->src_ip);
		f->set_dst_ip(			key->dst_ip,		mask->dst_ip);
		f->set_ip_prot(			key->ip_prot,		mask->ip_prot);
		f->set_ip_tos(			key->ip_tos,		mask->ip_tos);
		f->set_src_port(		key->src_port,		mask->src_port);
		f->set_dst_port(		key->dst_port,		mask->dst_port);

		// set the priority
		f->set_priority(priority);

		// copy the action (the egress_port is already net NetFPGA's format)
		upb_set_action_(f, action, egress_port);

		// check if this data plane really exists
		{
			b::mutex::scoped_lock lock(dp_map_mutex);
			if (dp_map.find(data_plane_id) != dp_map.end()) {

				// finally add the flow
				dp_map[data_plane_id]->add_flow(f);

			} else {
				return UPB_ERROR_DATA_PLANE_DOES_NOT_EXIST;
			}
		}

		return UPB_ERROR_OK;
	});
}

::errors upb_modify_flow(
	uint64_t flow_p,
	enum upb_action_t action,
	uint16_t egress_port,
	bool reset_counters
) {

	return c_exception_handling(false, [&]()
	{
		b::mutex::scoped_lock lock(flow_map_mutex); // make the map thread-safe
		flow_map_t::iterator it = flow_map.find(flow_p);

		if (it != flow_map.end()) {

			// found the flow

			upb_set_action_(it->second, action, egress_port);
			if (reset_counters)
				it->second->clear_statistics();

		} else {

			// could not find the flow
			return UPB_ERROR_COULD_NOT_FIND_FLOW_BY_REF_ID;
		}

		return UPB_ERROR_OK;
	});

}

::errors upb_delete_flow(uint64_t flow_p)
{
	return c_exception_handling(false, [&]()
	{
		b::mutex::scoped_lock lock(flow_map_mutex); // make the map thread-safe
		flow_map_t::iterator it = flow_map.find(flow_p);

		if (it != flow_map.end()) {

			// found the flow
			assert(it->second.unique()); // just be sure that this pointer is unique (everything else would be a design error)

			flow_map.erase(it); // delete flow from map - this will call the destructor as this is the only shared pointer
		} else {

			// could not find the flow
			return UPB_ERROR_COULD_NOT_FIND_FLOW_BY_REF_ID;
		}

		return UPB_ERROR_OK;
	});
}

::errors upb_get_statistics(uint64_t flow_p, uint64_t *packets, uint64_t *bytes, uint64_t *ms_since_last_packet)
{
	return c_exception_handling(false, [&]()
	{
		b::mutex::scoped_lock lock(flow_map_mutex); // make the map thread-safe
		flow_map_t::iterator it = flow_map.find(flow_p);

		if (it != flow_map.end()) {

			// found the flow

			bpt::ptime utc_last_packet;
			it->second->get_statistics(*packets, *bytes, utc_last_packet);

			if (utc_last_packet.is_not_a_date_time()) {

				*ms_since_last_packet = (uint64_t)-1;

			} else {

				bpt::time_duration td = bpt::microsec_clock::universal_time() - utc_last_packet;
				*ms_since_last_packet = td.total_milliseconds();
			}

		} else {

			// could not find the flow
			return UPB_ERROR_COULD_NOT_FIND_FLOW_BY_REF_ID;
		}

		return UPB_ERROR_OK;
	});
}





















}
