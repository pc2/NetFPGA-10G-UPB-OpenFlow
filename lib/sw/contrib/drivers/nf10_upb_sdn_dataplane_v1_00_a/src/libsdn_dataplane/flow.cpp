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

#include "flow.hpp"
#include <assert.h>
#include <string.h>
#include <iomanip>
#include <arpa/inet.h>
#include <boost/thread/mutex.hpp>
#include <boost/format.hpp>
#include <boost/log/trivial.hpp>

namespace upb {

namespace b = boost;

flow::flow()
{
	// initialize all bits of key and mask with 0's (including the unused!) that we can efficiently use it for calculations
	memset(&key, 0, sizeof(key));
	memset(&mask, 0, sizeof(mask));
}

flow::~flow()
{
	b::recursive_mutex::scoped_lock lock(*data_plane_mutex); // make this function thread-safe - set_mutex has to be used before

	if (is_bound()) {
		// remove the flow from the hardware
		unbind();
	}
	delete_handlers(); // send the terminate info
	delete_handlers.disconnect_all_slots();
}

void flow::delete_flow()
{
	b::recursive_mutex::scoped_lock lock(*data_plane_mutex); // make this function thread-safe - set_mutex has to be used before

	if (!is_flow_added())
		throw errors::FLOW_NOT_INSTALLED;

	if (is_bound()) {
		// remove the flow from the hardware
		unbind();
	}
	delete_handlers();
	delete_handlers.disconnect_all_slots();
}

void flow::update_statistics()
{
	assert(is_bound());

	uint64_t packet_count;
	uint64_t byte_count;
	bpt::ptime utc_last_packet;

	get_statistics_function(packet_count, byte_count, utc_last_packet);

	if (packet_count) {
		this->packet_count += packet_count;
		this->byte_count += byte_count;
		this->utc_last_packet = utc_last_packet;
	}
}

void flow::get_statistics(uint64_t &packet_count, uint64_t &byte_count, bpt::ptime &utc_last_packet)
{
	b::recursive_mutex::scoped_lock lock(*data_plane_mutex); // make this function thread-safe - set_mutex has to be used before

	if (is_bound()) {
		update_statistics();
	}

	packet_count = this->packet_count;
	byte_count = this->byte_count;
	utc_last_packet = this->utc_last_packet;
}

void flow::clear_statistics()
{
	if (is_bound()) {
		update_statistics();
	}

	this->packet_count = 0;
	this->byte_count = 0;
	this->utc_last_packet = bpt::not_a_date_time;
}

void flow::set_action(action_types type, uint16_t port_vport)
{
	action.type = type;
	action.egress_port.portvport = port_vport;

	if (is_bound()) {
		update_action_function(get_action_hw_representation());
	}
}

void flow::eliminate_unneccessary_wildcards()
{
	bool keep_src_ip = true;
	bool keep_dst_ip = true;
	bool keep_ip_prot = true;
	bool keep_ip_tos = true;
	bool keep_src_port = true;
	bool keep_dst_port = true;

	if (mask.ethertype == 0xffff) {

		switch (htons(key.ethertype)) {

			case 0x0800: // IPv4

				if (mask.ip_prot == 0xff) {

					switch (key.ip_prot) {

						case 1:	// ICMP
							break;

						case 6: // TCP
							break;

						case 17: // UDP
							break;

						default:
							// all other protocols don't need src_port and dst_port
							keep_src_port = false;
							keep_dst_port = false;
							break;
					}
				}
				break;


			case 0x0806: // ARP
				keep_ip_tos = false;
				keep_src_port = false;
				keep_dst_port = false;

				break;

			default:
				// all other Ethernet types don't need any of these fields
				keep_src_ip = false;
				keep_dst_ip = false;
				keep_ip_prot = false;
				keep_ip_tos = false;
				keep_src_port = false;
				keep_dst_port = false;
				break;
		}
	}

	if (!keep_src_ip) {
		key.src_ip = 0;
		mask.src_ip = (uint32_t)-1;
	}

	if (!keep_dst_ip) {
		key.dst_ip = 0;
		mask.dst_ip = (uint32_t)-1;
	}

	if (!keep_ip_prot) {
		key.ip_prot = 0;
		mask.ip_prot = (uint8_t)-1;
	}

	if (!keep_ip_tos) {
		key.ip_tos = 0;
		mask.ip_tos = (uint8_t)-1;
	}

	if (!keep_src_port) {
		key.src_port = 0;
		mask.src_port = (uint16_t)-1;
	}

	if (!keep_dst_port) {
		key.dst_port = 0;
		mask.dst_port = (uint16_t)-1;
	}
}

std::string flow::mac_to_string(mac_t key, mac_t mask)
{
	if (	mask[0] == 0xff
		&&	mask[1] == 0xff
		&&	mask[2] == 0xff
		&&	mask[3] == 0xff
		&&	mask[4] == 0xff
		&&	mask[5] == 0xff
	) {
		// direct match
		b::format fmter("%1%:%2%:%3%:%4%:%5%:%6%");
		for (int i = 0; i < 6; i++)
			fmter % boost::io::group(std::setfill('0'), std::hex, std::setw(2), (uint32_t)key[i]);
		return fmter.str();
	}

	if (		mask[0] == 0x00
			&&	mask[1] == 0x00
			&&	mask[2] == 0x00
			&&	mask[3] == 0x00
			&&	mask[4] == 0x00
			&&	mask[5] == 0x00
		) {
		// complete wildcard
		return std::string("*");
	}

	return std::string("(partial *)");
}

std::string flow::ethertype_to_string()
{
	if (mask.ethertype == 0xffff) {
		switch (htons(key.ethertype)) {

			case 0x0800:
				return std::string("IPv4");
			case 0x0806:
				return std::string("ARP");
			case 0x86DD:
				return std::string("IPv6");
			case 0x8847:
				return std::string("MPLS UC");
			case 0x8848:
				return std::string("MPLS MC");
			case 0x9100:
				return std::string("Q-in-Q");
			default: {
				b::format fmter("0x%1%");
				fmter % boost::io::group(std::setfill('0'), std::hex, std::setw(4), (uint32_t)htons(key.ethertype));
				return fmter.str();
			}
		}
	}
	if (mask.ethertype == 0) {
		return std::string("*");
	}
	return std::string("(partial *)");
}

std::string flow::ipproto_to_string()
{
	if (
			mask.ethertype != 0xffff		// direct match for ethertype
		||	htons(key.ethertype) != 0x0800	// IPv4
	) return std::string();

	if (mask.ip_prot == 0xff) {
		switch (key.ip_prot) {

			case 1:
				return std::string("ICMP");
			case 6:
				return std::string("TCP");
			case 17:
				return std::string("UDP");
			default: {
				b::format fmter("%d");
				fmter % (uint32_t)key.ip_prot;
				return fmter.str();
			}
		}
	}
	if (mask.ip_prot == 0) {
		return std::string("*");
	}
	return std::string("(partial *)");
}

std::string flow::ip_to_string(uint32_t key, uint32_t mask)
{
	if (
			this->mask.ethertype != 0xffff				// direct match for ethertype
		||	(		htons(this->key.ethertype) != 0x0800	// IPv4
				&&	htons(this->key.ethertype) != 0x0806	// ARP
			)
	) return std::string();

	if (mask == 0xffffffff) {

		// direct match

		b::format fmter("%1%.%2%.%3%.%4%");
		fmter
			% (uint32_t)((uint8_t*)&key)[0]
			% (uint32_t)((uint8_t*)&key)[1]
			% (uint32_t)((uint8_t*)&key)[2]
			% (uint32_t)((uint8_t*)&key)[3]
		;
		return fmter.str();
	}

	if (mask == 0) {
		// complete wildcard
		return std::string("*");
	}

	return std::string("(partial *)");
}

std::string flow::port_to_string(uint16_t key, uint16_t mask)
{
	if (
			this->mask.ethertype != 0xffff		// direct match for ethertype
		||	htons(this->key.ethertype) != 0x0800// IPv4
		||	this->mask.ip_prot != 0xff			// direct match for IP protocol
		||	(	this->key.ip_prot != 1			// ICMP
				&&	this->key.ip_prot != 6		// TCP
				&&	this->key.ip_prot != 17		// UDP
			)
	) return std::string();

	if (mask == 0xffff) {

		// direct match
		b::format fmter("%d");
		fmter % (uint32_t)key;
		return fmter.str();
	}

	if (mask == 0) {
		// complete wildcard
		return std::string("*");
	}

	return std::string("(partial *)");
}

std::string flow::action_to_string()
{
	switch (action.type) {

		case action_types::FORWARD:
			if (action.egress_port.portvport == 0) {
				return std::string("DROP");
			} else {
				return std::string("FORWARD");
			}

		case action_types::BACK_TO_IN_PORT:
			return std::string("IN PORT");

		case action_types::BROADCAST:
			return std::string("BROADCAST");

		case action_types::SEND_TO_CONTROLLER:
			return std::string("CONTROLLER");
	}

	return std::string("UNKNOWN");
}

std::string flow::priority_to_string()
{
	b::format fmter("%d");
	fmter % (uint32_t)priority;
	return fmter.str();
}

std::string flow::action_egress_port_to_string()
{
	return
			action_to_string()
		+	" "
		+	port::egress_ports_to_string(action.egress_port)
	;
}

std::string flow::to_string()
{
	return
			action_to_string()
			+ " ("
			+ priority_to_string()
			+ ") ("
			+ port::ingress_ports_to_string(key.ingress_port, mask.ingress_port)
			+ "->"
			+ port::egress_ports_to_string(action.egress_port)
			+ ") "
			+ src_mac_to_string()
			+ "->"
			+ mac_to_string(key.dst_mac, mask.dst_mac)
			+ " "
			+ ethertype_to_string()
			+ " "
			+ src_ip_to_string()
			+ " "
			+ dst_ip_to_string()
			+ " "
			+ ipproto_to_string()
			+ " "
			+ src_port_to_string()
			+ " "
			+ dst_port_to_string()
	;
}

} // namespace upb

