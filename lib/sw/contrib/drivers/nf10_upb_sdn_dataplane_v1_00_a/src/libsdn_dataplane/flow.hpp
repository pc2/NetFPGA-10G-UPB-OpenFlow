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
#include <array>
#include <string>
#include <boost/signals2.hpp>
#include <boost/function.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <errors.hpp>
#include <port.hpp>
#include <hw_flow_table_base.hpp>

namespace upb {

namespace b = boost;
namespace bs2 = boost::signals2;
namespace bpt = boost::posix_time;

class hw_flow;

/**
 * The main class which represents an OpenFlow flow table entry in the software.
 * Shared objects of this class can be added to the hardware table classes (CAM/TCAM).
 */
class flow : public b::enable_shared_from_this<flow> {

	friend class hw_flow;

public:
	/**
	 * Constructs a new flow where all key and mask fields are zero (so every packet matches) and a DROP action
	 */
	flow();
	/**
	 * Destructs the flow table entry. If this entry was bound to a hardware table, it gets uninstalled there.
	 */
	~flow();

private:
	typedef b::shared_ptr<boost::recursive_mutex> mutex_t;
	mutex_t data_plane_mutex;

public:
	/**
	 * Adds a mutex object the the flow object that the access functions can be made thread-safe
	 * @param data_plane_mutex Shared pointer to a boost::recursive_mutex object
	 */
	void set_mutex(const mutex_t &data_plane_mutex)
	{
		assert(!this->data_plane_mutex);
		this->data_plane_mutex = data_plane_mutex;
	}

private:
	enum class flow_states {
		UNBOUND,
		BOUND,
		UNSUPPORTED,
	};

	flow_states flow_state = flow_states::UNBOUND;

public:

	/**
	 * Marks this flow entry as unsupported (for example if it has unsupported header fields of later OpenFlow versions).
	 */
	void set_unsupported() {
		assert(flow_state != flow_states::BOUND);
		flow_state = flow_states::UNSUPPORTED;
	}

	/**
	 * Query if this is a supported flow entry
	 * @return true if the flow is supported
	 */
	bool is_supported() {
		return flow_state != flow_states::UNSUPPORTED;
	}

private:

	b::function<void ()> unbind_function;
	b::function<void (uint32_t)> update_action_function;
	b::function<void (uint64_t&, uint64_t&, bpt::ptime&)> get_statistics_function;
	b::shared_ptr<hw_flow_table_base> associated_hw_table;

public:

	/**
	 * Query if the flow entry is bound to a hardware table
	 * @return true if the flow is bound
	 */
	bool is_bound()
	{
		return flow_state == flow_states::BOUND;
	}

	/**
	 * Bind the flow to a hardware table. This is used by the classes cam and tcam.
	 * @param hw_table Shared pointer to a hardware table object
	 * @param unbind_function Function to be called if the flow entry should be unbound from hardware
	 * @param update_action_function Function to be called if the associated action should be changed
	 * @param get_statistics_function Function to be called if statistics are requested
	 */
	template <class T, class U, class V>
	void bind(const b::shared_ptr<hw_flow_table_base> &hw_table, const T &unbind_function, const U &update_action_function, const V &get_statistics_function)
	{
		assert(is_supported());
		assert(!is_bound());
		associated_hw_table = hw_table;
		this->unbind_function = unbind_function;
		this->update_action_function = update_action_function;
		this->get_statistics_function = get_statistics_function;
		flow_state = flow_states::BOUND;
	}

	/**
	 * Update the functions already set with bind(...). This is used by the classes cam and tcam.
	 * @param unbind_function Function to be called if the flow entry should be unbound from hardware
	 * @param update_action_function Function to be called if the associated action should be changed
	 * @param get_statistics_function Function to be called if statistics are requested
	 */
	template <class T, class U, class V>
	void update_binding(const T &unbind_function, const U &update_action_function, const V &get_statistics_function)
	{
		assert(is_supported());
		assert(is_bound());
		this->unbind_function = unbind_function;
		this->update_action_function = update_action_function;
		this->get_statistics_function = get_statistics_function;
	}

	/**
	 * Returns the associated hardware table object (base class)
	 * @return A shared pointer to a hw_flow_table_base object
	 */
	const b::shared_ptr<hw_flow_table_base>& get_hw_flow_table()
	{
		assert(is_bound());
		return associated_hw_table;
	}

	/**
	 * Unbinds the flow entry from the hardware table
	 */
	void unbind()
	{
		assert(is_bound());

		unbind_function();
		unbind_function.clear();
		update_action_function.clear();
		update_statistics(); // fetch the statistics for the last time
		get_statistics_function.clear();
		associated_hw_table.reset();
		flow_state = flow_states::UNBOUND;
	}

	/**
	 * Throws an exception (errors::FLOW_ALREADY_INSTALLED) if the flow entry is bound to a hardware table
	 */
	void ensure_unbound()
	{
		if (is_bound()) {
			throw errors::FLOW_ALREADY_INSTALLED;
		}
	}

private:

	bs2::signal<void ()> delete_handlers;

public:
	/**
	 * Adds a delete handler which gets called when the flow is deleted
	 * @param delete_handler Handler to the delete function object
	 */
	template <class T>
	void add_flow_delete_handler(const T &delete_handler)
	{
		delete_handlers.connect(delete_handler);
	}

	/**
	 * Test if there are flow delete handlers registered with the flow entry
	 * @return true if handlers were installed
	 */
	bool is_flow_added()
	{
		return !delete_handlers.empty();
	}

	/**
	 * Unbinds the flow from hardware and calls the delete handlers
	 */
	void delete_flow();

private:
	uint64_t packet_count = 0;
	uint64_t byte_count = 0;
	bpt::ptime utc_last_packet = bpt::not_a_date_time;

	void update_statistics();

public:
	/**
	 * Request statistics
	 * @param packet_count Number of received packets
	 * @param byte_count Number of received bytes
	 * @param utc_last_packet Point in time when the last packet was received
	 */
	void get_statistics(uint64_t &packet_count, uint64_t &byte_count, bpt::ptime &utc_last_packet);
	void clear_statistics();


public:
	/**
	 * Type of MAC addresses
	 */
	typedef std::array<uint8_t,6> mac_t;

private:
	struct tuple {
		port::port_t ingress_port;
		mac_t src_mac, dst_mac;
		uint16_t ethertype;
		uint16_t vlan_id;
		uint8_t vlan_pcp;

		uint32_t src_ip, dst_ip;
		uint8_t ip_prot;
		uint8_t ip_tos;
		uint16_t src_port, dst_port;
	};

	tuple key;
	tuple mask; // each set bit in the mask specifies an exact match with the corresponding bit in the key ('0' = wildcard)

public:
	/**
	 * Type of the flow entry priorities
	 */
	typedef uint32_t priority_t;

private:
	priority_t priority = 0xffff;

public:
	/**
	 * All supported action types
	 */
	enum class action_types : uint8_t {
		FORWARD,			//!< Forward to a specified port/vport
		BACK_TO_IN_PORT,	//!< Send the packet back to the ingress port
		BROADCAST,			//!< Broadcast the packet
		SEND_TO_CONTROLLER	//!< Send the packet to the software switch
	};

private:

	// in this case the software and the hardware can share the same representation
	struct fw_action {

		union {
			struct {
				port::port_t egress_port;
				action_types type;
			};
			uint32_t hw_representation;
		};
	};

	fw_action action = {}; // default: DROP (all initialized to 0)

public:
	/**
	 * Returns the hardware representation of the associated action
	 * @return Hardware representation of the action
	 */
	uint32_t get_action_hw_representation()
	{
		return action.hw_representation;
	}

private:

	b::function<void (action_types type, uint16_t port_vport)> update_action_f;

public:
	/**
	 * Set the action which should be processed on a match
	 * @param type One action of action_types
	 * @param port_vport Egress port and vport in case of a forwarding action
	 */
	void set_action(action_types type, uint16_t port_vport);

	/**
	 * Set the priority of the flow entry
	 * @param priority The priority
	 */
	void set_priority(priority_t priority)
	{
		ensure_unbound();
		this->priority = priority;
	}

	/**
	 * Get the priority of the flow entry
	 * @return The priority
	 */
	priority_t get_priority()
	{
		return priority;
	}

	/**
	 * Set a key/mask combination for the ingress port
	 * @param key The ingress port
	 * @param mask The associated mask. Each '1' bit in the mask denotes that the same bit in the key has to match. A '0' bit denotes a wildcard bit (don't care bit).
	 */
	void set_ingress_ports(uint16_t key, uint16_t mask)
	{
		ensure_unbound();
		this->key.ingress_port.portvport = key;
		this->mask.ingress_port.portvport = mask;
	}

	/**
	 * Set a direct match for the source MAC address
	 * @param src_mac The source MAC address
	 */
	void set_src_mac(mac_t &src_mac)
	{
		ensure_unbound();
		key.src_mac = src_mac;
		this->mask.src_mac.fill(0xff);
	}

	/**
	 * Set a key/mask combination for the source MAC address
	 * @param src_mac The source MAC address
	 * @param mask The associated mask. Each '1' bit in the mask denotes that the same bit in the key has to match. A '0' bit denotes a wildcard bit (don't care bit).
	 */
	void set_src_mac(mac_t &src_mac, mac_t &mask)
	{
		ensure_unbound();
		key.src_mac = src_mac;
		this->mask.src_mac = mask;
	}

	/**
	 * Set a direct match for the destination MAC address
	 * @param dst_mac The source MAC address
	 */
	void set_dst_mac(mac_t &dst_mac)
	{
		ensure_unbound();
		key.src_mac = dst_mac;
		this->mask.src_mac.fill(0xff);
	}

	/**
	 * Set a key/mask combination for the destination MAC address
	 * @param dst_mac The destination MAC address
	 * @param mask The associated mask. Each '1' bit in the mask denotes that the same bit in the key has to match. A '0' bit denotes a wildcard bit (don't care bit).
	 */
	void set_dst_mac(mac_t &dst_mac, mac_t &mask)
	{
		ensure_unbound();
		key.dst_mac = dst_mac;
		this->mask.dst_mac = mask;
	}

	/**
	 * Set a key/mask combination for the ethertype
	 * @param ethertype The ethertype
	 * @param mask The associated mask. Each '1' bit in the mask denotes that the same bit in the key has to match. A '0' bit denotes a wildcard bit (don't care bit).
	 */
	void set_ethertype(uint16_t ethertype, uint16_t mask = (uint16_t)-1)
	{
		ensure_unbound();
		key.ethertype = ethertype;
		this->mask.ethertype = mask;
	}

	/**
	 * Set a key/mask combination for the vlan id
	 * @param vlan_id The vlan id
	 * @param mask The associated mask. Each '1' bit in the mask denotes that the same bit in the key has to match. A '0' bit denotes a wildcard bit (don't care bit).
	 */
	void set_vlan_id(uint16_t vlan_id, uint16_t mask = 0x0fff)
	{
		ensure_unbound();
		if ((vlan_id & 0xf000) || (mask & 0xf000))
			throw errors::UNUSED_BITS_SET_IN_HEADER_FIELD;
		key.vlan_id = vlan_id;
		this->mask.vlan_id = mask;
	}

	/**
	 * Set a key/mask combination for the vlan pcp
	 * @param vlan_pcp The vlan pcp
	 * @param mask The associated mask. Each '1' bit in the mask denotes that the same bit in the key has to match. A '0' bit denotes a wildcard bit (don't care bit).
	 */
	void set_vlan_pcp(uint8_t vlan_pcp, uint8_t mask = 0x07)
	{
		ensure_unbound();
		if ((vlan_pcp & 0xf8) || (vlan_pcp & 0xf8))
			throw errors::UNUSED_BITS_SET_IN_HEADER_FIELD;
		key.vlan_pcp = vlan_pcp;
		this->mask.vlan_pcp = mask;
	}

	/**
	 * Set a key/mask combination for the source IP
	 * @param src_ip The source IP
	 * @param mask The associated mask. Each '1' bit in the mask denotes that the same bit in the key has to match. A '0' bit denotes a wildcard bit (don't care bit).
	 */
	void set_src_ip(uint32_t src_ip, uint32_t mask = (uint32_t)-1)
	{
		ensure_unbound();
		key.src_ip = src_ip;
		this->mask.src_ip = mask;
	}

	/**
	 * Set a key/mask combination for the destination IP
	 * @param dst_ip The destination IP
	 * @param mask The associated mask. Each '1' bit in the mask denotes that the same bit in the key has to match. A '0' bit denotes a wildcard bit (don't care bit).
	 */
	void set_dst_ip(uint32_t dst_ip, uint32_t mask = (uint32_t)-1)
	{
		ensure_unbound();
		key.dst_ip = dst_ip;
		this->mask.dst_ip = mask;
	}

	/**
	 * Set a key/mask combination for the IP protocol
	 * @param ip_prot The IP protocol
	 * @param mask The associated mask. Each '1' bit in the mask denotes that the same bit in the key has to match. A '0' bit denotes a wildcard bit (don't care bit).
	 */
	void set_ip_prot(uint8_t ip_prot, uint8_t mask = (uint8_t)-1)
	{
		ensure_unbound();
		key.ip_prot = ip_prot;
		this->mask.ip_prot = mask;
	}

	/**
	 * Set a key/mask combination for the TOS field
	 * @param ip_tos The TOS field. Only the upper 6 bits are used (according to the OF 1.0 specification).
	 * @param mask The associated mask. Each '1' bit in the mask denotes that the same bit in the key has to match. A '0' bit denotes a wildcard bit (don't care bit).
	 */
	void set_ip_tos(uint8_t ip_tos, uint8_t mask = 0xfc)
	{
		ensure_unbound();
		if ((ip_tos & 0x02) || (mask & 0x02))
			throw errors::UNUSED_BITS_SET_IN_HEADER_FIELD;
		key.ip_tos = ip_tos;
		this->mask.ip_tos = mask;
	}

	/**
	 * Set a key/mask combination for the source port
	 * @param src_port The source port
	 * @param mask The associated mask. Each '1' bit in the mask denotes that the same bit in the key has to match. A '0' bit denotes a wildcard bit (don't care bit).
	 */
	void set_src_port(uint16_t src_port, uint16_t mask = (uint16_t)-1)
	{
		ensure_unbound();
		key.src_port = src_port;
		this->mask.src_port = mask;
	}

	/**
	 * Set a key/mask combination for the destination port
	 * @param dst_port The destination port
	 * @param mask The associated mask. Each '1' bit in the mask denotes that the same bit in the key has to match. A '0' bit denotes a wildcard bit (don't care bit).
	 */
	void set_dst_port(uint16_t dst_port, uint16_t mask = (uint16_t)-1)
	{
		ensure_unbound();
		key.dst_port = dst_port;
		this->mask.dst_port = mask;
	}

	/**
	 * The parser inside the OpenFlow core sets irrelevant header fields to zero. By calling this method, the same is done here.
	 * This should always be called before a flow is tried to be added to the CAM.
	 */
	void eliminate_unneccessary_wildcards();

public:
	/**
	 * Converts the source mac address to a std::string
	 * @return The std::string
	 */
	std::string src_mac_to_string()
	{
		return mac_to_string(key.src_mac, mask.src_mac);
	}
	/**
	 * Converts the destination mac address to a std::string
	 * @return The std::string
	 */
	std::string dst_mac_to_string()
	{
		return mac_to_string(key.dst_mac, mask.dst_mac);
	}
private:
	std::string mac_to_string(mac_t key, mac_t mask);
public:
	/**
	 * Converts the ethertype to a std::string
	 * @return The std::string
	 */
	std::string ethertype_to_string();
	/**
	 * Converts the source IP to a std::string
	 * @return The std::string
	 */
	std::string src_ip_to_string()
	{
		return ip_to_string(key.src_ip, mask.src_ip);
	}
	/**
	 * Converts the destination IP to a std::string
	 * @return The std::string
	 */
	std::string dst_ip_to_string()
	{
		return ip_to_string(key.dst_ip, mask.dst_ip);
	}
private:
	std::string ip_to_string(uint32_t key, uint32_t mask);
public:
	/**
	 * Converts the IP protocol type to a std::string
	 * @return The std::string
	 */
	std::string ipproto_to_string();
private:
	std::string port_to_string(uint16_t key, uint16_t mask);
public:
	/**
	 * Converts the source port to a std::string
	 * @return The std::string
	 */
	std::string src_port_to_string()
	{
		return port_to_string(key.src_port, mask.src_port);
	}
	/**
	 * Converts the destination port to a std::string
	 * @return The std::string
	 */
	std::string dst_port_to_string()
	{
		return port_to_string(key.dst_port, mask.dst_port);
	}
	/**
	 * Converts the action to a std::string
	 * @return The std::string
	 */
	std::string action_to_string();
	/**
	 * Converts the priority to a std::string
	 * @return The std::string
	 */
	std::string priority_to_string();
	/**
	 * Converts the egress port to a std::string
	 * @return The std::string
	 */
	std::string action_egress_port_to_string();
	/**
	 * Converts the flow entry to a std::string
	 * @return The std::string
	 */
	std::string to_string();

};


}
