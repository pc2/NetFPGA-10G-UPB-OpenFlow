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
#include <string>
#include <config_port_associations.hpp>

namespace upb {

/**
 * This class provides access to physical ports on the NetFPGA
 */
class port : config_port_associations {

public:

	/**
	 * Datatype for port/vport combinations which is used in various functions
	 */
	typedef union {
		uint16_t portvport;
		struct {
			uint8_t port;
			uint8_t vport;
		};
	} port_t;
	/**
	 * Parses a device name and tries to extract a card_id and a port_id
	 * @param device_name The name of the device (e.g. /dev/nf10av0)
	 * @param card_id Reference to a variable that should get the id of the card
	 * @param port_id Reference to a variable that should get the id of the port
	 * @return true on success
	 */
	static bool parse_port_name(const char *device_name, uint32_t &card_id, uint32_t &port_id);
	/**
	 * Transforms a port_id to an ingress port bit pattern
	 * @param port_id The port_id
	 * @return Bit pattern of the ingress port
	 */
	static uint16_t ingress_port(uint32_t port_id);
	/**
	 * Transforms a port_id to an egress port bit pattern
	 * @param port_id The port_id
	 * @return Bit pattern of the egress port
	 */
	static uint16_t egress_port(uint32_t port_id);
	/**
	 * Transforms an ingress port to a std::string
	 * @param port_key Key of the ingress port (as data type port_t)
	 * @param port_mask Mask of the ingress port (as data type port_t)
	 * @return The human readable port name as a std::string
	 */
	static std::string ingress_ports_to_string(port_t port_key, port_t port_mask);
	/**
	 * Transforms an egress port to a std::string
	 * @param port The ingress port (as data type port_t)
	 * @return The human readable port name as a std::string
	 */
	static std::string egress_ports_to_string(port_t port);

};


}
