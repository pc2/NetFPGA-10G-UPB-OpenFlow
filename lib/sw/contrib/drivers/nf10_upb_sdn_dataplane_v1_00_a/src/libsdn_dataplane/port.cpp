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

#include "port.hpp"
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/qi_lit.hpp>
#include <boost/spirit/include/qi_char_.hpp>
#include <boost/spirit/include/qi_uint.hpp>
#include <errors.hpp>

namespace upb {

namespace qi = boost::spirit::qi;

bool port::parse_port_name(const char *device_name, uint32_t &card_id, uint32_t &port_id)
{
	char const* first = device_name;
	char const* last = device_name + strlen(device_name);

	using qi::parse;
	using boost::spirit::qi::lit;
	using boost::spirit::qi::char_;
	using boost::spirit::qi::uint_;

	uint32_t card_id_temp, port_id_temp;

	bool result = parse(
		first,
		last,
		-lit("/dev/") >> "nf10" >> char_('a','z') >> 'v' >> uint_,
		card_id_temp,
		port_id_temp
	);

	card_id_temp -= 'a';

	if (	result
		&&	card_id_temp < config_port_associations::netfpga_nic_virtual_ports
	) {
		card_id = card_id_temp;
		port_id = port_id_temp;
		return true;
	}

	return false;
}

uint16_t port::ingress_port(uint32_t port_id)
{
	if (port_id < netfpga_nic_virtual_ports) {
		return ingress_port_association[port_id] | (uint16_t)ingress_vport_association[port_id] << 8;
	} else
		throw errors::INVALID_PORT_NUMBER;
}

uint16_t port::egress_port(uint32_t port_id)
{
	if (port_id < netfpga_nic_virtual_ports) {
		return egress_port_association[port_id] | (uint16_t)egress_vport_association[port_id] << 8;
	} else
		throw errors::INVALID_PORT_NUMBER;
}

std::string port::egress_ports_to_string(port_t port)
{
	for (uint32_t i = 0; i < netfpga_nic_virtual_ports; i++) {
		if (
				port.port == egress_port_association[i]
			&&	port.vport == egress_vport_association[i]
		) {
			return std::string(port_friendly_names[i]);
		}
	}
	if (port.portvport == 0)
		return std::string("(nowhere)");

	return std::string("(unknown)");
}

std::string port::ingress_ports_to_string(port_t port_key, port_t port_mask)
{
	std::string ret_string;
	bool first_finding = true;
	for (uint32_t i = 0; i < netfpga_nic_virtual_ports; i++) {

		port_t port_value;
		port_value.port = ingress_port_association[i];
		port_value.vport = ingress_vport_association[i];

		if (((port_key.portvport ^ port_value.portvport) & port_mask.portvport) == 0) { // significant bits must match

			if (!first_finding) {
				ret_string += " ";
			}
			ret_string += port_friendly_names[i];
			first_finding = false;
		}
	}

	return ret_string;
}


}
