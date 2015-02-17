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

#include "cam.hpp"
#include <string.h>
#include <errors.hpp>
#include <math.h>
#include <iostream>
#include <iomanip>
#include <boost/bind.hpp>
#include <boost/log/trivial.hpp>
#include <bit_lib.hpp>

#define div_round_up(x,y) (x/y + (x%y?1:0))

namespace upb {

namespace b = boost;

cam::cam(const b::shared_ptr<axi_bus_connector> &axi_bus, uint32_t axi_address, hw_priority_t hw_priority) :
	axi_bus(axi_bus),
	axi_address(axi_address),
	hw_priority_(hw_priority),
	crc_generator(0x04C11DB7, 0xFFFFFFFF, 0, false, false)
{
	uint32_t module_name = axi_read(module_name);
	if (module_name != *(const uint32_t*)"MAC")
		throw errors::CAM_COULD_NOT_BE_IDENTIFIED;

	// read out the cam size from the FPGA core and resize table to hw table size
	cam_table.resize(axi_read(cam_entries));

	uint32_t cam_size_log2 = floor(log(table_size() + 0.5)/log(2.0));
	cam_address_mask = ~((uint32_t)-1 << cam_size_log2);

	// generate the only mask that is compatible with a cam (direct match mask, every supported bit is '1')
	for (uint32_t i = 0; i < hw_flow::hw_tuple_size_32bits; i++) {
		if (i != hw_flow::hw_tuple_size_32bits-1) {

			// not the last word
			cam_compatible_mask[i] = (uint32_t)-1;
		} else {

			// last word
			cam_compatible_mask[i] = (uint32_t)-1 >> (32 - (hw_flow::hw_tuple_size_bits % 32));
		}
	}

	// bring the table and the FSM inside the CAM into a known state
	reset_table();

	BOOST_LOG_TRIVIAL(info) << axi_bus->netfpga_id() << ": CAM: Found hardware CAM with " << cam_table.size() << " entries";
}

cam::~cam()
{
	reset_table(); // just to be sure
}

void cam::set_flow_statistics(const b::shared_ptr<flow_statistics> &stat, uint32_t stat_offset)
{
	this->stat = stat;
	this->stat_offset = stat_offset;
}

cam::table_size_t cam::table_size()
{
	return cam_table.size();
}

cam::hw_priority_t cam::hw_priority()
{
	return hw_priority_;
}

bool cam::add_flow(const b::shared_ptr<flow> &new_flow)
{
	// optimize the flow for the tcam
	new_flow->eliminate_unneccessary_wildcards();

	// get a hardware representation of the flow
	hw_flow new_hw_flow(*new_flow);

	// check if the key is compatible with the cam: It must be a direct match (all mask bits '1')
	for (uint32_t i = 0; i < hw_flow::hw_tuple_size_32bits; i++) {
		if (new_hw_flow.mask[i] != cam_compatible_mask[i])
			return false;
	}

	/*
	 * generate the CRC32 hash value...
	 */
	crc_generator.reset();

	if (hw_flow::hw_tuple_size_bits % 8) { // tuple size is not a multiple of 8 -> process the remaining bits first

		crc_generator.process_bits(
			((uint8_t*)&new_hw_flow.key[0])[hw_flow::hw_tuple_size_bits / 8],
			hw_flow::hw_tuple_size_bits % 8
		);
	}

	for (int32_t i = hw_flow::hw_tuple_size_bits / 8 - 1; i >= 0; i--) {

		crc_generator.process_byte(((uint8_t*)&new_hw_flow.key[0])[i]);
	}

	uint32_t hashed_address = crc_generator.checksum() & cam_address_mask;

	// add a '1' bit at the end ("flow entry activated" bit), this bit enables the flow entry in the hw cam
	uint32_t temp = 1;
	uint32_t last_bit_pos = hw_flow::hw_tuple_size_bits;
	transfer_bits<>(new_hw_flow.key, last_bit_pos, last_bit_pos, temp);

	if (cam_table[hashed_address].flow_ptr == NULL) {

		// found a free cam table entry

		// activate the statistics counters
		stat->activate_counters(stat_offset + hashed_address);

		// put the shared flow object into the cam table
		cam_table[hashed_address].flow_ptr = new_flow.get();
		// tell the (shared) flow object how this flow can be deleted: call delete_flow with parameter "hashed_address"
		cam_table[hashed_address].flow_ptr->bind(
			shared_from_this(),
			b::bind(&cam::delete_flow, shared_from_this(), hashed_address),
			b::bind(&cam::update_action, shared_from_this(), _1, hashed_address),
			b::bind(&flow_statistics::get_flow_statistics, stat, stat_offset + hashed_address, _1, _2, _3, true)
		);

		// write the action to the hardware cam
		axi_write(cam[hashed_address][0], new_flow->get_action_hw_representation());

		// copy the flow entry to the hardware cam
		for (uint32_t i = 0; i < hw_flow::hw_tuple_size_32bits; i++) {
			axi_write(cam[hashed_address][i], new_hw_flow.key[i]);
		}

		// store the last word of the key that we can activate/reactivate the cam line later without recalculating the key
		cam_table[hashed_address].key_last_word = new_hw_flow.key[hw_flow::hw_tuple_size_32bits-1];

		BOOST_LOG_TRIVIAL(debug) << axi_bus->netfpga_id() << ": CAM: Insert at 0x" << std::hex << std::setw(4) << std::setfill('0') << hashed_address << " " << new_flow->to_string();

	} else {
/*
		BOOST_LOG_TRIVIAL(debug)
			<< axi_bus->netfpga_id()
			<< ": CAM: Hash collision at 0x"
			<< std::hex << std::setw(4) << std::setfill('0') << hashed_address
			//<< " (" << std::dec << used_entries << " entries)."
			<< " " << new_flow->to_string();*/
		return false;
	}

	used_entries++;
	return true;
}

void cam::delete_flow(uint32_t flow_pos)
{
	assert(flow_pos < cam_table.size());
	assert(cam_table[flow_pos].flow_ptr); // there must be a flow installed at position flow_pos

	// deactivate flow in hardware cam
	deactivate_flow_hw(flow_pos);

	// deactivate the statistics counters
	stat->deactivate_counters(stat_offset + flow_pos);

	// read out the statistics (just for debug messages)
	uint64_t packet_count;
	uint64_t byte_count;
	bpt::ptime utc_last_packet;

	cam_table[flow_pos].flow_ptr->get_statistics(packet_count, byte_count, utc_last_packet);

	// remove the pointer to the flow
	cam_table[flow_pos].flow_ptr = NULL;

	used_entries--;

	BOOST_LOG_TRIVIAL(debug) << axi_bus->netfpga_id() << ": CAM: Deleted entry 0x" << std::hex << std::setw(4) << std::setfill('0') << flow_pos
		<< std::dec
		<< " Statistics: Packets: " << packet_count << " Bytes: " << byte_count << " Time since last packet: "
		<< (utc_last_packet.is_not_a_date_time() ? -1 : bpt::time_duration(bpt::microsec_clock::universal_time()-utc_last_packet).total_milliseconds()) << " ms";
}

void cam::update_action(uint32_t action_hardware_representation, uint32_t flow_pos_cam_table)
{
	axi_write(cam[flow_pos_cam_table][0], action_hardware_representation); // rewrite action, FSM goes to WR_ISTATE_PROCESSING
	axi_write(cam[flow_pos_cam_table][7], cam_table[flow_pos_cam_table].key_last_word); // rewrite the last word of the key, including the valid bit (FSM goes back to state WR_ISTATE_WAITING_FOR_ACTION)
	BOOST_LOG_TRIVIAL(debug) << axi_bus->netfpga_id() << ": CAM: Modify action #" << flow_pos_cam_table << ": 0x" << std::hex << std::setw(4) << std::setfill('0') << cam_table[flow_pos_cam_table].flow_ptr->action_egress_port_to_string();
}

void cam::activate_flow_hw(uint32_t flow_pos_cam_table)
{
	axi_write(cam[flow_pos_cam_table][0], cam_table[flow_pos_cam_table].flow_ptr->get_action_hw_representation()); // rewrite action, FSM goes to WR_ISTATE_PROCESSING
	axi_write(cam[flow_pos_cam_table][7], cam_table[flow_pos_cam_table].key_last_word); // rewrite the last word of the key, including the valid bit (FSM goes back to state WR_ISTATE_WAITING_FOR_ACTION)
}

void cam::deactivate_flow_hw(uint32_t flow_pos_cam_table)
{
	axi_write(cam[flow_pos_cam_table][0], cam_table[flow_pos_cam_table].flow_ptr->get_action_hw_representation()); // rewrite action, FSM goes to WR_ISTATE_PROCESSING
	axi_write(cam[flow_pos_cam_table][7], 0); // clear the last word including the valid bit (FSM goes back to state WR_ISTATE_WAITING_FOR_ACTION)
}

void cam::reset_table()
{
	for (uint32_t i = 0; i < table_size(); i++) {
		axi_write(cam[i][0], 0); // ensure to be in state WR_ISTATE_PROCESSING by potentially writing a DROP action to that address (FORWARD without port/vport bits set)
		axi_write(cam[i][7], 0); // clear the last word including the valid bit (FSM goes back to state WR_ISTATE_WAITING_FOR_ACTION)
	}
}

} // namespace upb
