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

#include "tcam.hpp"
#include <string.h>
#include <errors.hpp>
#include <boost/bind.hpp>
#include <boost/log/trivial.hpp>
#include <bit_lib.hpp>

#define div_round_up(x,y) (x/y + (x%y?1:0))

namespace upb {

tcam::tcam(const b::shared_ptr<axi_bus_connector> &axi_bus, uint32_t axi_address, hw_priority_t hw_priority) :
	axi_bus(axi_bus),
	axi_address(axi_address),
	hw_priority_(hw_priority)
{
	uint32_t module_name = axi_read(module_name);
	if (module_name != *(const uint32_t*)"MACT")
		throw errors::TCAM_COULD_NOT_BE_IDENTIFIED;

	// read out the cam size from the FPGA core and resize table to hw table size
	tcam_table.resize(axi_read(tcam_entries));
	BOOST_LOG_TRIVIAL(info) << axi_bus->netfpga_id() << ": TCAM: Found hardware TCAM with " << tcam_table.size() << " entries";
	// disable all entries in the hw table
	deactivate_all_flows_hw();
}

tcam::~tcam()
{
	deactivate_all_flows_hw(); // just to be sure
}

void tcam::set_flow_statistics(const b::shared_ptr<flow_statistics> &stat, uint32_t stat_offset)
{
	this->stat = stat;
	this->stat_offset = stat_offset;
}

tcam::table_size_t tcam::table_size()
{
	return tcam_table.size();
}

tcam::hw_priority_t tcam::hw_priority()
{
	return hw_priority_;
}

bool tcam::add_flow(const b::shared_ptr<flow> &new_flow)
{

	bool found_pos = false;
	uint32_t flow_pos;

	for (flow_pos = 0; flow_pos < tcam_table.size(); flow_pos++) {

		if (!tcam_table[flow_pos].flow_ptr) {

			// first empty slot -> reached the end of the flow table -> we can put the flow here
			found_pos = true;
			break;

		} else if (tcam_table[flow_pos].flow_ptr->get_priority() < new_flow->get_priority()) {

			 // we reached a flow with a smaller priority -> we can use this position if we shift this flow and all following flows by one line
			found_pos = shift_all_flows_one_line(flow_pos);
			break;
		}
	}

	if (!found_pos) {
		//BOOST_LOG_TRIVIAL(debug) << axi_bus->netfpga_id() << ": TCAM: I'm full. Could not add flow";
		return false;
	}

	BOOST_LOG_TRIVIAL(debug) << axi_bus->netfpga_id() << ": TCAM: Insert at " << flow_pos << " " << new_flow->to_string();

	// put the shared flow object into the tcam table
	tcam_table[flow_pos].flow_ptr = new_flow.get();
	// tell the (shared) flow object how this flow can be unbound from the tcam: call delete_flow with parameter "flow_pos"
	tcam_table[flow_pos].flow_ptr->bind(
		shared_from_this(),
		b::bind(&tcam::delete_flow, shared_from_this(), flow_pos),
		b::bind(&tcam::update_action, shared_from_this(), _1, flow_pos),
		b::bind(&flow_statistics::get_flow_statistics, stat, stat_offset + flow_pos, _1, _2, _3, true)
	);
	// copy the flow entry to the hardware tcam
	add_flow_hw(flow_pos, *new_flow);
	// activate the statistics counters
	stat->activate_counters(stat_offset + flow_pos);
	// activate the entry in the hardware
	activate_flow_hw(flow_pos);

	return true;
}

void tcam::delete_flow(uint32_t flow_pos)
{
	assert(flow_pos < tcam_table.size());
	assert(tcam_table[flow_pos].flow_ptr); // there must be a flow installed at position flow_pos

	// deactivate flow in hardware TCAM
	deactivate_flow_hw(flow_pos);

	// deactivate the counters
	stat->deactivate_counters(stat_offset + flow_pos);

	// read out the statistics (just for debug messages)
	uint64_t packet_count;
	uint64_t byte_count;
	bpt::ptime utc_last_packet;

	tcam_table[flow_pos].flow_ptr->get_statistics(packet_count, byte_count, utc_last_packet);

	// remove the pointer to the flow
	tcam_table[flow_pos].flow_ptr = NULL;

	bool moved_entries = false;

	// move all following flow entries one line up
	for (uint32_t i = flow_pos + 1; i < tcam_table.size(); i++) {

		if (tcam_table[i].flow_ptr) { // there is a valid entry
			move_flow(i, i-1);
			moved_entries = true;
		} else { // first empty entry...
			break; //...no more flow entries can follow => break loop
		}
	}
	BOOST_LOG_TRIVIAL(debug) << axi_bus->netfpga_id() << ": TCAM: Deleted entry #" << flow_pos << (moved_entries ? " and moved succeeding entries one line up." : "")
		<< " Statistics: Packets: " << packet_count << " Bytes: " << byte_count << " Time since last packet: "
		<< (utc_last_packet.is_not_a_date_time() ? -1 : bpt::time_duration(bpt::microsec_clock::universal_time()-utc_last_packet).total_milliseconds()) << " ms";
}

void tcam::update_action(uint32_t action_hardware_representation, uint32_t flow_pos)
{
	BOOST_LOG_TRIVIAL(debug) << axi_bus->netfpga_id() << ": TCAM: Modify action #" << flow_pos << ": " << tcam_table[flow_pos].flow_ptr->action_egress_port_to_string();
	axi_write(actions[flow_pos], action_hardware_representation);
}

bool tcam::shift_all_flows_one_line(uint32_t flow_pos)
{
	assert(flow_pos < tcam_table.size());
	assert(tcam_table[flow_pos].flow_ptr); // there must be a flow installed at position flow_pos

	// try to find the first empty position after flow_pos
	for (uint32_t i = flow_pos + 1; i < tcam_table.size(); i++) {
		if (!tcam_table[i].flow_ptr) {
			// found empty slot i

			// shift all flows by one position
			for (uint32_t j = i; j > flow_pos; j--) {
				move_flow(j-1, j);
			}
			BOOST_LOG_TRIVIAL(debug) << axi_bus->netfpga_id() << ": TCAM: Moved " << i-flow_pos << " entries to get an appropriate free slot";
			return true;
		}
	}
	return false;
}

void tcam::move_flow(uint32_t old_pos, uint32_t new_pos)
{
	assert(old_pos < tcam_table.size());
	assert(new_pos < tcam_table.size());
	assert(tcam_table[old_pos].flow_ptr);
	assert(!tcam_table[new_pos].flow_ptr);

	// 1. copy the flow to the new position in the tcam
	add_flow_hw(new_pos, *tcam_table[old_pos].flow_ptr);

	// 2. activate the new entry
	stat->activate_counters(stat_offset + new_pos);
	activate_flow_hw(new_pos);

	// now we have two duplicate TCAM entries in the hardware and we can safely deactivate the old one
	deactivate_flow_hw(old_pos);
	stat->deactivate_counters(stat_offset + old_pos);

	// 3. move the statistics counters
	stat->move_counters_values(stat_offset + old_pos, stat_offset + new_pos);

	// update the software representation of these two flows

	// 1. copy the flow entry pointer to the new position
	tcam_table[new_pos].flow_ptr = tcam_table[old_pos].flow_ptr;
	// 2. clear the old pointer
	tcam_table[old_pos].flow_ptr = NULL;

	// 3. update the delete handler
	tcam_table[new_pos].flow_ptr->update_binding(
		b::bind(&tcam::delete_flow, shared_from_this(), new_pos),
		b::bind(&tcam::update_action, shared_from_this(), _1, new_pos),
		b::bind(&flow_statistics::get_flow_statistics, stat, stat_offset + new_pos, _1, _2, _3, true)
	);
}

void tcam::add_flow_hw(uint32_t flow_pos, flow &new_flow)
{
	// get a hardware representation of the flow
	hw_flow new_hw_flow(new_flow);

	memset(tcam_temp_write_data, 0, sizeof(tcam_temp_write_data)); // clear the temp table

	for (uint32_t lut = 0; lut < tcam_fpga_lut_count_per_line; lut++) { // over all 49 LUTs

		uint32_t key_5_bit = get_bits<>(
			new_hw_flow.key,
			lut * tcam_fpga_lut_width,
			std::min(
				lut * tcam_fpga_lut_width + tcam_fpga_lut_width -1,
				hw_flow::hw_tuple_size_bits -1
			)
		);

		uint32_t mask_5_bit = get_bits<>(
			new_hw_flow.mask,
			lut * tcam_fpga_lut_width,
			std::min(
				lut * tcam_fpga_lut_width + tcam_fpga_lut_width -1,
				hw_flow::hw_tuple_size_bits -1
			)
		);

		// the LUT on the FPGA needs 32 shift (write) operations to be filled
		for (uint32_t lut_pos = 0; lut_pos < tcam_fpga_lut_size; lut_pos++) {

			if (((
					~mask_5_bit				// '0' bit in the mask means wildcard (the corresponding key bit does not count anymore)
				|	~(lut_pos ^ key_5_bit)	// or there is a match with the key bit (xnor)
			) & (tcam_fpga_lut_size-1)) == (tcam_fpga_lut_size-1)) { // all 5 bits must be one

				// set this LUT bit to '1'
				tcam_temp_write_data[lut / 32][lut_pos] |= (uint32_t)1 << (lut % 32);
			}
		}
	}

	// write the precalculated LUT contents into one line of the TCAM

	// the shift operations are performed the other way round (see srl32e xilinx documentation)
	for (int lut_pos = tcam_fpga_lut_size-1; lut_pos >= 0; lut_pos--) {

		// 32 LUTs can be shifted simultaneously with one bus access (@ 49 LUTs: 2 32bit bus accesses)
		for (uint32_t word_access = 0; word_access < div_round_up(tcam_fpga_lut_count_per_line, 32); word_access++) {

			axi_write(srl_access[flow_pos][word_access], tcam_temp_write_data[word_access][lut_pos]);
		}
	}

	// add action to tcam
	axi_write(actions[flow_pos], new_flow.get_action_hw_representation());
}

void tcam::activate_flow_hw(uint32_t flow_pos_tcam_table)
{
	assert(flow_pos_tcam_table < tcam_table.size());

	uint32_t active_bits_word_index = flow_pos_tcam_table / 32;
	uint32_t active_bits_bit_index = flow_pos_tcam_table % 32;

	active_bits_copy[active_bits_word_index] |= ((uint32_t)1 << active_bits_bit_index);
	axi_write(active_bits[active_bits_word_index], active_bits_copy[active_bits_word_index]);
}

void tcam::deactivate_flow_hw(uint32_t flow_pos_tcam_table)
{
	assert(flow_pos_tcam_table < tcam_table.size());

	uint32_t active_bits_word_index = flow_pos_tcam_table / 32;
	uint32_t active_bits_bit_index = flow_pos_tcam_table % 32;

	active_bits_copy[active_bits_word_index] &= ~((uint32_t)1 << active_bits_bit_index);
	axi_write(active_bits[active_bits_word_index], active_bits_copy[active_bits_word_index]);
}

void tcam::deactivate_all_flows_hw()
{
	for (uint32_t i = 0; i < div_round_up(tcam_table.size(), 32); i++)
		axi_write(active_bits[i], 0);
}

} // namespace upb
