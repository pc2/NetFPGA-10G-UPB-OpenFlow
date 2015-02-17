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
#include <boost/shared_ptr.hpp>
#include <flow_statistics.hpp>

namespace upb {

namespace b = boost;

class flow;

class hw_flow_table_base {

public:
	virtual ~hw_flow_table_base() {}

	/**
	 * Attaches a flow_statistics (shared) object to the hardware lookup table (CAM/TCAM) class. This has to be done right after constructing a hardware lookup table object
	 * @param stat Shared pointer to an instance of the flow_statistics class
	 * @param stat_offset Table position offset for the entries associated with this lookup table
	 */
	virtual void set_flow_statistics(const b::shared_ptr<flow_statistics> &stat, uint32_t stat_offset) = 0;

	typedef uint32_t table_size_t;

	/**
	 * Returns the hardware table (CAM/TCAM) size which was retrieved from the FPGA core
	 * @return The table size in lines
	 */
	virtual table_size_t table_size() = 0;

	typedef uint32_t hw_priority_t;


	/**
	 * Returns the hardware priority of the lookup table (CAM/TCAM)
	 * @return The hardware priority
	 */
	virtual hw_priority_t hw_priority() = 0;

	const static hw_priority_t highest_priority = (hw_priority_t)-1;
	const static hw_priority_t lowest_priority = 0;

	/**
	 * Adds a flow table entry to the hardware lookup table (CAM or TCAM)
	 * @param new_flow Shared pointer to the flow object
	 * @return true if successful
	 */
	virtual bool add_flow(const b::shared_ptr<flow> &new_flow) = 0;

};

}
