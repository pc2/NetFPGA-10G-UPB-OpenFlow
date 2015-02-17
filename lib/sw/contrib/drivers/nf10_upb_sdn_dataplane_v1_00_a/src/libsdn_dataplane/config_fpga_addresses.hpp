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

namespace upb {

class config_fpga_addresses {

public:
	/**
	 * AXI-4 Lite bus address of the TCAM
	 */
	const uint32_t tcam_address = 0xa0000000;
	/**
	 * Hardware priority of the TCAM
	 */
	const uint32_t tcam_hw_priority = 0;
	/**
	 * AXI-4 Lite bus address of the CAM
	 */
	const uint32_t cam_address = 0xb0000000;
	/**
	 * Hardware priority of the CAM
	 */
	const uint32_t cam_hw_priority = 1; // the cam has a higher priority
	/**
	 * Clock frequency of the OpenFlow core on the FPGA
	 */
	const float clock_frequency_hz_statistics_counters = 120e6f;
	/**
	 * Polling interval for the statistics. Must be chosen such that the 32 bit counters cannot overrun on the FPGA
	 */
	const uint32_t default_statistics_poll_interval_ms = 10000;
	/**
	 * AXI-4 Lite bus address of the statistics module
	 */
	const uint32_t statistics_module_address = 0xc0000000;
	/**
	 * Number of static lookup entries in the lookup table engine
	 */
	const uint32_t action_processor_static_lookup_entries = 8;
	/**
	 * Number of vport lookup entries in the lookup table engine
	 */
	const uint32_t action_processor_vport_lookup_entries = 8;

};

}
