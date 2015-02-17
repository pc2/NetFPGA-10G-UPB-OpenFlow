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

class config_port_associations {

public:

	/**
	 * Number of virtual ports (nf10xv0..nf10xv5)
	 */
	const static uint32_t netfpga_nic_virtual_ports = 6; // nf10xv0..nf10xv5

	/**
	 * Egress port "port" value for each virtual port
	 */
	static constexpr uint8_t egress_port_association[netfpga_nic_virtual_ports] = {
		0x01,	// nf10xv0 - 10G Port 1 (closest to the PCIe slot)
		0x02,	// nf10xv1 - 10G Port 2 (closest to the PCIe slot)
		0x04,	// nf10xv2 - 10G Port 3 (closest to the PCIe slot)
		0x08,	// nf10xv3 - 10G Port 4 (closest to the PCIe slot)
		0x10,	// nf10xv4 - Interconnect
		0x20	// nf10xv5 - general purpose Linux NIC port nf10x0 (DMA port on FPGA)
	};

	/**
	 * Egress port "vport" value for each virtual port
	 */
	static constexpr uint8_t egress_vport_association[netfpga_nic_virtual_ports] = {
		0x01,	// nf10xv0 - 10G Port 1 (closest to the PCIe slot)
		0x01,	// nf10xv1 - 10G Port 2 (closest to the PCIe slot)
		0x01,	// nf10xv2 - 10G Port 3 (closest to the PCIe slot)
		0x01,	// nf10xv3 - 10G Port 4 (closest to the PCIe slot)
		0x01,	// nf10xv4 - Interconnect
		0x40	// nf10xv5 - general purpose Linux NIC port nf10x0 (DMA port on FPGA)
	};

	/**
	 * Ingress port "port" ID for each virtual port
	 */
	static constexpr uint8_t ingress_port_association[netfpga_nic_virtual_ports] = {
		0,	// nf10xv0 - 10G Port 1 (closest to the PCIe slot)
		1,	// nf10xv1 - 10G Port 2 (closest to the PCIe slot)
		2,	// nf10xv2 - 10G Port 3 (closest to the PCIe slot)
		3,	// nf10xv3 - 10G Port 4 (closest to the PCIe slot)
		4,	// nf10xv4 - Interconnect
		5	// nf10xv5 - general purpose Linux NIC port nf10x0 (DMA port on FPGA)
	};

	/**
	 * Ingress port "vport" ID for each virtual port
	 */
	static constexpr uint8_t ingress_vport_association[netfpga_nic_virtual_ports] = {
		0,	// nf10xv0 - 10G Port 1 (closest to the PCIe slot)
		0,	// nf10xv1 - 10G Port 2 (closest to the PCIe slot)
		0,	// nf10xv2 - 10G Port 3 (closest to the PCIe slot)
		0,	// nf10xv3 - 10G Port 4 (closest to the PCIe slot)
		0,	// nf10xv4 - Interconnect
		6	// nf10xv5 - general purpose Linux NIC port nf10x0 (DMA port on FPGA)
	};

	/**
	 * Human readable names for each virtual port
	 */
	static constexpr const char* port_friendly_names[netfpga_nic_virtual_ports] = {
		"1", "2", "3", "4", "IC", "ETH"
	};

	/**
	 * Human readable device names for each virtual port
	 */
	static constexpr const char* port_interface_names[netfpga_nic_virtual_ports] = {
		"nf10xv0", "nf10xv1", "nf10xv2", "nf10xv3", "nf10xv4", "nf10xv5"
	};
};

}
