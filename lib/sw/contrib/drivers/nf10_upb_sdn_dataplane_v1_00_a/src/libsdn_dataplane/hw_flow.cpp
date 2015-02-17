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

#include "hw_flow.hpp"
#include <bit_lib.hpp>
#include <string.h>
#include <arpa/inet.h>
#include <flow.hpp>

namespace upb {

hw_flow::hw_flow(const flow &flow)
{
	memset(key, 0, sizeof(key));
	memset(mask, 0, sizeof(mask));
	convert_tuple(key, flow.key);
	convert_tuple(mask, flow.mask);
}

void hw_flow::convert_tuple(hw_tuple &tuple_hw, const flow::tuple &tuple_sw)
{
// see tuple_t.v in nf10_upb_ofswitch_v1_00_a:
/*
    logic[2:0] port;    // 242:240
    logic[2:0] vport;   // 239:237
    logic[47:0] dmac;   // 236:189
    logic[47:0] smac;   // 188:141
    logic[15:0] typ;    // 140:125
    logic[11:0] vid;    // 124:113
    logic[2:0] pcp;     // 112:110
    logic[31:0] sip;    // 109:78
    logic[31:0] dip;    //  77:46
    logic[7:0] prot;    //  45:38
    logic[5:0] tos;     //  37:32
    logic[15:0] tsp;    //  31:16
    logic[15:0] tdp;    //  15:0
*/

	transfer_bits<>(tuple_hw, 0, 15, tuple_sw.dst_port);
	transfer_bits<>(tuple_hw, 16, 31, tuple_sw.src_port);
	// TODO: Test TOS (up to now it's untested)
	uint32_t ip_tos_shifted = tuple_sw.ip_tos >> 2;
	transfer_bits<>(tuple_hw, 32, 37, ip_tos_shifted); // only the upper 6 bits are important
	transfer_bits<>(tuple_hw, 38, 45, tuple_sw.ip_prot);
	transfer_bits<>(tuple_hw, 46, 77, tuple_sw.dst_ip);
	transfer_bits<>(tuple_hw, 78, 109, tuple_sw.src_ip);
	// TODO: Test VLAN (up to now it's untested)
	transfer_bits<>(tuple_hw, 110, 112, tuple_sw.vlan_pcp);
	transfer_bits<>(tuple_hw, 113, 124, tuple_sw.vlan_id);
	transfer_bits<>(tuple_hw, 125, 140, tuple_sw.ethertype);
	transfer_bits<>(tuple_hw, 141, 188, tuple_sw.src_mac[0]);
	transfer_bits<>(tuple_hw, 189, 236, tuple_sw.dst_mac[0]);
	transfer_bits<>(tuple_hw, 237, 239, tuple_sw.ingress_port.vport);
	transfer_bits<>(tuple_hw, 240, 242, tuple_sw.ingress_port.port);
}

}
