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

#include <stdint.h>
#include <config_port_associations.hpp>

namespace upb {

constexpr uint8_t config_port_associations::egress_port_association[config_port_associations::netfpga_nic_virtual_ports];
constexpr uint8_t config_port_associations::egress_vport_association[config_port_associations::netfpga_nic_virtual_ports];
constexpr uint8_t config_port_associations::ingress_port_association[config_port_associations::netfpga_nic_virtual_ports];
constexpr uint8_t config_port_associations::ingress_vport_association[config_port_associations::netfpga_nic_virtual_ports];
constexpr const char* config_port_associations::port_friendly_names[config_port_associations::netfpga_nic_virtual_ports];
constexpr const char* config_port_associations::port_interface_names[config_port_associations::netfpga_nic_virtual_ports];

}
