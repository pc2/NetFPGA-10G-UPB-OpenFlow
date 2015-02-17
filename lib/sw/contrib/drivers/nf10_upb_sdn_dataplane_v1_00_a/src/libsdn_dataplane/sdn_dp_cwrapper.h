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

#include <stddef.h>
#include <stdint.h>
#include <errors.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initializes the logger
 */
void upb_init(void);

/**
 * Helper function to print a message to the log
 * @param text The text message
 */
void upb_log_error(const char *text);

/**
 * Creates a new instance of the class sdn_dataplane
 * @param dataplane_id Id of the data plane (e.g. 0 for /dev/nf10a, 1 for /dev/nf10b)
 * @param data_plane_name Optional name of the data plane
 * @return 0 if successful, an error code otherwise (see errors.inc)
 */
enum errors upb_create_data_plane(uint32_t dataplane_id, const char *data_plane_name);
/**
 * Sets the corresponding egress port bits for the named port
 * @param port_name Name of the port (e.g. /dev/nf10av0)
 * @param egress_port_bits Pointer to an initially empty uint16_t which receives the bits corresponding to the named port
 * @param data_plane_id Pointer to a variable that gets the associated data plane ID
 * @return 0 if successful, an error code otherwise (see errors.inc)
 */
enum errors upb_add_egress_port_bits(const char *port_name, uint16_t *egress_port_bits, uint32_t *data_plane_id);
/**
 * Translates a named port to the ingress port id
 * @param port_name Name of the port (e.g. /dev/nf10av0)
 * @param ingress_port Pointer to an uint16_t which receives the port ID
 * @param data_plane_id Pointer to an uint32_t that gets the associated data plane ID
 * @return 0 if successful, an error code otherwise (see errors.inc)
 */
enum errors upb_get_ingress_port(const char *port_name, uint16_t *ingress_port, uint32_t *data_plane_id);
/**
 * Translates a named port to a data plane id
 * @param port_name Name of the port (e.g. /dev/nf10av0)
 * @param data_plane_id Pointer to an uint32_t that gets the associated data plane ID
 * @return 0 if successful, an error code otherwise (see errors.inc)
 */
enum errors upb_get_data_plane_id(const char *port_name, uint32_t *data_plane_id);
/**
 * Type of a MAC address
 */
typedef uint8_t upb_mac_t[6];
/**
 * Type of a flow priority
 */
typedef uint32_t upb_priority_t;
/**
 * Supported actions
 */
enum upb_action_t {
	UPB_ACTION_UNSET, 			//!< it is not allowed to pass this to upb_add_flow
	UPB_ACTION_FORWARD,			//!< Forward to a specified port/vport
	UPB_ACTION_BACK_TO_IN_PORT,	//!< Send the packet back to the ingress port
	UPB_ACTION_BROADCAST,		//!< Broadcast the packet
	UPB_ACTION_SEND_TO_CONTROLLER //!< Send the packet to the software switch
};

/**
 * C flow entry representation (analogous to the definition in flow.hpp)
 */
struct upb_flow_t {
	uint16_t ingress_port;
	uint8_t src_mac[6], dst_mac[6];
	uint16_t ethertype;
	uint16_t vlan_id;
	uint8_t vlan_pcp;
	uint32_t src_ip;
	uint32_t dst_ip;
	uint8_t ip_prot;
	uint8_t ip_tos;
	uint16_t src_port;
	uint16_t dst_port;
};
/**
 * Sets all field to zero
 */
void upb_clear_flow(struct upb_flow_t *flow);

/**
 * Adds a flow entry to the automatic flow management (class sdn_dataplane)
 * @param data_plane_id The id of the data plane
 * @param flow_p Pointer to a uint64_t variable that receives the unique flow entry ID (for further references)
 * @param unsupported Set to true if the flow is unsupported
 * @param key The key part of the flow entry
 * @param mask The mask part of the flow entry
 * @param priority The priority of the flow entry
 * @param action One action of upb_action_t
 * @param egress_port Only for UPB_ACTION_FORWARD: The egress ports to which the packet should be sent (use upb_add_egress_port_bits(...))
 * @return 0 if successful, an error code otherwise (see errors.inc)
 */
enum errors upb_add_flow(
	uint32_t data_plane_id,
	uint64_t *flow_p,
	bool unsupported,
	struct upb_flow_t *key,
	struct upb_flow_t *mask,
	upb_priority_t priority,
	enum upb_action_t action,
	uint16_t egress_port
);

/**
 * Modifies the action of the flow entry
 * @param flow_p The ID of the flow
 * @param action One action of upb_action_t
 * @param egress_port Only for UPB_ACTION_FORWARD: The egress ports to which the packet should be sent (use upb_add_egress_port_bits(...))
 * @param reset_counters: if set to true: Resets the statistics
 * @return 0 if successful, an error code otherwise (see errors.inc)
 */
enum errors upb_modify_flow(
	uint64_t flow_p,
	enum upb_action_t action,
	uint16_t egress_port,
	bool reset_counters
);

/**
 * Deletes a flow entry
 * @param flow_p The ID of the flow
 * @return 0 if successful, an error code otherwise (see errors.inc)
 */
enum errors upb_delete_flow(uint64_t flow_p);

/**
 * Requests statistics for a flow entry
 * @param flow_p The ID of the flow
 * @param packets Pointer to a uint64_t variable that receives the packet count
 * @param bytes Pointer to a uint64_t variable that receives the byte count
 * @param ms_since_last_packet Pointer to a uint64_t variable that receives the duration since the last packet arrived (in milliseconds). If no packet yet arrived this value is 0xffffffffffffffff.
 * @return 0 if successful, an error code otherwise (see errors.inc)
 */
enum errors upb_get_statistics(uint64_t flow_p, uint64_t *packets, uint64_t *bytes, uint64_t *ms_since_last_packet);

#ifdef __cplusplus
}
#endif
