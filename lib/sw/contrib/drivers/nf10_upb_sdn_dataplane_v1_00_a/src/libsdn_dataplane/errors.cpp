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

namespace upb {

namespace errors_text {

char const * error_text[] = {

	"", // OK
	"The NetFPGA device could not be opened", // COULD_NOT_OPEN_NETFPGA_DEVICE
	"Could not map NetFPGA IO memory space to user space", // COULD_NOT_MAP_TO_USER_SPACE
	"Error during erase of flash block", // ERROR_ERASING_FLASH_BLOCK
	"Error during verification of flash contents", // ERROR_VERIFYING_FLASH_CONTENTS
	"Error during programming of a flash buffer", // ERROR_PROGRAMMING_FLASH_BUFFER
	"Could not open the file for flash programming", // ERROR_COULD_NOT_OPEN_FILE_FOR_FLASH_PROG
	"Timeout while waiting for flash ready", // FLASH_TIMEOUT
	"TCAM could not be identified", // TCAM_COULD_NOT_BE_IDENTIFIED
	"Tried to do operations on a flow which is not installed", // FLOW_NOT_INSTALLED
	"Tried to modify a already installed flow", // FLOW_ALREADY_INSTALLED
	"Tried to set unused bits in key/mask", // UNUSED_BITS_SET_IN_HEADER_FIELD
	"An invalid port number was given", // INVALID_PORT_NUMBER
	"A C++ exception was raised", // CPP_EXCEPTION
	"An unknown exception was raised", // UNKNOWN_EXCEPTION
	"The dataplane already exists", // DATAPLANE_ALREADY_EXISTS
	"The specified port does not follow the UPB NetFPGA port scheme", // INVALID_PORT_NAME
	"The specified data plane does not exist", // DATA_PLANE_DOES_NOT_EXIST
	"Internal error: Could not insert into flow table", // COULD_NOT_INSERT_INTO_FLOW_TABLE
	"Reference id of flow unknown", // COULD_NOT_FIND_FLOW_BY_REF_ID
	"CAM could not be identified", // CAM_COULD_NOT_BE_IDENTIFIED
};


}

}
