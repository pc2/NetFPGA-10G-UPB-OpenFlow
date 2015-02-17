
UPB_ERROR_PREFIX(OK), //!< No error
UPB_ERROR_PREFIX(COULD_NOT_OPEN_NETFPGA_DEVICE),//!< The NetFPGA device could not be opened
UPB_ERROR_PREFIX(COULD_NOT_MAP_TO_USER_SPACE),//!< Could not map NetFPGA IO memory space to user space
UPB_ERROR_PREFIX(ERROR_ERASING_FLASH_BLOCK),//!< Error during erase of flash block
UPB_ERROR_PREFIX(ERROR_VERIFYING_FLASH_CONTENTS),//!< Error during verification of flash contents
UPB_ERROR_PREFIX(ERROR_PROGRAMMING_FLASH_BUFFER),//!< Error during programming of a flash buffer
UPB_ERROR_PREFIX(ERROR_COULD_NOT_OPEN_FILE_FOR_FLASH_PROG), //!< Could not open the file for flash programming
UPB_ERROR_PREFIX(FLASH_TIMEOUT), //!< Timeout while waiting for flash ready
UPB_ERROR_PREFIX(TCAM_COULD_NOT_BE_IDENTIFIED), //!< TCAM could not be identified
UPB_ERROR_PREFIX(FLOW_NOT_INSTALLED), //!< Tried to do operations on a flow which is not installed
UPB_ERROR_PREFIX(FLOW_ALREADY_INSTALLED), //!< Tried to modify a already installed flow
UPB_ERROR_PREFIX(UNUSED_BITS_SET_IN_HEADER_FIELD), //!< Tried to set unused bits in key/mask
UPB_ERROR_PREFIX(INVALID_PORT_NUMBER), //!< An invalid port number was given
UPB_ERROR_PREFIX(CPP_EXCEPTION), //!< A C++ exception was raised
UPB_ERROR_PREFIX(UNKNOWN_EXCEPTION), //!< An unknown exception was raised
UPB_ERROR_PREFIX(DATAPLANE_ALREADY_EXISTS), //!< The dataplane already exists
UPB_ERROR_PREFIX(INVALID_PORT_NAME), //!< The specified port does not follow the UPB NetFPGA port scheme
UPB_ERROR_PREFIX(DATA_PLANE_DOES_NOT_EXIST), //!< The specified data plane does not exist
UPB_ERROR_PREFIX(COULD_NOT_INSERT_INTO_FLOW_TABLE), //!< Internal error: Could not insert into flow table
UPB_ERROR_PREFIX(COULD_NOT_FIND_FLOW_BY_REF_ID), //!< Reference id of flow unknown
UPB_ERROR_PREFIX(CAM_COULD_NOT_BE_IDENTIFIED), //!< CAM could not be identified
