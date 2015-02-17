parameter C_IN_PORT_WIDTH = 3;
parameter C_NUM_INPUTS = 2**C_IN_PORT_WIDTH;
parameter C_OUT_PORT_WIDTH = 8;
parameter C_AXIS_TDATA_WIDTH = 256;
parameter C_AXIS_TKEEP_WIDTH = 32;
parameter C_PACKET_LENGTH_WIDTH = 14;
parameter C_MATCH_ADDR_WIDTH = 12; // "4096 Flows ought to be enough for anybody." --nobody, ever
                                   // Make sure this is enough to hold all flow table entries,
                                   // but remember that statistics ram will be scaled to hold
                                   // statistics for 2**C_MATCH_ADDR_WIDTH entries.
parameter C_COUNTER_WIDTH = 32; // Have a look at statistics_ram.v before changeing this
