typedef struct packed {
    logic valid;        // 243
    logic[2:0] port;    // 242:240
    logic[2:0] vport;   // 239:237
    logic[47:0] dmac;   // 236:189
    logic[47:0] smac;   // 188:141
    logic[15:0] typ;    // 140:125
    logic[11:0] vid;    // 124:113 - little endian !!!
    logic[2:0] pcp;     // 112:110
    logic[31:0] sip;    // 109:78
    logic[31:0] dip;    //  77:46
    logic[7:0] prot;    //  45:38
    logic[5:0] tos;     //  37:32
    logic[15:0] tsp;    //  31:16
    logic[15:0] tdp;    //  15:0
} tuple_t;
