/*
 * Copyright (c) 2014, 2015 Felix Wallaschek
 * felix@elektronenversand.de
 *
 * This file is part of the NetFPGA 10G UPB OpenFlow Switch project:
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
 * 
 * This file is free code: you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License version 2.1 as
 * published by the Free Software Foundation.
 *
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this project. If not, see <http://www.gnu.org/licenses/>.
 */
 
/*
 * We apply actions to packets in FIFO here.
 * Also statistics keeping is managed here.
 * Statistics are read-only. Overflow handling has to be done in software.
 *
 * The statistics address for a flow is build as follows:
 * C_AXI_BASE_ADDR | (MATCH_ADDR << 4)
 * Where MATCH_ADDR is the address of the flow entry.
 * The bits [4:3] define the type of statistic that is read:
 * 00: number of packets matched by that flow
 * 01: number of bytes matched by that flow
 * 10: timestamp of last match by that flow
 * 11: undefined
 *
 * To read the current timestamp of the card read address:
 * C_AXI_BASE_ADDR | ((2**(C_MATCH_ADDR_WIDTH)+1) << 4)
 * (For C_MATCH_ADDR_WIDTH=12 and C_AXI_BASE_ADDR=0xC0000000: 0xC0010010)
 */
`include "parameters.v"
module processor #(
    parameter C_AXI_BASE_ADDR = 32'hFFFFFFFF,
    parameter C_AXI_HIGH_ADDR = 32'h00000000,
    parameter C_AXI_TIMESTMP_ADDR = C_AXI_BASE_ADDR | ((2**(C_MATCH_ADDR_WIDTH)+1) << 4),
    parameter C_DMA_PORT = 0,
    parameter C_DMA_FIRST_EXTERNAL_PORT = 0,
    parameter C_DMA_LAST_EXTERNAL_PORT = 0,
    parameter C_BRIDGED_ETH_A_VPORT = 0,
    parameter C_BRIDGED_ETH_B_VPORT = 0
    ) (
    input clk,
    input reset,
    
    input action_match,
    input action_valid,
    input [1:0] action_type,
    input [C_OUT_PORT_WIDTH-1:0] action_port,
    input [C_OUT_PORT_WIDTH-1:0] action_vport,
    input [C_MATCH_ADDR_WIDTH-1:0] action_match_addr,
    
    input [C_AXIS_TDATA_WIDTH-1:0] s_axis_tdata,
    input [C_AXIS_TKEEP_WIDTH-1:0] s_axis_tkeep,
    input s_axis_tvalid,
    input s_axis_tlast,
    input [C_IN_PORT_WIDTH-1:0] s_axis_tuser_in_port,
    input [C_IN_PORT_WIDTH-1:0] s_axis_tuser_in_vport,
    input [C_OUT_PORT_WIDTH-1:0] s_axis_tuser_out_port,
    input [C_OUT_PORT_WIDTH-1:0] s_axis_tuser_out_vport,
    input [C_PACKET_LENGTH_WIDTH-1:0] s_axis_tuser_packet_length,
    output reg s_axis_tready,
    
    
    output reg [C_AXIS_TDATA_WIDTH-1:0] m_axis_tdata,
    output reg [C_AXIS_TKEEP_WIDTH-1:0] m_axis_tkeep,
    output reg m_axis_tvalid,
    output reg m_axis_tlast,
    output reg [C_IN_PORT_WIDTH-1:0] m_axis_tuser_in_port,
    output reg [C_IN_PORT_WIDTH-1:0] m_axis_tuser_in_vport,
    output reg [C_OUT_PORT_WIDTH-1:0] m_axis_tuser_out_port,
    output reg [C_OUT_PORT_WIDTH-1:0] m_axis_tuser_out_vport,
    output reg [C_PACKET_LENGTH_WIDTH-1:0] m_axis_tuser_packet_length,
    input m_axis_tready,
    
    
    input s_axi_aclk,
    input s_axi_aresetn,
    input s_axi_arvalid,
    input [31:0] s_axi_araddr,
    input [2:0] s_axi_arprot,
    input s_axi_rready,
    output reg s_axi_arready, 
    output reg s_axi_rvalid, 
    output reg [31:0] s_axi_rdata,
    output reg [1:0] s_axi_rresp
);

reg m_axis_tvalid_internal;
reg [C_OUT_PORT_WIDTH-1:0] m_axis_tuser_out_port_internal, m_axis_tuser_out_vport_internal;
reg ready_to_get_data = 0;

always_ff @(posedge clk) begin

    if (m_axis_tvalid_internal) begin

        m_axis_tdata <= s_axis_tdata;
        m_axis_tkeep <= s_axis_tkeep;
        m_axis_tlast <= s_axis_tlast;

        m_axis_tuser_packet_length <= s_axis_tuser_packet_length;
        m_axis_tuser_in_port <= s_axis_tuser_in_port;
        m_axis_tuser_in_vport <= s_axis_tuser_in_vport;
        m_axis_tuser_out_port <= m_axis_tuser_out_port_internal;
        m_axis_tuser_out_vport <= m_axis_tuser_out_vport_internal;

        m_axis_tvalid <= 1;

    end else begin

        m_axis_tvalid <= 0;
    end

    if (reset) begin
        m_axis_tvalid <= 0;
    end

end

always_comb begin
    s_axis_tready = 
            ~(m_axis_tvalid & ~m_axis_tready) // there is valid data forwarded to the master interface but the master interface cannot process the data => do not accept further data from the slave interface
            &   ready_to_get_data; // we are ready to process new data
end

logic fifo_action_match;
logic fifo_action_valid;
logic [1:0] fifo_action_type;
logic [C_OUT_PORT_WIDTH-1:0] fifo_action_port;
logic [C_OUT_PORT_WIDTH-1:0] fifo_action_vport;
logic [C_MATCH_ADDR_WIDTH-1:0] fifo_action_match_addr;

logic [C_COUNTER_WIDTH-1:0] timestamp=0;

logic [C_MATCH_ADDR_WIDTH:0] raddr = 0; // we need one additional bit here to handle C_TIMESTMP_ADDR
logic [C_MATCH_ADDR_WIDTH:0] raddr_next;
logic [1:0] statistics_type_selector, statistics_type_selector_next;

logic [C_COUNTER_WIDTH-1:0] rdata_bytes;
logic [C_COUNTER_WIDTH-1:0] rdata_time;
logic [C_COUNTER_WIDTH-1:0] rdata_pkts;


typedef enum logic [2:0] {
    WAIT_FOR_ACTION, TRANSMIT, DROP
} state_t;

state_t state = WAIT_FOR_ACTION;
logic [1:0] atype; // type is reserved keyword so we just use atype :-/
logic [C_OUT_PORT_WIDTH-1:0] port;
logic [C_OUT_PORT_WIDTH-1:0] vport;


/*
 * FIFO to store actions.
 * We need this as parser might be pipelined and output tuples while we
 * haven't handled the previous one yet.
 */
logic fifo_ren = 0;
logic fifo_empty;

action_fifo fifo (
  .clk(clk), 
  .rst(reset), 
  .din({action_type,action_match_addr,action_vport,action_port,action_match,action_valid}),
  .wr_en(action_valid),
  .rd_en(fifo_ren),
  .dout({fifo_action_type,fifo_action_match_addr,fifo_action_vport,fifo_action_port,fifo_action_match,fifo_action_valid}),
  .empty(fifo_empty)
);

logic stat_inc_en=0; // update statistics when set to o
logic[C_MATCH_ADDR_WIDTH-1:0] stat_inc_addr=0;

statistics stat_pkt_counter (
    .clk(clk),
    .reset(reset),
    .addr(stat_inc_addr),
    .we(stat_inc_en),
    .din(1),
    .rclk(s_axi_aclk),
    .raddr(raddr),
    .rdata(rdata_pkts)
);
statistics stat_byte_counter (
    .clk(clk),
    .reset(reset),
    .addr(stat_inc_addr),
    .we(stat_inc_en),
    .din({{32-C_PACKET_LENGTH_WIDTH{1'b0}},s_axis_tuser_packet_length}),
    .rclk(s_axi_aclk),
    .raddr(raddr),
    .rdata(rdata_bytes)
);
statistics #(.C_MODE(1)) stat_timest (
    .clk(clk),
    .reset(reset),
    .addr(stat_inc_addr),
    .we(stat_inc_en),
    .din(timestamp),
    .rclk(s_axi_aclk),
    .raddr(raddr),
    .rdata(rdata_time)
);

always_comb begin
        
    if(state == TRANSMIT) begin
        case (atype)
            2'b00: begin //Forward / Drop
                m_axis_tuser_out_port_internal = port;
                m_axis_tuser_out_vport_internal = vport;
            end
            2'b01: begin // In_Port
                m_axis_tuser_out_port_internal = {{(C_OUT_PORT_WIDTH-1){1'b0}},1'b1} << s_axis_tuser_in_port; // packet is sent back to the incoming port
                m_axis_tuser_out_vport_internal = {{(C_OUT_PORT_WIDTH-1){1'b0}},1'b1} << s_axis_tuser_in_vport; // in case of DMA: also the incoming vport is copied to the outgoing vport
            end
            2'b10: begin // All (Broadcast)

                if (s_axis_tuser_in_port != C_DMA_PORT) begin // the packet does not come from the DMA

                    m_axis_tuser_out_port_internal = ~({{(C_OUT_PORT_WIDTH-1){1'b0}},1'b1} << s_axis_tuser_in_port); // broadcast to all ports, except the incoming one
                    m_axis_tuser_out_vport_internal = {{(C_OUT_PORT_WIDTH-1){1'b0}},1'b1} << C_BRIDGED_ETH_B_VPORT; // on the DMA: broadcast only to the ETH port

                end else begin // the packet comes from the DMA

                    if (s_axis_tuser_in_vport != C_BRIDGED_ETH_B_VPORT) begin // the packet does do not come from the ETH port

                        m_axis_tuser_out_port_internal = '1; // broadcast to all ports including the DMA port (the incoming port in this case)...
                        m_axis_tuser_out_vport_internal = {{(C_OUT_PORT_WIDTH-1){1'b0}},1'b1} << C_BRIDGED_ETH_B_VPORT; // on the DMA: broadcast only to the ETH port

                    end else begin // the packet comes from the ETH port

                        m_axis_tuser_out_port_internal = ~({{(C_OUT_PORT_WIDTH-1){1'b0}},1'b1} << C_DMA_PORT); // send to all ports except the DMA
                        m_axis_tuser_out_vport_internal = 1; // the vport is not relevant (default value)

                    end
                end
            end
            2'b11: begin // send to Controller

                m_axis_tuser_out_port_internal = 0;
                m_axis_tuser_out_vport_internal = 0;

                if (s_axis_tuser_in_port >= C_DMA_FIRST_EXTERNAL_PORT && s_axis_tuser_in_port <= C_DMA_LAST_EXTERNAL_PORT) begin // packet comes from one of the external ports (10G, Interconnect)

                    m_axis_tuser_out_port_internal = 8'b00000001 << C_DMA_PORT;
                    m_axis_tuser_out_vport_internal = 8'b00000001 << port;

                end else begin

                    if (s_axis_tuser_in_port == C_DMA_PORT) begin // packet comes from the DMA

                        if (s_axis_tuser_in_vport == C_BRIDGED_ETH_B_VPORT) begin // packet comes from the ETH port

                            m_axis_tuser_out_port_internal = 8'b00000001 << C_DMA_PORT;
                            m_axis_tuser_out_vport_internal = 8'b00000001 << C_BRIDGED_ETH_A_VPORT;

                        end

                    end
                end
            end
        endcase
    end
    else begin
        m_axis_tuser_out_port_internal = 0;
        m_axis_tuser_out_vport_internal = 0;
    end
    case(state)
        DROP: m_axis_tvalid_internal = 0;
        TRANSMIT: m_axis_tvalid_internal = s_axis_tvalid;
        WAIT_FOR_ACTION: m_axis_tvalid_internal = 0;
    endcase
end


always_ff @(posedge clk) begin
    timestamp <= timestamp + 1;
    fifo_ren <= 0;
    stat_inc_en <= 0;
    if((state == WAIT_FOR_ACTION && ~fifo_empty) || ((state == TRANSMIT || state == DROP) && s_axis_tvalid && s_axis_tready && s_axis_tlast && ~fifo_empty)) begin
        if(fifo_action_port != 0 || fifo_action_vport != 0 || fifo_action_type != 0) begin
            port <= fifo_action_port;
            vport <= fifo_action_vport;
            atype <= fifo_action_type;
            state <= TRANSMIT;
            ready_to_get_data <= 1;
            stat_inc_addr <= fifo_action_match_addr;
            stat_inc_en <= 1;
        end else begin
            state <= DROP;
            ready_to_get_data <= 1;
        end
        fifo_ren <= 1;
    end
    else if((state == TRANSMIT || state == DROP) && s_axis_tvalid && s_axis_tready && s_axis_tlast && fifo_empty) begin
        state <= WAIT_FOR_ACTION;
        ready_to_get_data <= 0;
    end
    if(reset) begin
        state <= WAIT_FOR_ACTION;
        ready_to_get_data <= 0;
        timestamp <= 0;
    end
end



/*
 * AXI4Lite interface to read statistics
 */
typedef enum {RD_STATE_WAITING, RD_STATE_PROCESSING, RD_STATE_ANSWERING} rd_states;
rd_states rd_state = RD_STATE_WAITING, rd_state_next;
wire s_axi_araddr_in_range = (s_axi_araddr & (C_AXI_BASE_ADDR ~^ C_AXI_HIGH_ADDR)) == C_AXI_BASE_ADDR;
wire [31:0] s_axi_araddr_masked_32b_aligned = (s_axi_araddr & (C_AXI_BASE_ADDR ^ C_AXI_HIGH_ADDR)) >> 2; // reduce address lines from byte adressing to (32 bit) word adressing

always_ff @(posedge s_axi_aclk) begin
    rd_state <= rd_state_next;
    raddr <= raddr_next;
	statistics_type_selector <= statistics_type_selector_next;
end

always_comb begin
    s_axi_arready = 0;
    s_axi_rvalid = 0;
    s_axi_rresp = 0; // OK
    s_axi_rdata = 32'hFBADBEEF;
    
    
    raddr_next = raddr;
    rd_state_next = rd_state;
    
    case (rd_state)
        RD_STATE_WAITING: begin
            s_axi_arready = 1;
            if (s_axi_arvalid && s_axi_araddr_in_range) begin
                if(s_axi_araddr == C_AXI_TIMESTMP_ADDR) begin
                    rd_state_next = RD_STATE_ANSWERING;
                end else begin
                    rd_state_next = RD_STATE_PROCESSING;
                end
                raddr_next = s_axi_araddr_masked_32b_aligned>>2; //reduce  address lines again to remove stat.-type selector
				statistics_type_selector_next = s_axi_araddr_masked_32b_aligned;
            end
        end
        
        RD_STATE_PROCESSING: begin
            rd_state_next = RD_STATE_ANSWERING;
        end
        
        RD_STATE_ANSWERING: begin
            case (statistics_type_selector)
                2'b00: begin
                    s_axi_rdata = rdata_pkts;
                end
                2'b01: begin
                    s_axi_rdata = rdata_bytes;
                end
                2'b10: begin
                    s_axi_rdata = rdata_time;
                end
            endcase
            if(raddr == ((C_AXI_TIMESTMP_ADDR & (C_AXI_BASE_ADDR ^ C_AXI_HIGH_ADDR)) >> 4))
                s_axi_rdata = timestamp;
            s_axi_rvalid = 1;
            if (s_axi_rready)
                rd_state_next = RD_STATE_WAITING;
        end
    endcase
    if(~s_axi_aresetn) begin
        rd_state_next = RD_STATE_WAITING;
    end
end

endmodule
