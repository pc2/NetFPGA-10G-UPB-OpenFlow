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
 
`timescale 1ns / 1ps
module nf10_upb_switch_tb;

	// Inputs
	reg reset;
	reg clk100;
	
    reg [255:0] m_axis_tdata=0;
    reg [31:0] m_axis_tkeep={32{1'b1}};
    reg m_axis_tvalid=0;
    reg m_axis_tlast=0;
    reg [2:0] m_axis_tuser_in_port=0;
    reg [2:0] m_axis_tuser_in_vport=0;
    reg [7:0] m_axis_tuser_out_port=0;
    reg [7:0] m_axis_tuser_out_vport=0;
    reg [13:0] m_axis_tuser_packet_length=0;
    wire m_axis_tready;
    
    
    wire [255:0] s_axis_tdata;
    wire [31:0] s_axis_tkeep;
    wire s_axis_tvalid;
    wire s_axis_tlast;
    wire [2:0] s_axis_tuser_in_port;
    wire [2:0] s_axis_tuser_in_vport;
    wire [7:0] s_axis_tuser_out_port;
    wire [7:0] s_axis_tuser_out_vport;
    wire [13:0] s_axis_tuser_packet_length;
    reg s_axis_tready = 1;
    
    wire clk150_bufg, clk150, clk200_bufg, clk200, clk300_bufg, clk300, clk300_90_bufg, clk300_90;
    wire clk_fb;
    wire pll_locked;

	// Instantiate the Unit Under Test (UUT)
	 
	
	wire qdr_c_k;					// K
	wire qdr_c_k_n;				// K
	wire qdr_c_c;					// C
	wire qdr_c_c_n;				// Cn
	logic qdr_c_cq, qdr_c_cq_memory;					// CQ
	logic qdr_c_cq_n, qdr_c_cq_n_memory;				// CQn
	wire [18:0] qdr_c_sa;		// A[18:0]
	wire qdr_c_r_n;				// RPS
	logic [35:0] qdr_c_q, qdr_c_q_memory;			// Q[35:0]
	wire qdr_c_w_n;				// WPS
	wire [3:0] qdr_c_bw_n;		// BWS[3:0]
	wire [35:0] qdr_c_d;			// D[35:0]

    integer simtime = 0;
	nf10_upb_switch uut (
		.clk(clk), 
		.clk2x(clk300), 
		.clk2x90(clk300_90), 
		.reset(reset),
		.qdr_c_k(qdr_c_k),
		.qdr_c_k_n(qdr_c_k_n),
		.qdr_c_c(qdr_c_c),
		.qdr_c_c_n(qdr_c_c_n),
		.qdr_c_cq(qdr_c_cq),
		.qdr_c_cq_n(qdr_c_cq_n),
		.qdr_c_sa(qdr_c_sa),
		.qdr_c_r_n(qdr_c_r_n),
		.qdr_c_q(qdr_c_q),
		.qdr_c_w_n(qdr_c_w_n),
		.qdr_c_bw_n(qdr_c_bw_n),
		.qdr_c_d(qdr_c_d),
		
		
		.s_axis_tdata(m_axis_tdata),
		.s_axis_tkeep(m_axis_tkeep),
		.s_axis_tvalid(m_axis_tvalid),
		.s_axis_tlast(m_axis_tlast),
		.s_axis_tuser_in_port(m_axis_tuser_in_port),
		.s_axis_tuser_in_vport(m_axis_tuser_in_vport),
		.s_axis_tuser_out_port(m_axis_tuser_out_port),
		.s_axis_tuser_out_vport(m_axis_tuser_out_vport),
		.s_axis_tuser_packet_length(m_axis_tuser_packet_length),
		.s_axis_tready(m_axis_tready),
		
		.m_axis_tdata(s_axis_tdata),
		.m_axis_tkeep(s_axis_tkeep),
		.m_axis_tvalid(s_axis_tvalid),
		.m_axis_tlast(s_axis_tlast),
		.m_axis_tuser_in_port(s_axis_tuser_in_port),
		.m_axis_tuser_in_vport(s_axis_tuser_in_vport),
		.m_axis_tuser_out_port(s_axis_tuser_out_port),
		.m_axis_tuser_out_vport(s_axis_tuser_out_vport),
		.m_axis_tuser_packet_length(s_axis_tuser_packet_length),
		.m_axis_tready(s_axis_tready)
	);
	
	wire [255:0] tdata[0:12];
	assign tdata[0] = 256'h1FFEDEADAFFEDEADAFFEDEADAFFEDEADAFFEDEAD000000000000111111111111;
    assign tdata[1] = 256'h2FFEDEADAFFEDEADAFFEDEADAFFEDEADAFFEDEADAFFEDEADAFFEDEADAFFEDEAD;

    assign tdata[2] = 256'h1FFEDEADAFFEDEADAFFEDEADAFFEDEADAFFEDEAD111111111111000000000000;
    assign tdata[3] = 256'h2FFEDEADAFFEDEADAFFEDEADAFFEDEADAFFEDEADAFFEDEADAFFEDEADAFFEDEAD;
    assign tdata[4] = 256'h3FFEDEADAFFEDEADAFFEDEADAFFEDEADAFFEDEADAFFEDEADAFFEDEADAFFEDEAD;

    assign tdata[5] = 256'h1FFEDEADAFFEDEADAFFEDEADAFFEDEADAFFEDEAD111111111111222222222222;
    assign tdata[6] = 256'h2FFEDEADAFFEDEADAFFEDEADAFFEDEADAFFEDEADAFFEDEADAFFEDEADAFFEDEAD;
    assign tdata[7] = 256'h3FFEDEADAFFEDEADAFFEDEADAFFEDEADAFFEDEADAFFEDEADAFFEDEADAFFEDEAD;
    assign tdata[8] = 256'h4FFEDEADAFFEDEADAFFEDEADAFFEDEADAFFEDEADAFFEDEADAFFEDEADAFFEDEAD;
    assign tdata[9] = 256'h5FFEDEADAFFEDEADAFFEDEADAFFEDEADAFFEDEADAFFEDEADAFFEDEADAFFEDEAD;
    assign tdata[10] = 256'h6FFEDEADAFFEDEADAFFEDEADAFFEDEADAFFEDEADAFFEDEADAFFEDEADAFFEDEAD;
    assign tdata[11] = 256'h7FFEDEADAFFEDEADAFFEDEADAFFEDEADAFFEDEADAFFEDEADAFFEDEADAFFEDEAD;
    
    assign tdata[12] = 256'h0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF0123456789ABCDE;
	
	wire tlast[0:12];
	assign tlast[0] = 0;
	assign tlast[1] = 1;
	assign tlast[2] = 0;
	assign tlast[3] = 0;
	assign tlast[4] = 1;
	assign tlast[5] = 0;
	assign tlast[6] = 0;
	assign tlast[7] = 0;
	assign tlast[8] = 0;
	assign tlast[9] = 0;
	assign tlast[10] = 0;
	assign tlast[11] = 1;
    assign tlast[12] = 1;
	
	wire [2:0] tuser_in_port[0:12];
	assign tuser_in_port[0] = 0;
	assign tuser_in_port[1] = 0;
	assign tuser_in_port[2] = 1;
	assign tuser_in_port[3] = 1;
	assign tuser_in_port[4] = 1;
	assign tuser_in_port[5] = 1;
	assign tuser_in_port[6] = 1;
	assign tuser_in_port[7] = 1;
	assign tuser_in_port[8] = 1;
	assign tuser_in_port[9] = 1;
	assign tuser_in_port[10] = 1;
	assign tuser_in_port[11] = 1;
	assign tuser_in_port[12] = 2;
	
	wire [13:0] tuser_packet_length[0:12];
	assign tuser_packet_length[0] = 64;
	assign tuser_packet_length[1] = 64;
	assign tuser_packet_length[2] = 96;
	assign tuser_packet_length[3] = 96;
	assign tuser_packet_length[4] = 96;
	assign tuser_packet_length[5] = 224;
	assign tuser_packet_length[6] = 224;
	assign tuser_packet_length[7] = 224;
	assign tuser_packet_length[8] = 224;
	assign tuser_packet_length[9] = 224;
	assign tuser_packet_length[10] = 224;
	assign tuser_packet_length[11] = 224;
	assign tuser_packet_length[12] = 42;
	
	/*
    *		Clocking
    */
    

    PLL_BASE #(

	    .CLKFBOUT_MULT(6),        // Multiplication factor for all output clocks
	    .CLKIN_PERIOD(10.0),	     // Clock period (ns) of input clock on CLKIN
	
	    .CLKOUT0_DIVIDE(4),       // Division factor for CLKOUT0 (1 to 128)
	    .CLKOUT1_DIVIDE(2),       // Division factor for CLKOUT1 (1 to 128)
	    .CLKOUT2_DIVIDE(2),       // Division factor for CLKOUT2 (1 to 128)
	    .CLKOUT2_PHASE(90.0),     // Phase shift (degrees) for CLKOUT2 (0.0 to 360.0)
	    .CLKOUT3_DIVIDE(3)        // Division factor for CLKOUT3 (1 to 128)

    ) main_pll (
	    .CLKFBOUT(clk_fb),        // General output feedback signal
	    .CLKOUT0(clk150_bufg),         // One of six general clock output signals
	    .CLKOUT1(clk300_bufg),         // One of six general clock output signals
	    .CLKOUT2(clk300_90_bufg),      // One of six general clock output signals
	    .CLKOUT3(clk200_bufg),    // One of six general clock output signals
	    .LOCKED(pll_locked),      // Active high PLL lock signal
	    .CLKFBIN(clk_fb),         // Clock feedback input
	    .CLKIN(clk100),           // Clock input
	    .RST() // Asynchronous PLL reset
    );

    
    BUFG bufg_clk150 (
	    .I(clk150_bufg),
	    .O(clk)
    );

    BUFG bufg_clk200 (
	    .I(clk200_bufg),
	    .O(clk200)
    );

    BUFG bufg_clk300 (
	    .I(clk300_bufg),
	    .O(clk300)
    );

    BUFG bufg_clk300_90 (
	    .I(clk300_90_bufg),
	    .O(clk300_90)
    );
	
	
	CY7C1515JV18 qdr_c (
		.D(qdr_c_d),
		.Q(qdr_c_q_memory),
		.A(qdr_c_sa),
		.RPS_n(qdr_c_r_n),
		.WPS_n(qdr_c_w_n),
		.BW_n(qdr_c_bw_n),
		.K(qdr_c_k),
		.K_n(qdr_c_k_n),
		.C(qdr_c_c),
		.C_n(qdr_c_c_n),
		.CQ(qdr_c_cq_memory),
		.CQ_n(qdr_c_cq_n_memory)
	);
	
	// simulate delay on the PCB's traces and on the FPGA's IOB
	always @(*) begin
		qdr_c_cq <= #0 qdr_c_cq_memory;
		qdr_c_cq_n <= #0 qdr_c_cq_n_memory;
		
		qdr_c_q <= #6 qdr_c_q_memory;
	end
	
	
	
	
	
	

	initial begin
		// Initialize Inputs
		clk100 = 0;
		reset = 0;

		// Wait 100 ns for global reset to finish
		#100;
        
		// Add stimulus here

	end
	reg [3:0] current = 0;
	always @(posedge clk) begin
	    simtime = simtime + 1;
	    s_axis_tready <=1;
	    if(simtime > 23 && simtime < 30) begin
	        s_axis_tready <=0;
        end
	    else begin
	        m_axis_tvalid <= 1;
	        if(m_axis_tready) begin
	            current <= current + 1;
	            if(current > 11) begin
	                current <= 0;
	                
                end
                if(current == 11 ) begin
                    m_axis_tvalid <= 0;
                end
	        end
	    end
        if(simtime==30) begin
            current <= current +1 ;
        end
	    
	end
	always @(*) begin
        m_axis_tdata = tdata[current];
        m_axis_tlast = tlast[current];
        m_axis_tuser_in_port = tuser_in_port[current];
        m_axis_tuser_packet_length = tuser_packet_length[current];
    end
	
    always
		#5 clk100 = ~clk100;
endmodule
