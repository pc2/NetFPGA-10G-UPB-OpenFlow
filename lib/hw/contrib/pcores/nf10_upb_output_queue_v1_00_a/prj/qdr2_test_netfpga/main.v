/*
 * UPB Output Queue core
 *
 * Copyright (c) 2014, 2015 Jörg Niklas
 * osjsn@niklasfamily.de
 *
 * This file is part of the NetFPGA 10G UPB OpenFlow Switch project.
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

//`define use_chipscope

`default_nettype none

module main(

	input	wire				clk100,
	
	output logic [2:0]	led,
	input wire [1:0]		pb_n,
	
	output logic			qdr_a_k,				// K
	output logic			qdr_a_k_n,			// K
	output logic			qdr_a_c,				// C
	output logic			qdr_a_c_n,			// Cn
	input	wire				qdr_a_cq,			// CQ
	input	wire				qdr_a_cq_n,			// CQn
	output logic [18:0]	qdr_a_sa,			// A[18:0]
	output logic			qdr_a_r_n,			// RPS
	input wire [35:0]		qdr_a_q,				// Q[35:0]
	output logic			qdr_a_w_n,			// WPS
	output logic [3:0]	qdr_a_bw_n,			// BWS[3:0]
	output logic [35:0]	qdr_a_d,				// D[35:0]
	output logic			qdr_a_dll_off_n,	// DOFFn
	
	output logic			qdr_b_k,				// K
	output logic			qdr_b_k_n,			// K
	output logic			qdr_b_c,				// C
	output logic			qdr_b_c_n,			// Cn
	input	wire				qdr_b_cq,			// CQ
	input wire				qdr_b_cq_n,			// CQn
	output logic [18:0]	qdr_b_sa,			// A[18:0]
	output logic			qdr_b_r_n,			// RPS
	input wire [35:0]		qdr_b_q,				// Q[35:0]
	output logic			qdr_b_w_n,			// WPS
	output logic [3:0]	qdr_b_bw_n,			// BWS[3:0]
	output logic [35:0]	qdr_b_d,				// D[35:0]
	output logic			qdr_b_dll_off_n,	// DOFFn
	
	output logic			qdr_c_k,				// K
	output logic			qdr_c_k_n,			// K
	output logic			qdr_c_c,				// C
	output logic			qdr_c_c_n,			// Cn
	input wire				qdr_c_cq,			// CQ
	input wire				qdr_c_cq_n,			// CQn
	output logic [18:0]	qdr_c_sa,			// A[18:0]
	output logic			qdr_c_r_n,			// RPS
	input wire [35:0]		qdr_c_q,				// Q[35:0]
	output logic			qdr_c_w_n,			// WPS
	output logic [3:0]	qdr_c_bw_n,			// BWS[3:0]
	output logic [35:0]	qdr_c_d,				// D[35:0]
	output logic			qdr_c_dll_off_n,	// DOFFn
	
	output logic [2:0]	dci_masterbank_dummy_pin = 0
	
);

/*
*		Power-On-Reset and reset button
*/
logic system_reset_clk100 = 1, reset_button_sync = 1, reset_chipscope_sync = 0;
logic [15:0] reset_debounce = 0; // ~21ms debounce/por time
logic reset_chipscope = '0;

always @ ( posedge clk100 ) begin

	if (reset_debounce != 2**$size(reset_debounce)-1) begin
		system_reset_clk100 <= 1;
		reset_debounce <= reset_debounce + 1;
	end else begin
		system_reset_clk100 <= 0;
	end
	
	reset_button_sync <= ~pb_n[0];
	reset_chipscope_sync <= reset_chipscope;
	
	if (reset_button_sync || reset_chipscope_sync) begin
		reset_debounce <= 0;
	end

end

/*
*		DCI
*/
wire dci_locked;

DCIRESET DCIRESET_inst (
	.LOCKED(dci_locked),			// 1-bit DCI LOCKED Output
	.RST(system_reset_clk100)	// 1-bit DCI Reset Input
);

/*
*		Clocking
*/
wire clk150_bufg, clk150, clk200_bufg, clk200, clk300_bufg, clk300, clk300_90_bufg, clk300_90;
wire clk_fb;
wire pll_locked;

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
	.RST(system_reset_clk100) // Asynchronous PLL reset
);

BUFG bufg_clk150 (
	.I(clk150_bufg),
	.O(clk150)
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

/*
*		IODELAYCTRL...
*/

localparam IODELAY_GRP_QDR2 = "IODELAY_GRP_QDR2";

logic idelayctrl_reset = 1, pll_locked_clk100 = 0;

always @ ( posedge clk100 ) begin

	pll_locked_clk100 <= pll_locked;
	idelayctrl_reset <= system_reset_clk100 | ~pll_locked_clk100;

end

wire iodelayctrl_rdy;

(* IODELAY_GROUP = IODELAY_GRP_QDR2 *) // Specifies group name for associated IODELAYs and IDELAYCTRL
IDELAYCTRL IDELAYCTRL_inst (
	.RDY(iodelayctrl_rdy),		// 1-bit ready output
	.REFCLK(clk200),				// 1-bit reference clock input
	.RST(idelayctrl_reset)		// 1-bit reset input
);

/*
*		Global reset (for clk150)
*/
logic reset_clk150 = 1;
logic dci_locked_clk150 = 0, pll_locked_clk150 = 0, iodelayctrl_rdy_clk150 = 0, system_reset_clk150 = 1;
logic reset_clk150_chipscope = '0;

always @ ( posedge clk150 ) begin

	dci_locked_clk150 <= dci_locked;
	pll_locked_clk150 <= pll_locked;
	iodelayctrl_rdy_clk150 <= iodelayctrl_rdy;
	system_reset_clk150 <= system_reset_clk100;
	
	reset_clk150 <= ~dci_locked_clk150 | ~pll_locked_clk150 | ~iodelayctrl_rdy_clk150 | system_reset_clk150 | reset_clk150_chipscope;
	
end

/*
*		Connect qdr2 module
*/
qdr2_sram_if qdr_a();

assign qdr_a_k = qdr_a.k;
assign qdr_a_k_n = qdr_a.k_n;
assign qdr_a_c = qdr_a.c;
assign qdr_a_c_n = qdr_a.c_n;
assign qdr_a.cq = qdr_a_cq;
assign qdr_a.cq_n = qdr_a_cq_n;
assign qdr_a_sa = qdr_a.sa;
assign qdr_a_r_n = qdr_a.r_n;
assign qdr_a.q = qdr_a_q;
assign qdr_a_w_n = qdr_a.w_n;
assign qdr_a_bw_n = qdr_a.bw_n;
assign qdr_a_d = qdr_a.d;
assign qdr_a_dll_off_n = qdr_a.dll_off_n;

wire [35:0] chipscope_qdr2_sram_a;
wire ready_qdr2_sram_a;

qdr2_sram #(

	.IODELAY_GRP(IODELAY_GRP_QDR2)
	
) qdr2_sram_a (

	.clk				(clk150),
	.clk2x			(clk300),
	.clk2x90			(clk300_90),
	.reset			(reset_clk150),
	
	.qdr				(qdr_a),
	
	.ready			(ready_qdr2_sram_a),
	
	.chipscope		(chipscope_qdr2_sram_a)

);

qdr2_sram_if qdr_b();

assign qdr_b_k = qdr_b.k;
assign qdr_b_k_n = qdr_b.k_n;
assign qdr_b_c = qdr_b.c;
assign qdr_b_c_n = qdr_b.c_n;
assign qdr_b.cq = qdr_b_cq;
assign qdr_b.cq_n = qdr_b_cq_n;
assign qdr_b_sa = qdr_b.sa;
assign qdr_b_r_n = qdr_b.r_n;
assign qdr_b.q = qdr_b_q;
assign qdr_b_w_n = qdr_b.w_n;
assign qdr_b_bw_n = qdr_b.bw_n;
assign qdr_b_d = qdr_b.d;
assign qdr_b_dll_off_n = qdr_b.dll_off_n;

wire [35:0] chipscope_qdr2_sram_b;
wire ready_qdr2_sram_b;

qdr2_sram #(

	.IODELAY_GRP(IODELAY_GRP_QDR2)
	
) qdr2_sram_b (

	.clk				(clk150),
	.clk2x			(clk300),
	.clk2x90			(clk300_90),
	.reset			(reset_clk150),
	
	.qdr				(qdr_b),
	
	.ready			(ready_qdr2_sram_b),
	
	.chipscope		(chipscope_qdr2_sram_b)

);

qdr2_sram_if qdr_c();

assign qdr_c_k = qdr_c.k;
assign qdr_c_k_n = qdr_c.k_n;
assign qdr_c_c = qdr_c.c;
assign qdr_c_c_n = qdr_c.c_n;
assign qdr_c.cq = qdr_c_cq;
assign qdr_c.cq_n = qdr_c_cq_n;
assign qdr_c_sa = qdr_c.sa;
assign qdr_c_r_n = qdr_c.r_n;
assign qdr_c.q = qdr_c_q;
assign qdr_c_w_n = qdr_c.w_n;
assign qdr_c_bw_n = qdr_c.bw_n;
assign qdr_c_d = qdr_c.d;
assign qdr_c_dll_off_n = qdr_c.dll_off_n;

wire [35:0] chipscope_qdr2_sram_c;
wire ready_qdr2_sram_c;

qdr2_sram #(

	.IODELAY_GRP(IODELAY_GRP_QDR2)
	
) qdr2_sram_c (

	.clk				(clk150),
	.clk2x			(clk300),
	.clk2x90			(clk300_90),
	.reset			(reset_clk150),
	
	.qdr				(qdr_c),
	
	.ready			(ready_qdr2_sram_c),
	
	.chipscope		(chipscope_qdr2_sram_c)

);

`ifdef use_chipscope

wire [35:0] chipscope_vio;

icon icon0 
(
	.CONTROL0(chipscope_vio),
	.CONTROL1(chipscope_qdr2_sram_a),
	.CONTROL2(chipscope_qdr2_sram_b),
	.CONTROL3(chipscope_qdr2_sram_c)
);

vio_main vio
(
	.CONTROL(chipscope_vio),
	.ASYNC_IN( {
		ready_qdr2_sram_c,
		ready_qdr2_sram_b,
		ready_qdr2_sram_a,
		reset_clk150,
		iodelayctrl_rdy,
		pll_locked,
		dci_locked,
		system_reset_clk100,
		reset_button_sync
	} ),
	.ASYNC_OUT( {
		reset_chipscope,
		reset_clk150_chipscope
	} )
);

`endif

endmodule
