/*
 * UPB Clock Generator core
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

`default_nettype none

module nf10_upb_clock_generator #(

	parameter reduce_clk_to_120mhz = 0,
	parameter use_dci = 1,
	parameter wait_for_dci_locked = 1,
	parameter use_iodelay_control = 1
	
) (
	
	input wire clk100_in,
	input wire async_reset_in_n,
	
	output wire clk_out,
	output wire clk2x_out,
	output wire clk2x90_out,
	output wire clk100_out,
	output wire clk20_out,
	
	output wire reset_out,
	output wire reset_n_out

);

wire clk_internal;
assign clk_out = clk_internal;

wire clk2x_internal;
assign clk2x_out = clk2x_internal;

wire clk2x90_internal;
assign clk2x90_out = clk2x90_internal;

wire clk100_internal;
assign clk100_out = clk100_internal;

wire clk200_internal;

reg reset_internal = 1'b1;
assign reset_out = reset_internal;

reg reset_n_internal = 1'b0;
assign reset_n_out = reset_n_internal;

/*
*		Power-On-Reset and reset button
*/
reg system_reset_clk100_in = 1'b1, reset_button_sync = 1'b1;

localparam debounce_counter_width = 23;
reg [debounce_counter_width-1:0] reset_debounce = 0; // ~84ms debounce/por time

always @ ( posedge clk100_in ) begin

	if (reset_debounce != 2**debounce_counter_width-1) begin
		system_reset_clk100_in <= 1;
		reset_debounce <= reset_debounce + 1;
	end else begin
		system_reset_clk100_in <= 0;
	end
	
	reset_button_sync <= ~async_reset_in_n;
	
	if (reset_button_sync) begin
		reset_debounce <= 0;
	end

end

/*
*		DCI
*/
wire dci_locked;

generate

	if (use_dci) begin
	
		wire dci_locked_internal;
	
		DCIRESET DCIRESET_inst (
			.LOCKED(dci_locked_internal),			// 1-bit DCI LOCKED Output
			.RST(system_reset_clk100_in)	// 1-bit DCI Reset Input
		);
		
		if (wait_for_dci_locked)
			assign dci_locked = dci_locked_internal;
		else
			assign dci_locked = 1'b1;
		
	end else begin
	
		assign dci_locked = 1'b1;
	
	end

endgenerate

/*
*		Clocking
*/
wire clk_bufg, clk2x_bufg, clk2x90_bufg, clk200_bufg, clk100_bufg, slowest_clk_for_reset_bufg, slowest_clk_for_reset;
wire clk_fb;
wire pll_locked;

PLL_BASE #(

	.CLKIN_PERIOD(10.0),		// input clock is 100 MHz
	.CLKFBOUT_MULT(12),		// clock gets multiplied by 12 => 1200 MHz
	
	.CLKOUT0_DIVIDE(reduce_clk_to_120mhz ? 10 : 8),	// /8 = 150 MHz, /10 = 120 MHz
	.CLKOUT1_DIVIDE(reduce_clk_to_120mhz ? 5 : 4),	// /4 = 300 MHz, /5 = 240 MHz
	.CLKOUT2_DIVIDE(reduce_clk_to_120mhz ? 5 : 4),	// /4 = 300 MHz, /5 = 240 MHz
	.CLKOUT2_PHASE(90.0),	// 90° phase shifted
	.CLKOUT3_DIVIDE(6),		// /6 = 200 MHz
	.CLKOUT4_DIVIDE(12),		// /12 = 100 MHz
	.CLKOUT5_DIVIDE(60)		//	/60 = 20 MHz

) main_pll (
	.CLKFBOUT(clk_fb),
	.CLKOUT0(clk_bufg),		// 150 MHz
	.CLKOUT1(clk2x_bufg),	// 300 MHz
	.CLKOUT2(clk2x90_bufg),	// 300 MHz, 90° phase shifted
	.CLKOUT3(clk200_bufg),	// 200 MHz
	.CLKOUT4(clk100_bufg),	// 100 MHz
	.CLKOUT5(slowest_clk_for_reset_bufg), // 20 MHz
	.LOCKED(pll_locked),
	.CLKFBIN(clk_fb),
	.CLKIN(clk100_in),
	.RST(system_reset_clk100_in)
);

assign clk20_out = slowest_clk_for_reset;

BUFG bufg_clk (
	.I(clk_bufg),
	.O(clk_internal)
);

BUFG bufg_clk2x (
	.I(clk2x_bufg),
	.O(clk2x_internal)
);

BUFG bufg_clk2x90 (
	.I(clk2x90_bufg),
	.O(clk2x90_internal)
);

BUFG bufg_clk200 (
	.I(clk200_bufg),
	.O(clk200_internal)
);

BUFG bufg_clk100 (
	.I(clk100_bufg),
	.O(clk100_internal)
);

BUFG bufg_slowest_clk_for_reset (
	.I(slowest_clk_for_reset_bufg),
	.O(slowest_clk_for_reset)
);

/*
*		IODELAYCTRL...
*/

localparam IODELAY_GRP_QDR2 = "IODELAY_GRP_QDR2";

reg idelayctrl_reset_clk100_in = 1'b1, dci_locked_clk100_in = 1'b0, pll_locked_clk100_in = 1'b0;

always @ ( posedge clk100_in ) begin

	dci_locked_clk100_in <= dci_locked;
	pll_locked_clk100_in <= pll_locked;
	idelayctrl_reset_clk100_in <= system_reset_clk100_in | ~dci_locked_clk100_in | ~pll_locked_clk100_in;

end

wire iodelayctrl_rdy;

generate

	if (use_iodelay_control) begin

		(* IODELAY_GROUP = IODELAY_GRP_QDR2 *) // Specifies group name for associated IODELAYs and IDELAYCTRL
		IDELAYCTRL IDELAYCTRL_inst (
			.RDY(iodelayctrl_rdy),		// 1-bit ready output
			.REFCLK(clk200_internal),	// 1-bit reference clock input
			.RST(idelayctrl_reset_clk100_in)		// 1-bit reset input
		);

	end else begin
	
		assign iodelayctrl_rdy = 1'b1;
	
	end

endgenerate

// Asynchnous reset logic: generates reset_clk100_in which ist at least 2048 clk100_in cycles active

reg iodelayctrl_rdy_clk100_in = 1'b0;
reg reset_pulse_clk100_in = 1'b1, reset_clk100_in = 1'b1;
reg [10:0] reset_clk100_in_active_counter = 0;

always @ ( posedge clk100_in ) begin

	iodelayctrl_rdy_clk100_in <= iodelayctrl_rdy;
	reset_pulse_clk100_in <= system_reset_clk100_in | ~dci_locked_clk100_in | ~pll_locked_clk100_in | ~iodelayctrl_rdy_clk100_in;
	
	reset_clk100_in <= 1'b1;
	
	if (reset_pulse_clk100_in) begin
		reset_clk100_in_active_counter <= 0;
	end else begin
		if (reset_clk100_in_active_counter != 11'b11111111111) begin
			reset_clk100_in_active_counter <= reset_clk100_in_active_counter + 1;
		end else begin
			reset_clk100_in <= 1'b0;
		end
	end

end

// ensure, that the reset is at least 2048 cycles on

reg [10:0] reset_active_counter = 0;
reg all_ready_slowest_clk_for_reset_sync = 1'b0;

always @ ( posedge slowest_clk_for_reset, posedge reset_clk100_in ) begin

	if (reset_clk100_in) begin
	
		reset_internal <= 1'b1;
		reset_n_internal <= 1'b0;
		reset_active_counter <= 0;
	
	end else begin
		
		reset_internal <= 1'b1;
		reset_n_internal <= 1'b0;
		
		if (reset_active_counter != 11'b11111111111) begin
			reset_active_counter <= reset_active_counter + 1;
		end else begin
			reset_internal <= 1'b0;
			reset_n_internal <= 1'b1;
		end
	
	end

end

endmodule
