/*
 * UPB Clock Generator core testbench
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

`timescale 1ns / 1ps
`default_nettype none

module nf10_upb_clock_generator_tb;

	// 100 MHz input clock
	logic clk100_in = '0;
	always #5 clk100_in = ~clk100_in;
	
	logic async_reset_in_n = '1;
	
	wire clk_out, clk2x_out, clk2x90_out, clk100_out, reset_out, reset_n_out;


	nf10_upb_clock_generator #(
		
		.use_dci(1),
		.wait_for_dci_locked(1),
		.use_iodelay_control(1)
	
	) clockgen_0 (
		.*
	);
	
	initial begin
	
		
		#100000000; // wait 100 ms
		
		async_reset_in_n <= '0;
		
		#100; // wait 100ns
		
		async_reset_in_n <= '1;
		
		#100000000; // wait 100 ms
	
	end

endmodule
