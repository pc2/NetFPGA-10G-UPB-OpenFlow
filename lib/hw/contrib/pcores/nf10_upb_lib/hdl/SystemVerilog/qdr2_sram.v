/*
 * UPB QDR2-SRAM controller core
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

//`define use_chipscope_qdr2_sram

`default_nettype none

// QDR-II SRAM:
// CY7C1515JV18-300BZXC

interface qdr2_sram_if #(

	parameter mem_data_width			= 36,
	parameter mem_addr_width			= 19
	
);

	logic									k;						// K
	logic									k_n;					// K
	logic									c;						// C
	logic									c_n;					// Cn
	logic									cq;					// CQ
	logic									cq_n;					// CQn
	logic [mem_addr_width-1:0]		sa;					// A[18:0]
	logic									r_n;					// RPS
	logic [mem_data_width-1:0]		q;						// Q[35:0]
	logic									w_n;					// WPS
	logic [mem_data_width/8-1:0]	bw_n;					// BWS[3:0] (no byte write support)
	logic [mem_data_width-1:0]		d;						// D[35:0]
	logic									dll_off_n;			// DOFFn (DLL is always used)
	
	modport master (
		
		output k, k_n, c, c_n, sa, r_n, w_n, bw_n, d, dll_off_n,
		input cq, cq_n, q
	
	);
	
	modport slave (
		
		input k, k_n, c, c_n, sa, r_n, w_n, bw_n, d, dll_off_n,
		output cq, cq_n, q
	
	);

endinterface

module qdr2_sram #(

	parameter mem_data_width			= 36,
	parameter mem_addr_width			= 19,
	parameter IODELAY_GRP				= "IODELAY_GRP_QDR2",
	parameter read_delay_cycles		= 5,								// 5 cycles delay from r_int to r_valid(_int)
	parameter qdr2_pll_lock_cycles	= 1024,							// number of sram clock cycles the qdr2 sram's pll needs to lock
	parameter simulation_speed_up		= 0

) (

	input	wire									clk,
	input	wire									clk2x,
	input	wire									clk2x90,
	input	wire									reset,
	
	qdr2_sram_if.master						qdr,
	
	output logic								ready,
	output logic								error,
	
	input wire									rd,
	input wire [mem_addr_width-1:0]		r_addr,
	output logic								r_valid,
	output logic [mem_data_width*4-1:0]	r_data,
	
	input wire									wr,
	input wire [mem_addr_width-1:0]		w_addr,
	input wire [mem_data_width*4-1:0]	w_data,
	
	inout wire [35:0]							chipscope
	
);

/*
*		Clock output k, k_n lines
*/

assign qdr.c = '1;
assign qdr.c_n = '1;
assign qdr.bw_n = '0;
assign qdr.dll_off_n = '1;


OSERDES oser_k (

	.OQ(qdr.k),

	.CLK(clk2x90),
	.CLKDIV(clk),
	.SR(reset),
	
	.D1('1), .D2('0), .D3('1), .D4('0),
	.D5('0), .D6('0),
	
	.T1('0), .T2('0), .T3('0), .T4('0),
	
	.REV('0),
	.SHIFTIN1('0), .SHIFTIN2('0),

	.OCE('1), .TCE('1)
);

OSERDES oser_k_n (

	.OQ(qdr.k_n),

	.CLK(clk2x90),
	.CLKDIV(clk),
	.SR(reset),
	
	.D1('0), .D2('1), .D3('0), .D4('1),
	.D5('0), .D6('0),
	
	.T1('0), .T2('0), .T3('0), .T4('0),
	
	.REV('0),
	.SHIFTIN1('0), .SHIFTIN2('0),

	.OCE('1), .TCE('1)
);


/*
*		Data output qdr.sa, r_n, w_n, qdr.d
*/
localparam qdr2_out_data_width = mem_addr_width + 1 + 1 + mem_data_width;
wire [qdr2_out_data_width-1:0] output_to_qdr2;
assign {qdr.sa, qdr.r_n, qdr.w_n, qdr.d} = output_to_qdr2;
wire [3:0][qdr2_out_data_width-1:0] qdr2_oserdes;

generate

	for (genvar i = 0; i < qdr2_out_data_width; i++) begin
	
		OSERDES oser_outputs (

			.OQ(output_to_qdr2[i]),

			.CLK(clk2x),
			.CLKDIV(clk),
			.SR(reset),

			.D1(qdr2_oserdes[3][i]),
			.D2(qdr2_oserdes[2][i]),
			.D3(qdr2_oserdes[1][i]),
			.D4(qdr2_oserdes[0][i]),
			.D5('0), .D6('0),
			
			.T1('0), .T2('0), .T3('0), .T4('0),
			
			.REV('0),
			.SHIFTIN1('0), .SHIFTIN2('0),

			.OCE('1), .TCE('1)
		);
	
	end

endgenerate

/*
*		cq_n clock (negative clock to capture "q" data input)
*/

logic cq_dlyce = 0, cq_dlyinc = 0, cq_dlrst = 0;
wire cq_n_delayed;

(* IODELAY_GROUP = IODELAY_GRP *)
IODELAY # (
	.DELAY_SRC("I"),  // Specify which input port to be used, "I"=IDATAIN,
							//  "O"=ODATAIN, "DATAIN"=DATAIN, "IO"=Bi-directional
	.HIGH_PERFORMANCE_MODE("TRUE"), // "TRUE" specifies lower jitter
											  //   at expense of more power 
	.IDELAY_TYPE("VARIABLE"),  // "FIXED" or "VARIABLE" 
	.IDELAY_VALUE(0),         // 0 to 63 tap values
	.ODELAY_VALUE(0),         // 0 to 63 tap values

	.SIGNAL_PATTERN("CLOCK")  // Input signal type, "CLOCK" or "DATA" 
	
) iodelay_cq_n (

	.IDATAIN(qdr.cq_n),
	.DATAOUT(cq_n_delayed),
	
	.C(clk),
	
	.CE(cq_dlyce),
	.INC(cq_dlyinc),
	.RST(cq_dlrst),
	
	.T(1'b1)
);

wire q_iserdes_clk_n;

BUFIO bufio_cq_n (
	.I(cq_n_delayed),
	.O(q_iserdes_clk_n)
);

/*
*		Data input q
*/

logic [3:0][mem_data_width-1:0] r_data_int;
logic q_dlyce = 0, q_dlyinc = 0, q_dlrst = 0;

generate
	for (genvar i = 0; i < mem_data_width; i++) begin
	
		(* IODELAY_GROUP = IODELAY_GRP *)
		ISERDES #(
			
			.IOBDELAY("IFD"),
			.IOBDELAY_TYPE("VARIABLE"),
			.IOBDELAY_VALUE(0)

		)ISERDES_inst (
		
			.CLK(q_iserdes_clk_n), // phasis shifted by 180°
			.OCLK(clk2x90),
			.CLKDIV(clk),
			.SR(reset),
			
			.D(qdr.q[i]),
			.CE1('1), .CE2('1),
		
			.Q1(r_data_int[3][i]),
			.Q2(r_data_int[2][i]),
			.Q3(r_data_int[1][i]),
			.Q4(r_data_int[0][i]),
			
			.DLYCE(q_dlyce),
			.DLYINC(q_dlyinc),
			.DLYRST(q_dlrst),
			
			.REV('0),
			.SHIFTIN1('0), .SHIFTIN2('0)
		);
	
	end
endgenerate

assign r_data = r_data_int;

/*
*		remaining read/write logic
*/
logic rd_int = '0, rd_int_delayed = '0, wr_int = '0;
logic [mem_addr_width-1:0] r_addr_int, r_addr_int_delayed, w_addr_int;
logic [3:0][mem_data_width-1:0] w_data_int;
logic [3:2][mem_data_width-1:0] w_data_int_delayed;

always_ff @(posedge clk) begin
	r_addr_int_delayed <= r_addr_int;
	rd_int_delayed <= rd_int;
	w_data_int_delayed[3:2] <= w_data_int[3:2];
end

assign qdr2_oserdes = { // qdr.sa, qdr.r_n, qdr.w_n, qdr.d
	w_addr_int, 1'b1, ~wr_int, w_data_int_delayed[2],
	w_addr_int, 1'b1, ~wr_int, w_data_int_delayed[3],
	r_addr_int, ~rd_int, 1'b1, w_data_int[0],
	r_addr_int, ~rd_int, 1'b1, w_data_int[1]
};

logic r_valid_delay[read_delay_cycles] = '{ default: 0 };

always_ff @(posedge clk) begin

	r_valid_delay[$high(r_valid_delay)] <= rd_int;
	
	for (integer i = 0; i < $bits(r_valid_delay)-1; i++)
		r_valid_delay[i] <= r_valid_delay[i+1];
		
end

wire r_valid_int = r_valid_delay[0];

/*
*		reset and qdr2-sram initialization FSM
*/

// memory test signals
wire memtest_ready;
enum { MEMTEST_NO_TEST, MEMTEST_OUTPUT_DDR_TEST, MEMTEST_EXTENSIVE_TEST, MEMTEST_COMPLETE_TEST } memtest_run, selected_memtest;
logic memtest_last_result_ok;

typedef enum { 
	STATE_WAIT_RAM_PLL, 
	STATE_DDR_TIMING_INC_CQ_DELAY,
	STATE_DDR_TIMING_INC_Q_DELAY,
	STATE_DDR_TIMING_ANALYZE_RESULT,
	STATE_DDR_TIMING_SET_CQ_DELAY,
	STATE_DDR_TIMING_SET_Q_DELAY,
	STATE_OCLK_TIMING_INC_CQ_Q_DELAY,
	STATE_OCLK_TIMING_UNDO_INC_CQ_Q_DELAY,
	STATE_OCLK_TIMING_DEC_Q_DEC_QC_DELAY,
	STATE_OCLK_TIMING_UNDO_DEC_Q_DEC_CQ_DELAY,
	STATE_OCLK_TIMING_ANALYZE_RESULT,
	STATE_OCLK_TIMING_SET_INC_CQ_Q_DELAY,
	STATE_OCLK_TIMING_SET_DEC_CQ_Q_DELAY,
	STATE_MEMTEST_COMPLETE_TEST,
	

	STATE_READY,
	STATE_MEMORY_ERROR
} init_states;

init_states init_state;

assign error = init_state == STATE_MEMORY_ERROR ? 1'b1 : 1'b0;

logic [$clog2(qdr2_pll_lock_cycles)-1:0] pll_lock_counter = '0;

logic [5:0] 
	iodelay_counter,
	cq_valid_begin, cq_valid_end,
	q_valid_begin, q_valid_end,
	inc_cq_q_valid_begin, inc_cq_q_valid_end, dec_cq_q_valid_begin, dec_cq_q_valid_end,
	temp_cq_delay, temp_q_delay, temp_inc_cq_q_delay, temp_dec_cq_q_delay
;

logic cq_found_valid_begin, q_found_valid_begin, inc_cq_q_found_valid_begin, dec_cq_q_found_valid_begin;
logic timing_test_done;

always_ff @(posedge clk) begin

	memtest_run <= MEMTEST_NO_TEST;
	
	cq_dlrst <= '0;
	cq_dlyce <= '0;
	cq_dlyinc <= '0;
	
	q_dlrst <= '0;
	q_dlyce <= '0;
	q_dlyinc <= '0;
	

	case (init_state)
	
		STATE_WAIT_RAM_PLL: begin
		
			// reset IODELAYs
			cq_dlrst <= 1;
			q_dlrst <= 1;
		
			if (pll_lock_counter < qdr2_pll_lock_cycles-1)
			
				pll_lock_counter <= pll_lock_counter + 1;
				
			else begin
			
				if (memtest_ready) begin
				
					iodelay_counter <= '0;
					cq_found_valid_begin <= '0;
					q_found_valid_begin <= '0;
					
					memtest_run <= MEMTEST_OUTPUT_DDR_TEST;
					
					init_state <= STATE_DDR_TIMING_INC_CQ_DELAY;
				end
			end
		end
		
		STATE_DDR_TIMING_INC_CQ_DELAY: begin
		
			if (memtest_ready) begin
					
				memtest_run <= MEMTEST_OUTPUT_DDR_TEST; // there is always a test following
				
				cq_dlyce <= '1;
				cq_dlyinc <= '1;
				
				timing_test_done = '0;
				
				if (iodelay_counter != '1) begin
				
					iodelay_counter <= iodelay_counter + 1;
					
				end else begin
					// last tap reached
					timing_test_done = '1;
				end
				
				if (memtest_last_result_ok) begin
					
					if (!cq_found_valid_begin) begin
					
						cq_found_valid_begin <= '1;
						
						cq_valid_begin <= iodelay_counter;
						cq_valid_end <= iodelay_counter;
						
						if (iodelay_counter == '0) begin // timing already works with no delay at q and cq
							q_found_valid_begin <= '1;
							q_valid_begin <= '0;
							q_valid_end <= '0;
						end
					
					end else begin
						cq_valid_end <= iodelay_counter;
					end
					
				end else begin
					
					if (cq_found_valid_begin) begin
					
						// found the other edge
						timing_test_done = '1;
					end
					
				end
				
				if (timing_test_done) begin
					
					cq_dlrst <= '1; // reset cq delay
					
					// increment q delay (so we start at 1)
					q_dlyce <= '1;
					q_dlyinc <= '1;
					iodelay_counter <= 1; // (start at 1)
					
					init_state <= STATE_DDR_TIMING_INC_Q_DELAY;
					
				end
			end
		end
		
		STATE_DDR_TIMING_INC_Q_DELAY: begin
		
			if (memtest_ready) begin
				
				q_dlyce <= '1;
				q_dlyinc <= '1;
				
				timing_test_done = '0;
				
				if (iodelay_counter != '1) begin
				
					iodelay_counter <= iodelay_counter + 1;
					
				end else begin
					// last tap reached
					timing_test_done = '1;
				end
				
				if (memtest_last_result_ok) begin
					
					if (!q_found_valid_begin) begin
					
						q_found_valid_begin <= '1;
						
						q_valid_begin <= iodelay_counter;
						q_valid_end <= iodelay_counter;
					
					end else begin
					
						q_valid_end <= iodelay_counter;
					end
					
				end else begin
					
					if (q_found_valid_begin) begin
					
						// found the other edge
						timing_test_done = '1;
					end
					
				end
				
				if (timing_test_done) begin
					
					q_dlrst <= '1; // reset q delay
					
					init_state <= STATE_DDR_TIMING_ANALYZE_RESULT;
					
				end else begin
				
					memtest_run <= MEMTEST_OUTPUT_DDR_TEST;
					
				end
			end
		end
		
		STATE_DDR_TIMING_ANALYZE_RESULT: begin
		
			iodelay_counter <= '0;
			inc_cq_q_found_valid_begin <= '0;
			dec_cq_q_found_valid_begin <= '0;
		
			if (cq_found_valid_begin && q_found_valid_begin) begin
			
				if (cq_valid_begin == '0 && q_valid_begin == '0) begin // 0 delay for cq and q is already a valid timing
				
					if (cq_valid_end > q_valid_end) begin
					
						temp_cq_delay <= (cq_valid_end - q_valid_end) / 2;
						temp_q_delay <= '0;
					
						init_state <= STATE_DDR_TIMING_SET_CQ_DELAY;
					end else begin
					
						temp_cq_delay <= '0;
						temp_q_delay <= (q_valid_end - cq_valid_end) / 2;
						
						init_state <= STATE_DDR_TIMING_SET_Q_DELAY;
					end
				
				end else begin
				
					if (cq_valid_begin < q_valid_begin) begin
					
						// set delay of cq
						temp_cq_delay <= (cq_valid_end + cq_valid_begin) / 2;
						temp_q_delay <= '0;
					
						init_state <= STATE_DDR_TIMING_SET_CQ_DELAY;
						
					end else begin
					
						// set delay of q
						temp_cq_delay <= '0;
						temp_q_delay <= (q_valid_end + q_valid_begin) / 2;
						
					
						init_state <= STATE_DDR_TIMING_SET_Q_DELAY;
					
					end
				
				end
				
			end else begin
			
				if (cq_found_valid_begin) begin
				
					// valid timing only while setting cq delay
					// set delay of cq
					temp_cq_delay <= (cq_valid_end + cq_valid_begin) / 2;
					temp_q_delay <= '0;
					
					init_state <= STATE_DDR_TIMING_SET_CQ_DELAY;
					
				end else begin
				
					if (q_found_valid_begin) begin
				
						// valid timing only while setting cq delay
						// set delay of q
						temp_cq_delay <= '0;
						temp_q_delay <= (q_valid_end + q_valid_begin) / 2;
						
					
						init_state <= STATE_DDR_TIMING_SET_Q_DELAY;
						
					end else begin
					
						// no valid timing at all
						init_state <= STATE_MEMORY_ERROR;
					
					end
				
				end
			
			end
		end
		
		STATE_DDR_TIMING_SET_CQ_DELAY: begin
			
			if (iodelay_counter != temp_cq_delay) begin
			
				cq_dlyce <= '1;
				cq_dlyinc <= '1;
				iodelay_counter <= iodelay_counter + 1;
				
			end else begin
			
				if (memtest_ready) begin
				
					iodelay_counter <= '0;
					memtest_run <= MEMTEST_EXTENSIVE_TEST;
					
					init_state <= STATE_OCLK_TIMING_INC_CQ_Q_DELAY;
					
				end
			end
		end
		
		STATE_DDR_TIMING_SET_Q_DELAY: begin
		
			if (iodelay_counter != temp_q_delay) begin
			
				q_dlyce <= '1;
				q_dlyinc <= '1;
				iodelay_counter <= iodelay_counter + 1;
				
			end else begin
			
				if (memtest_ready) begin
			
					iodelay_counter <= '0;
					memtest_run <= MEMTEST_EXTENSIVE_TEST;
					
					init_state <= STATE_OCLK_TIMING_INC_CQ_Q_DELAY;
				
				end
			end
			
		end
		
		STATE_OCLK_TIMING_INC_CQ_Q_DELAY: begin
		
			if (memtest_ready) begin
			
				timing_test_done = '0;
			
				if (temp_cq_delay == '1 || temp_q_delay == '1) begin
				
					// reached the limit of one iodelay
					timing_test_done = '1;
				end;
				
				if (memtest_last_result_ok) begin
						
					if (!inc_cq_q_found_valid_begin) begin
					
						inc_cq_q_found_valid_begin <= '1;
						
						inc_cq_q_valid_begin <= iodelay_counter;
						inc_cq_q_valid_end <= iodelay_counter;
						
						if (iodelay_counter == '0) begin // timing already works without incrementing and decrementing cq and q
							dec_cq_q_found_valid_begin <= '1;
							dec_cq_q_valid_begin <= '0;
							dec_cq_q_valid_end <= '0;
						end
					
					end else begin
						inc_cq_q_valid_end <= iodelay_counter;
					end
					
				end else begin
					
					if (inc_cq_q_found_valid_begin) begin
					
						// found the other edge
						timing_test_done = '1;
					end
					
				end
				
				if (!timing_test_done) begin
				
					memtest_run <= MEMTEST_EXTENSIVE_TEST;
				
					// increment cq delay
					cq_dlyce <= '1;
					cq_dlyinc <= '1;
					temp_cq_delay <= temp_cq_delay + 1;
					
					// increment q delay
					q_dlyce <= '1;
					q_dlyinc <= '1;
					temp_q_delay <= temp_q_delay + 1;
					
					iodelay_counter <= iodelay_counter + 1;
					
				end else begin
				
					init_state <= STATE_OCLK_TIMING_UNDO_INC_CQ_Q_DELAY;
				end
			
			end
		end
		
		STATE_OCLK_TIMING_UNDO_INC_CQ_Q_DELAY: begin
		
			if (iodelay_counter != '0) begin
			
				// decrement cq delay
				cq_dlyce <= '1;
				temp_cq_delay <= temp_cq_delay - 1;
				
				// decrement q delay
				q_dlyce <= '1;
				temp_q_delay <= temp_q_delay - 1;
				
				iodelay_counter <= iodelay_counter - 1;
				
			end else begin
			
				if (memtest_ready) begin
				
					memtest_run <= MEMTEST_EXTENSIVE_TEST;
					init_state <= STATE_OCLK_TIMING_DEC_Q_DEC_QC_DELAY;
				end
			end
		end
		
		STATE_OCLK_TIMING_DEC_Q_DEC_QC_DELAY: begin
		
			if (memtest_ready) begin
		
				timing_test_done = '0;
			
				if (temp_q_delay == '0 || temp_cq_delay == '0) begin
				
					// reched the limit of one iodelay
					timing_test_done = '1;
				end;
				
				if (memtest_last_result_ok) begin
						
					if (!dec_cq_q_found_valid_begin) begin
					
						dec_cq_q_found_valid_begin <= '1;
						
						dec_cq_q_valid_begin <= iodelay_counter;
						dec_cq_q_valid_end <= iodelay_counter;
						
					end else begin
					
						dec_cq_q_valid_end <= iodelay_counter;
					end
					
				end else begin
					
					if (dec_cq_q_found_valid_begin) begin
					
						// found the other edge
						timing_test_done = '1;
					end
					
				end
				
				if (!timing_test_done) begin
				
					memtest_run <= MEMTEST_EXTENSIVE_TEST;
					
					// decrement q delay
					q_dlyce <= '1;
					q_dlyinc <= '1;
					temp_q_delay <= temp_q_delay - 1;
				
					// decrement cq delay
					cq_dlyce <= '1;
					cq_dlyinc <= '1;
					temp_cq_delay <= temp_cq_delay - 1;
				
					iodelay_counter <= iodelay_counter + 1;
				end else begin
				
					init_state <= STATE_OCLK_TIMING_UNDO_DEC_Q_DEC_CQ_DELAY;
				end
			
			end
		end
		
		STATE_OCLK_TIMING_UNDO_DEC_Q_DEC_CQ_DELAY: begin
		
			if (iodelay_counter != '0) begin
			
				// increment q delay
				q_dlyce <= '1;
				q_dlyinc <= '1;
				temp_q_delay <= temp_q_delay + 1;
				
				// increment cq delay
				cq_dlyce <= '1;
				cq_dlyinc <= '1;
				temp_cq_delay <= temp_cq_delay + 1;
				
				iodelay_counter <= iodelay_counter - 1;
				
			end else begin

				init_state <= STATE_OCLK_TIMING_ANALYZE_RESULT;
			end
		end
		
		STATE_OCLK_TIMING_ANALYZE_RESULT: begin
		
			iodelay_counter <= '0;
		
			if (inc_cq_q_found_valid_begin && dec_cq_q_found_valid_begin) begin
			
				if (inc_cq_q_valid_begin == '0 && dec_cq_q_valid_begin == '0) begin // there is already a valid timing without incrementing or decrementing the delay
				
					if (inc_cq_q_valid_end > dec_cq_q_valid_end) begin
					
						temp_inc_cq_q_delay <= (inc_cq_q_valid_end - dec_cq_q_valid_end) / 2;
						temp_dec_cq_q_delay <= '0;
					
						init_state <= STATE_OCLK_TIMING_SET_INC_CQ_Q_DELAY;
						
					end else begin
					
						temp_inc_cq_q_delay <= '0;
						temp_dec_cq_q_delay <= (dec_cq_q_valid_end - inc_cq_q_valid_end) / 2;
						
						init_state <= STATE_OCLK_TIMING_SET_DEC_CQ_Q_DELAY;
					end
				
				end else begin
				
					if (inc_cq_q_valid_begin < dec_cq_q_valid_begin) begin
					
						// increment delay of cq and q
						temp_inc_cq_q_delay <= (inc_cq_q_valid_end + inc_cq_q_valid_begin) / 2;
						temp_dec_cq_q_delay <= '0;
					
						init_state <= STATE_OCLK_TIMING_SET_INC_CQ_Q_DELAY;
						
					end else begin
					
						// decrement delay of cq and q
						temp_inc_cq_q_delay <= '0;
						temp_dec_cq_q_delay <= (dec_cq_q_valid_end + dec_cq_q_valid_begin) / 2;
						
					
						init_state <= STATE_OCLK_TIMING_SET_DEC_CQ_Q_DELAY;
					
					end
				
				end
				
			end else begin
			
				if (inc_cq_q_found_valid_begin) begin
				
					// valid timing only while incrementing cq and q delay
					// increment delay of cq and q
					temp_inc_cq_q_delay <= (inc_cq_q_valid_end + inc_cq_q_valid_begin) / 2;
					temp_dec_cq_q_delay <= '0;
					
					init_state <= STATE_OCLK_TIMING_SET_INC_CQ_Q_DELAY;
					
				end else begin
				
					if (dec_cq_q_found_valid_begin) begin
				
						// valid timing only while decrementing cq and q delay
						// decrement delay of cq and q
						temp_inc_cq_q_delay <= '0;
						temp_dec_cq_q_delay <= (dec_cq_q_valid_end + dec_cq_q_valid_begin) / 2;
						
					
						init_state <= STATE_OCLK_TIMING_SET_DEC_CQ_Q_DELAY;
						
					end else begin
					
						// no valid timing at all
						init_state <= STATE_MEMORY_ERROR;
					
					end
				
				end
			
			end
		end
		
		STATE_OCLK_TIMING_SET_INC_CQ_Q_DELAY: begin
			
			if (iodelay_counter != temp_inc_cq_q_delay) begin
			
				cq_dlyce <= '1;
				cq_dlyinc <= '1;
				q_dlyce <= '1;
				q_dlyinc <= '1;
				iodelay_counter <= iodelay_counter + 1;
				
			end else begin
			
				if (memtest_ready) begin
				
					if (simulation_speed_up) begin
					
						init_state <= STATE_READY;
						
					end else begin
					
						memtest_run <= MEMTEST_COMPLETE_TEST;
						init_state <= STATE_MEMTEST_COMPLETE_TEST;
					end
				
				end
			end
		end
		
		STATE_OCLK_TIMING_SET_DEC_CQ_Q_DELAY: begin
		
			if (iodelay_counter != temp_dec_cq_q_delay) begin
			
				cq_dlyce <= '1;
				q_dlyce <= '1;
				iodelay_counter <= iodelay_counter + 1;
				
			end else begin
			
				if (memtest_ready) begin

					if (simulation_speed_up) begin
					
						init_state <= STATE_READY;
						
					end else begin
					
						memtest_run <= MEMTEST_COMPLETE_TEST;
						init_state <= STATE_MEMTEST_COMPLETE_TEST;
					end
				
				end
			end
		
		end
		
		STATE_MEMTEST_COMPLETE_TEST: begin
		
			if (memtest_ready) begin
			
				if (memtest_last_result_ok) begin
				
					init_state <= STATE_READY; // for memory testing use this: STATE_MEMTEST_COMPLETE_TEST
					
				end else begin
				
					init_state <= STATE_MEMORY_ERROR;
				end
				
			end
		end
		
		STATE_READY: begin
		
		end
		
		STATE_MEMORY_ERROR: begin
		
		end
	
	endcase;
	
	if (reset) begin
		init_state <= STATE_WAIT_RAM_PLL;
		pll_lock_counter <= 0;
		memtest_run <= MEMTEST_NO_TEST;
	end
	
end

// test port signals
logic ready_tp = 0, rd_tp = 0, wr_tp = 0, r_valid_tp = 0;
logic [mem_addr_width-1:0] w_addr_tp, r_addr_tp;
logic [mem_data_width*4-1:0] w_data_tp;
wire [mem_data_width*4-1:0] r_data_tp = r_data_int;

always_comb begin

	ready <= 0;
	ready_tp <= 0;
	r_valid <= 0;

	if (init_state == STATE_READY) begin
	
		ready <= 1;
			
		rd_int <= rd;
		r_addr_int <= r_addr;
		r_valid <= r_valid_int;

		wr_int <= wr;
		w_addr_int <= w_addr;
		w_data_int <= w_data;
		
	end else begin
	
		ready_tp <= 1;
			
		rd_int <= rd_tp;
		r_addr_int <= r_addr_tp;
		r_valid_tp <= r_valid_int;
			
		wr_int <= wr_tp;
		w_addr_int <= w_addr_tp;
		w_data_int <= w_data_tp;
	end

end

/*
*		memory test
*/

const logic [mem_data_width*4-1:0] ddr_test_pattern = { { (mem_data_width/2){2'b10} }, { (mem_data_width/2){2'b01} }, { (mem_data_width/2){2'b10} }, { (mem_data_width/2){2'b01} } };

typedef struct {
	
	logic [mem_addr_width-1:0] addr;
	logic [mem_data_width*4-1:0] data;

} memory_test_patterns_t;

memory_test_patterns_t memory_test_patterns[4] = '{ // only 2^n patterns allowed!

	'{
		addr: {mem_addr_width{1'b1}},																														// addr: 0xff..
		data: { {(mem_data_width/2){2'b10}}, {(mem_data_width/2){2'b01}}, {mem_data_width{1'b0}}, {mem_data_width{1'b1}} }	// data: 0xaa..55..00..ff
	}, 
	'{
		addr: {mem_addr_width{1'b0}},																														// addr: 0x00..
		data: { {mem_data_width{1'b1}}, {(mem_data_width/2){2'b10}}, {(mem_data_width/2){2'b01}}, {mem_data_width{1'b0}} }	// data: 0xff..aa..55..00
	},
	'{
		addr: { {(mem_addr_width%2){1'b1}}, {(mem_addr_width/2){2'b01}} },																	// addr: 0x55..
		data: { {mem_data_width{1'b0}}, {mem_data_width{1'b1}}, {(mem_data_width/2){2'b10}}, {(mem_data_width/2){2'b01}} }	// data: 0x00..ff..aa..55
	},	
	'{
		addr: { {(mem_addr_width%2){1'b0}}, {(mem_addr_width/2){2'b10}} },																	// addr: 0xaa..
		data: { {(mem_data_width/2){2'b01}}, {mem_data_width{1'b0}}, {mem_data_width{1'b1}}, {(mem_data_width/2){2'b10}} }	// data: 0x55..00..ff..aa
	}
	
};

localparam mt_et_iterations_2n = 3; // memory test iterations for extensive test (as 2^n)
logic [$clog2($size(memory_test_patterns)) + mt_et_iterations_2n - 1 : 0] mt_et_it_counter, mt_et_check_it_counter;

localparam mt_ddr_iterations_2n = 3; // memory test iterations for output ddr test (as 2^n)
logic [mt_ddr_iterations_2n - 1 : 0] mt_ddr_it_counter, mt_ddr_check_it_counter;

localparam mt_complete_iterations_2n = 5; // memory test iterations for complete test (as 2^n) [5 takes about 100ms]
logic [mem_addr_width + mt_complete_iterations_2n - 1 : 0] mt_complete_it_counter, mt_complete_check_it_counter;

enum { MT_IDLE, MT_BUSY }  memtest_send_state, memtest_check_state;
assign memtest_ready = (memtest_send_state == MT_IDLE) && (memtest_check_state == MT_IDLE) && ready_tp && (memtest_run == MEMTEST_NO_TEST);

logic [$clog2(read_delay_cycles + 2) - 1 : 0] mt_read_delay_counter; // +2: two cycles more than read_delay_cycles

always_ff @(posedge clk) begin

	case (memtest_send_state)
	
		MT_IDLE: begin
		
			rd_tp <= 0;
			wr_tp <= 0;
			mt_et_it_counter <= 0;
			mt_ddr_it_counter <= 0;
			mt_complete_it_counter <= 0;
		
			if (ready_tp && memtest_check_state == MT_IDLE && memtest_run != MEMTEST_NO_TEST) begin
			
				memtest_send_state <= MT_BUSY;
				memtest_check_state <= MT_BUSY;
				
				selected_memtest <= memtest_run;
				
				case (memtest_run)
				
					MEMTEST_OUTPUT_DDR_TEST: begin // for ddr test: turn the sram's control signals on one cycle earlier (to prevent metastability inside iserdes)
					
						rd_tp <= 1;
						wr_tp <= 1;
						r_addr_tp <= 0;
						w_addr_tp <= 0;
						w_data_tp <= ddr_test_pattern;
					end
				
				endcase
			
			end
		end
		
		MT_BUSY: begin
	
			rd_tp <= 1;
			wr_tp <= 1;
			
			case (selected_memtest)
			
				MEMTEST_OUTPUT_DDR_TEST: begin
					
					mt_ddr_it_counter <= mt_ddr_it_counter + 1;
					if (mt_ddr_it_counter == '1)
						memtest_send_state <= MT_IDLE;
					
				end
			
				MEMTEST_EXTENSIVE_TEST: begin
			
					r_addr_tp <= memory_test_patterns[mt_et_it_counter[$clog2($size(memory_test_patterns))-1:0]].addr;
					w_addr_tp <= memory_test_patterns[mt_et_it_counter[$clog2($size(memory_test_patterns))-1:0]].addr;
					w_data_tp <= memory_test_patterns[mt_et_it_counter[$clog2($size(memory_test_patterns))-1:0]].data;
					
					mt_et_it_counter <= mt_et_it_counter + 1;
					if (mt_et_it_counter == '1)
						memtest_send_state <= MT_IDLE;
					
				end
				
				MEMTEST_COMPLETE_TEST: begin
			
					r_addr_tp <= memory_test_patterns[mt_complete_it_counter[$clog2($size(memory_test_patterns))-1:0]].addr;
					w_addr_tp <= memory_test_patterns[mt_complete_it_counter[$clog2($size(memory_test_patterns))-1:0]].addr;
					w_data_tp <= memory_test_patterns[mt_complete_it_counter[$clog2($size(memory_test_patterns))-1:0]].data;
					
					mt_complete_it_counter <= mt_complete_it_counter + 1;
					if (mt_complete_it_counter == '1)
						memtest_send_state <= MT_IDLE;
					
				end

			endcase
		end
	
	endcase
	
	case (memtest_check_state)
	
		MT_IDLE: begin
		
			mt_read_delay_counter <= 0;
			mt_et_check_it_counter <= 0;
			mt_ddr_check_it_counter <= 0;
			mt_complete_check_it_counter <= 0;
		
		end
		
		MT_BUSY: begin
		
			if (mt_read_delay_counter == read_delay_cycles + 1) begin // two cycles more than read_delay_cycles
			
				case (selected_memtest)
			
					MEMTEST_OUTPUT_DDR_TEST: begin
					
						if (r_data_tp == ddr_test_pattern) begin
						
							// no data error during this iteration
							if (mt_ddr_check_it_counter == '1) begin // this was the last iteration
								memtest_last_result_ok <= '1;
								memtest_check_state <= MT_IDLE;
							end

						end else begin
						
							// found a data error
							memtest_last_result_ok <= '0;
							memtest_send_state <= MT_IDLE; // abort the test
							memtest_check_state <= MT_IDLE;
							
						end
						
						mt_ddr_check_it_counter <= mt_ddr_check_it_counter + 1;
						
					end
					
					MEMTEST_EXTENSIVE_TEST: begin
			
						if (r_data_tp == memory_test_patterns[mt_et_check_it_counter[$clog2($size(memory_test_patterns))-1:0]].data) begin
						
							// no data error during this iteration
							if (mt_et_check_it_counter == '1) begin // this was the last iteration
								memtest_last_result_ok <= '1;
								memtest_check_state <= MT_IDLE;
							end
							
						end else begin
						
							// found a data error
							memtest_last_result_ok <= '0;
							memtest_send_state <= MT_IDLE; // abort the test
							memtest_check_state <= MT_IDLE;							
							
						end
						
						mt_et_check_it_counter <= mt_et_check_it_counter + 1;
				
					end
					
					MEMTEST_COMPLETE_TEST: begin
			
						if (r_data_tp == memory_test_patterns[mt_complete_check_it_counter[$clog2($size(memory_test_patterns))-1:0]].data) begin
							
							// no data error during this iteration
							if (mt_complete_check_it_counter == '1) begin // this was the last iteration
								memtest_last_result_ok <= '1;
								memtest_check_state <= MT_IDLE;
							end
							
						end else begin
						
							// found a data error
							memtest_last_result_ok <= '0;
							memtest_send_state <= MT_IDLE; // abort the test
							memtest_check_state <= MT_IDLE;
							
						end
						
						mt_complete_check_it_counter <= mt_complete_check_it_counter + 1;
				
					end
					
				endcase
				
			end else begin
			
				mt_read_delay_counter <= mt_read_delay_counter + 1;
				
			end
		
		end

	endcase
	
	if (reset) begin
		
		memtest_send_state <= MT_IDLE;
		memtest_check_state <= MT_IDLE;
		
	end

end

/*
*			Chipscope...
*/

`ifdef use_chipscope_qdr2_sram

init_states last_init_state;

always_ff @(posedge clk) begin
	last_init_state <= init_state;
end

wire state_changed = (init_state != last_init_state);
wire [7:0] state_chipscope = init_state;


ila_qdr2_sram ila
(
	.CLK(clk),
	.CONTROL(chipscope),
	
	.DATA( {

		state_chipscope,
		
		{ 4'b0, cq_found_valid_begin, q_found_valid_begin, inc_cq_q_found_valid_begin, dec_cq_q_found_valid_begin },
		
		// 8 bytes valid ranges
		{ 2'b0, cq_valid_begin },
		{ 2'b0, cq_valid_end },
		{ 2'b0, q_valid_begin },
		{ 2'b0, q_valid_end },
		{ 2'b0, inc_cq_q_valid_begin },
		{ 2'b0, inc_cq_q_valid_end },
		{ 2'b0, dec_cq_q_valid_begin },
		{ 2'b0, dec_cq_q_valid_end },
		
		// 4 bytes iodelay values
		{ 2'b0, temp_cq_delay },
		{ 2'b0, temp_q_delay },
		{ 2'b0, temp_inc_cq_q_delay },
		{ 2'b0, temp_dec_cq_q_delay }
		
	} ),
	
	.TRIG0(state_changed)
);

`endif

endmodule
