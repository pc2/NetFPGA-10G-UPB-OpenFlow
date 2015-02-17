`timescale 10ns / 10ps


module xgmac_to_axi_converter_tx_tb;

	// Inputs
	reg reset;
	reg clk156;
	reg tx_ack;
	reg [63:0] rx_data;
	reg [7:0] rx_data_valid;
	reg rx_good_frame;
	reg rx_bad_frame;
	reg m_axis_tready;
	reg [63:0] s_axis_tdata;
	reg [7:0] s_axis_tkeep;
	reg [0:0] s_axis_tuser;
	reg s_axis_tvalid;
	reg s_axis_tlast;

	// Outputs
	wire [63:0] tx_data;
	wire [7:0] tx_data_valid;
	wire tx_start;
	wire [63:0] m_axis_tdata;
	wire [7:0] m_axis_tkeep;
	wire [0:0] m_axis_tuser;
	wire m_axis_tvalid;
	wire m_axis_tlast;
	wire s_axis_tready;

	// Instantiate the Unit Under Test (UUT)
	xgmac_to_axi_converter uut (
		.reset(reset), 
		.clk156(clk156), 
		.tx_data(tx_data), 
		.tx_data_valid(tx_data_valid), 
		.tx_start(tx_start), 
		.tx_ack(tx_ack), 
		.rx_data(rx_data), 
		.rx_data_valid(rx_data_valid), 
		.rx_good_frame(rx_good_frame), 
		.rx_bad_frame(rx_bad_frame), 
		.m_axis_tdata(m_axis_tdata), 
		.m_axis_tkeep(m_axis_tkeep), 
		.m_axis_tuser(m_axis_tuser), 
		.m_axis_tvalid(m_axis_tvalid), 
		.m_axis_tready(m_axis_tready), 
		.m_axis_tlast(m_axis_tlast), 
		.s_axis_tdata(s_axis_tdata), 
		.s_axis_tkeep(s_axis_tkeep), 
		.s_axis_tuser(s_axis_tuser), 
		.s_axis_tvalid(s_axis_tvalid), 
		.s_axis_tready(s_axis_tready), 
		.s_axis_tlast(s_axis_tlast)
	);
    
    wire [63:0] tdata[0:8];
	assign tdata[0] = 64'h0000111111111111;
	assign tdata[1] = 64'h1FFEDEAD00000000;
	assign tdata[2] = 64'h2FFEDEADAFFEDEAD;
	assign tdata[3] = 64'h3EADBEEFDEADBEEF;
	assign tdata[4] = 64'h4FFEDEADAFFEDEAD;
	assign tdata[5] = 64'h5EADBEEFDEADBEEF;
	assign tdata[6] = 64'h6FFEDEADAFFEDEAD;
	assign tdata[7] = 64'h7EADBEEFDEADBEEF;
	assign tdata[8] = 64'h71234567890ABEEF;

    wire [7:0] tkeep[0:8];
	assign tkeep[0] = 8'b11111111;
	assign tkeep[1] = 8'b11111111;
	assign tkeep[2] = 8'b11111111;
	assign tkeep[3] = 8'b11111111;
	assign tkeep[4] = 8'b11111111;
	assign tkeep[5] = 8'b11111111;
	assign tkeep[6] = 8'b11111111;
	assign tkeep[7] = 8'b11111111;
	assign tkeep[8] = 8'b00000011;
	
    
	initial begin
		// Initialize Inputs
		reset = 0;
		clk156 = 0;
		tx_ack = 0;
		rx_data = 0;
		rx_data_valid = 0;
		rx_good_frame = 0;
		rx_bad_frame = 0;
		m_axis_tready = 0;
		s_axis_tdata = 1000;
		s_axis_tkeep = 0;
		s_axis_tuser = 0;
		s_axis_tvalid = 0;
		s_axis_tlast = 0;

		// Wait for global reset to finish
		#101;
		#1000;
		@(posedge clk156)
			axis_state <= SENDING;
	end
	
	integer current = 0;
	parameter IDLE = 0;
	parameter SENDING = 1;
	
	integer ack_counter =0;
	integer ack_countd =0;
	reg[1:0] axis_state = IDLE;
	always @(*) begin
	    s_axis_tvalid = 1'b0;
	    if(axis_state == SENDING) begin
    	    s_axis_tdata = tdata[current];
	        s_axis_tvalid = 1'b1;
	        s_axis_tkeep = tkeep[current];
	        s_axis_tlast =0;
	        if(current == 8) begin
	            s_axis_tlast = 1;
            end
		end
	end
    always @(*) begin
        tx_ack=0;
        if(ack_countd==1) begin
            tx_ack=1;
        end
    end
	always @(posedge clk156) begin
        if(s_axis_tready && s_axis_tvalid) begin
            if(current < 8) begin
	            current <= current + 1;
            end
            else begin
                //axis_state <= IDLE;
                current <= 0;
            end
        end
        if(ack_countd>0) begin
            ack_countd <= ack_countd - 1;
        end
        if(tx_start) begin
            ack_counter <= ack_counter + 1;
            if(ack_counter < 3 || ack_counter > 4) begin
                ack_countd <= 2;
            end
            if(ack_counter == 3) begin
                ack_countd <= 10;
            end
            if(ack_counter == 4) begin
                ack_countd <= 1;
            end
            
        end
	end
	always #100 clk156 = ~clk156;
endmodule


