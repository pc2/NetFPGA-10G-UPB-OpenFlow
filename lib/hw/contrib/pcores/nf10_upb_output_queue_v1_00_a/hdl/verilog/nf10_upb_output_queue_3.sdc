create_clock -period 6.666 [get_ports clk]
create_generated_clock -source [get_ports clk] -multiply_by 2 [get_ports clk2x]
create_generated_clock -source [get_ports clk] -multiply_by 2 -phase 90 [get_ports clk2x90]
create_clock -period 6.666 [get_ports m_axis_0_clk]
create_clock -period 6.666 [get_ports m_axis_1_clk]
create_clock -period 6.666 [get_ports m_axis_2_clk]
derive_pll_clocks

