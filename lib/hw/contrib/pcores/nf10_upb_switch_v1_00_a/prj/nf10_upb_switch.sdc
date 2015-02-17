create_clock -period 150MHz [get_ports clk]
create_generated_clock -source [get_ports clk] -multiply_by 2 [get_ports clk2x]
create_generated_clock -source [get_ports clk] -multiply_by 2 -phase 90 [get_ports clk2x90]
derive_pll_clocks