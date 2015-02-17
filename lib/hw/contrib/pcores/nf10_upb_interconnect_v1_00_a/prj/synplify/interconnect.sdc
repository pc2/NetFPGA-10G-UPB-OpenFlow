create_clock -period 6.666 [get_ports axi_aclk]
create_clock -period 8.000 [get_ports GTXD8_P]
create_clock -period 50.000 [get_ports INIT_CLK]
derive_pll_clocks

