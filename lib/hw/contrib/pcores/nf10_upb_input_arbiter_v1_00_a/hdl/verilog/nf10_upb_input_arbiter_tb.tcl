run 800ns
if {! ([test /nf10_upb_input_arbiter_tb/uut/m_axis_tlast 1] & [test /nf10_upb_input_arbiter_tb/uut/m_axis_tvalid 1] & [test /nf10_upb_input_arbiter_tb/uut/m_axis_tdata(255:0) c0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c0 -radix hex])} {
    puts "Test Failed"
    quit
}
run 3200ns #4000
if {! ([test /nf10_upb_input_arbiter_tb/uut/m_axis_tlast 0] & [test /nf10_upb_input_arbiter_tb/uut/m_axis_tvalid 1] & [test /nf10_upb_input_arbiter_tb/uut/m_axis_tdata(255:0) b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0 -radix hex])} {
    puts "Test Failed"
    quit
}
run 6000ns #10000
if {! ([test /nf10_upb_input_arbiter_tb/uut/m_axis_tlast 0] & [test /nf10_upb_input_arbiter_tb/uut/m_axis_tvalid 1] & [test /nf10_upb_input_arbiter_tb/uut/m_axis_tdata(255:0) b2b2b2b2b2b2b2b2b2b2b2b2b2b2b2b2b2b2b2b2b2b2b2b2b2b2b2b2b2b2b2b2 -radix hex])} {
    puts "Test Failed"
    quit
}
run 7400ns #17400
if {! ([test /nf10_upb_input_arbiter_tb/uut/m_axis_tlast 1] & [test /nf10_upb_input_arbiter_tb/uut/m_axis_tvalid 1] & [test /nf10_upb_input_arbiter_tb/uut/m_axis_tdata(255:0) c2c2c2c2c2c2c2c2c2c2c2c2c2c2c2c2c2c2c2c2c2c2c2c2c2c2c2c2c2c2c2c2 -radix hex])} {
    puts "Test Failed"
    quit
}
run 13200ns #30600
if {! ([test /nf10_upb_input_arbiter_tb/uut/m_axis_tvalid 1] & [test /nf10_upb_input_arbiter_tb/uut/m_axis_tdata(255:0) c2c2c2c2c2c2c2c2c2c2c2c2c2c2c2c2c2c2c2c2c2c2c2c2c2c2c2c2c2c2c2c2 -radix hex])} {
    puts "Test Failed"
    quit
}
run 200ns #30800
if {! ([test /nf10_upb_input_arbiter_tb/uut/m_axis_tvalid 1] & [test /nf10_upb_input_arbiter_tb/uut/m_axis_tdata(255:0) a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0 -radix hex])} {
    puts "Test Failed"
    quit
}
run 11400ns #42200
if {! ([test /nf10_upb_input_arbiter_tb/uut/m_axis_tlast 0] & [test /nf10_upb_input_arbiter_tb/uut/m_axis_tvalid 1] & [test /nf10_upb_input_arbiter_tb/uut/m_axis_tdata(255:0) a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0 -radix hex])} {
    puts "Test Failed"
    quit
}
run 2000ns #44200
if {! ([test /nf10_upb_input_arbiter_tb/uut/m_axis_tvalid 0])} {
    puts "Test Failed"
    quit
}
puts "Test Passed"
quit

