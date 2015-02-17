data_bit = 0
def data(name, count = 1):
 for i in range (0,count):
  global data_bit
  print ("Project.unit.0.data<" + str(data_bit) + ">=" + name  + ("" if count == 1 else "[" + str(i) + "]"))
  data_bit = data_bit + 1

trig_bit = 0
def trig(name, count = 1):
 for i in range (0,count):
  global trig_bit
  print ("Project.unit.0.trigger<" + str(trig_bit) + ">=" + name  + ("" if count == 1 else "[" + str(i) + "]"))
  trig_bit = trig_bit + 1

print ("#ChipScope Core Inserter Project Version 2.0")

data("channel_up_i")
data("lane_up_i", 10)
data("soft_err_i")
data("hard_err_i")
data("tx_stopped")
data("m_axis_tvalid")
data("m_axis_tlast")
data("m_axis_tkeep", 32)
data("s_axis_tvalid")
data("s_axis_tready")
data("s_axis_tlast")
data("s_axis_tkeep", 32)
data("flow_control_pause_req")
data("flow_control_pause_val")
data("error")
data("tx_d_i",320)
data("tx_src_rdy_n_i")
data("tx_dst_rdy_n_i")
data("rx_d_i", 320)
data("rx_src_rdy_n_i")

data("err_tvalid_deasserted_0");
data("err_tkeep_encoded_wrong_0");
data("err_tkeep_not_continuous_0");
data("err_too_big_0");
data("err_too_small_0");

data("err_tvalid_deasserted_1");
data("err_tkeep_encoded_wrong_1");
data("err_tkeep_not_continuous_1");
data("err_too_big_1");
data("err_too_small_1");

print ("Project.unit.0.dataWidth=1024")

trig("channel_up_i")
trig("soft_err_i")
trig("hard_err_i")
trig("tx_stopped")
trig("m_axis_tvalid")
trig("m_axis_tlast")
trig("s_axis_tvalid")
trig("s_axis_tready")
trig("s_axis_tlast")
trig("flow_control_pause_req")
trig("flow_control_pause_val")
trig("tx_src_rdy_n_i")
trig("tx_dst_rdy_n_i")
trig("rx_src_rdy_n_i")
trig("axis_error_0")
trig("axis_error_1")

print ("Project.unit.0.triggerWidth=16")

