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

data("pa_fifo_di", 256)
data("pa_fifo_mi", 19)
data("pa_fifo_wren")
data("pa_fifo_commit")
data("pa_fifo_revert")
data("pa_fifo_mo", 19)
data("pa_fifo_eop")
data("pa_fifo_full")
data("pa_fifo_empty")
data("pa_fifo_below_low")
data("pa_fifo_above_high")
data("wide_fifo_error")
data("wide_fifo_dop", 32)
data("wide_fifo_empty")
data("wide_fifo_almost_emtpy")
data("dropping")
data("pkt_length", 14)
data("next_pkt_length", 14)
data("length_check_ok")
data("wide_fifo_pkt_length", 14)
data("wide_fifo_almost_full_reg")
data("wide_fifo_rderr")
data("wide_fifo_wrerr")
data("pa_fifo_rderr")
data("pa_fifo_wrerr")
data("wide_fifo_full")

data("err_tvalid_deasserted_0");
data("err_tkeep_encoded_wrong_0");
data("err_tkeep_not_continuous_0");
data("err_too_big_0");
data("err_too_small_0");

print ("Project.unit.0.dataWidth=512")

trig("wide_fifo_error")
trig("wide_fifo_empty")
trig("dropping")
trig("length_check_ok")
trig("pa_fifo_wren")
trig("pa_fifo_commit")
trig("pa_fifo_revert")
trig("pa_fifo_full")
trig("pa_fifo_empty")
trig("pa_fifo_below_low")
trig("pa_fifo_above_high")
trig("wide_fifo_rdwrerr")
trig("pa_fifo_rdwrerr")
trig("wide_fifo_full")
trig("axis_error_0")

print ("Project.unit.0.triggerWidth=16")

