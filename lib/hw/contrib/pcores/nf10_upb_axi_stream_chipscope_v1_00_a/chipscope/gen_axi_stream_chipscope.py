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

data("reset")
data("axis_tvalid")
data("axis_tready")
data("axis_tdata",256)
data("axis_tkeep",32)
data("axis_tlast")
data("axis_tuser_packet_length",14)
data("axis_tuser_in_port",3)
data("axis_tuser_in_vport",3)
data("axis_tuser_out_port",8)
data("axis_tuser_out_vport",8)
data("err_tvalid_deasserted");
data("err_tkeep_encoded_wrong");
data("err_tkeep_not_continuous");
data("err_too_big");
data("err_too_small");

print ("Project.unit.0.dataWidth=512")

trig("reset")
trig("axis_tvalid")
trig("axis_tready")
trig("axis_error")

print ("Project.unit.0.triggerWidth=16")

