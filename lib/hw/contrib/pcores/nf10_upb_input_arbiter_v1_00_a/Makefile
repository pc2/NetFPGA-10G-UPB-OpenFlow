install: synthesize

unittest:
	hdl/verilog/nf10_upb_input_arbiter_tb.sh

synthesize: netlist/4_ports/nf10_upb_input_arbiter.edf netlist/5_ports/nf10_upb_input_arbiter.edf netlist/6_ports/nf10_upb_input_arbiter.edf netlist/7_ports/nf10_upb_input_arbiter.edf netlist/8_ports/nf10_upb_input_arbiter.edf

netlist/%_ports/nf10_upb_input_arbiter.edf: prj/nf10_upb_input_arbiter_%.prj hdl/verilog/*.v
	cd prj && $(SYNPLIFY_CMD) -batch ../$<

hdl/verilog/%.v:


clean:
	rm -rf hdl/verilog/unittest_build
	rm -rf netlist
	-find prj/* ! -name "arbiter.xise" ! -name "nf10_upb_input_arbiter.sdc" ! -name "nf10_upb_input_arbiter_4.prj" ! -name "nf10_upb_input_arbiter_5.prj" ! -name "nf10_upb_input_arbiter_6.prj" ! -name "nf10_upb_input_arbiter_7.prj" ! -name "nf10_upb_input_arbiter_8.prj" ! -exec rm -rf {} \; | true

all: unittest install

.PHONY: unittest clean
.INTERMEDIATE: synthesize
