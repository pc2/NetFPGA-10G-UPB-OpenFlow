install: synthesize

synthesize: netlist/nf10_upb_switch.edf

netlist/nf10_upb_switch.edf: hdl/SystemVerilog/*.v
	cd prj && $(SYNPLIFY_CMD) -batch nf10_upb_switch.prj
	@mkdir -p netlist
	@cp prj/nf10_upb_switch.edf netlist/

hdl/SystemVerilog/%.v:

clean:
	-rm -rf netlist
	
.INTERMEDIATE: synthesize
.PHONY: clean
