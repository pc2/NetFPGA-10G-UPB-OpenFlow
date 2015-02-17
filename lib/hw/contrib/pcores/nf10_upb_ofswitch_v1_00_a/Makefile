.NOTPARALLEL: # coregen cannot be run in parallel in the same directory (same tmp filenames)

install: synthesize

cores: prj/ipcore_dir/action_fifo.ngc prj/ipcore_dir/fifo.ngc

prj/ipcore_dir/action_fifo.ngc: prj/action_fifo.xco
	mkdir -p prj/ipcore_dir/
	cd prj/ipcore_dir/ && coregen -b ../action_fifo.xco

prj/ipcore_dir/fifo.ngc: prj/fifo.xco
	mkdir -p prj/ipcore_dir/
	cd prj/ipcore_dir/ && coregen -b ../fifo.xco

%.xco:

%.v:


synthesize: netlist/nf10_upb_ofswitch.edf

netlist/nf10_upb_ofswitch.edf: cores hdl/SystemVerilog/*.v
	cd prj && $(SYNPLIFY_CMD) -batch nf10_upb_ofswitch.prj 
	@mkdir -p netlist
	cp prj/nf10_upb_ofswitch.edf netlist/

all: install

clean:
	-rm -rf netlist
	-rm prj/nf10_upb_ofswitch.edf
	-rm prj/ipcore_dir/action_fifo.ngc
	-rm prj/ipcore_dir/action_fifo.v
	-rm prj/ipcore_dir/fifo.ngc
	-rm prj/ipcore_dir/fifo.v

.PHONY: clean

.INTERMEDIATE: synthesize cores
