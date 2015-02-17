
COREGEN_DIR:= coregen

install: hdl/verilog/aurora/aurora_8b10b_v5_3.v netlist/nf10_upb_interconnect.edf

.NOTPARALLEL: hdl/verilog/aurora/aurora_8b10b_v5_3.v netlist/nf10_upb_interconnect.edf

hdl/verilog/aurora/aurora_8b10b_v5_3.v: xco/aurora_8b10b_v5_3.xco
	@mkdir -p netlist;
	@mkdir -p $(COREGEN_DIR);
	@cd $(COREGEN_DIR) && coregen -b ../xco/aurora_8b10b_v5_3.xco
	@cp $(COREGEN_DIR)/aurora_8b10b_v5_3.v hdl/verilog/aurora
	@cp $(COREGEN_DIR)/aurora_8b10b_v5_3/src/*.v hdl/verilog/aurora
	@cp $(COREGEN_DIR)/aurora_8b10b_v5_3/example_design/cc_manager/aurora_8b10b_v5_3_standard_cc_module.v hdl/verilog/aurora
	@cp $(COREGEN_DIR)/aurora_8b10b_v5_3/example_design/clock_module/aurora_8b10b_v5_3_clock_module.v hdl/verilog/aurora
	@cp $(COREGEN_DIR)/aurora_8b10b_v5_3/example_design/gt/aurora_8b10b_v5_3_tile.v hdl/verilog/aurora
	@cp $(COREGEN_DIR)/aurora_8b10b_v5_3/example_design/gt/aurora_8b10b_v5_3_transceiver_wrapper.v hdl/verilog/aurora
	@cp $(COREGEN_DIR)/aurora_8b10b_v5_3/example_design/aurora_8b10b_v5_3_reset_logic.v hdl/verilog/aurora
	@patch hdl/verilog/aurora/aurora_8b10b_v5_3_reset_logic.v hdl/verilog/aurora/aurora_8b10b_v5_3_reset_logic.patch
	@echo "/////////////////////////////////////////";
	@echo "//Aurora core installed.";
	@echo "/////////////////////////////////////////";
	@rm -rf $(COREGEN_DIR);

netlist/nf10_upb_interconnect.edf:
	cd prj/synplify && $(SYNPLIFY_CMD) -batch interconnect.prj
	mkdir -p netlist
	cp prj/synplify/nf10_upb_interconnect/nf10_upb_interconnect.edf netlist/
	@echo "/////////////////////////////////////////";
	@echo "//Interconnect edif created.";
	@echo "/////////////////////////////////////////";

clean:
	@rm -rf $(COREGEN_DIR);
	@rm -rf netlist;
	@rm -rf hdl/verilog/aurora/*.v
	@echo "/////////////////////////////////////////";
	@echo "//Xilinx files removed.";
	@echo "/////////////////////////////////////////";
