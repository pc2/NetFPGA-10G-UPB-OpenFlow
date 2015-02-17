cd $(dirname $0)
rm -rf unittest_build
mkdir  unittest_build
cd     unittest_build
fuse -incremental -prj ../nf10_upb_input_arbiter_tb.prj -o nf10_upb_input_arbiter_tb.exe work.nf10_upb_input_arbiter_tb
./nf10_upb_input_arbiter_tb.exe -tclbatch ../nf10_upb_input_arbiter_tb.tcl
