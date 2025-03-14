sim:
	vcs -R +v2k -full64 tb.v ../hdl/fir.v -debug_acc +nospecify 
	#vcs -R +v2k -full64 testfixture_b.v ../hdl/fir.v -debug_acc +nospecify
	#vcs -R +v2k -full64 testfixture_c.v ../hdl/fir.v -debug_acc +nospecify
gatesim:
	vcs -R +v2k -full64 tb.v ../hdl/bram32.v ../syn/netlist/fir_syn.v -v /usr/cadtool/ee5216/CBDK_TSMC90GUTM_Arm_f1.0/CIC/Verilog/tsmc090.v +define+SDF -debug_acc +neg_tchk +nospecify   
	#vcs -R +v2k -full64 testfixture_b.v ../syn/netlist/fir_syn.v -v /usr/cadtool/ee5216/CBDK_TSMC90GUTM_Arm_f1.0/CIC/Verilog/tsmc090.v +define+SDF -debug_acc +neg_tchk
	#vcs -R +v2k -full64 testfixture_c.v ../syn/netlist/fir_syn.v -v /usr/cadtool/ee5216/CBDK_TSMC90GUTM_Arm_f1.0/CIC/Verilog/tsmc090.v +define+SDF -debug_acc +neg_tchk
