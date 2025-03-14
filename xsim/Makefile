xsim:
	xvlog -f ./include.rtl.list.xsim
	xelab -top fir_tb -debug all -snapshot testbench_elab
	xsim testbench_elab -R
clean:
	rm -rf xsim.dir/ *.log *.pb *.jou *.wdb
	rm fir.vcd
	rm answer.txt