#*
#***********************************************************************************************
#	Function  	: Makefile
#	Parameter 	: all clean download
#	Author    	: Lyu Yang
#	Date      	: 2014
#	Description : Makefile for ISE Project on Windows x86_64
#***********************************************************************************************
#*

PROJECT = OpenMSP430_FPGA
DEVICE = xc6slx16
PACKAGE = csg324
SPEED = 3

all: synthesize translate map par  trce bitgen
	@echo Project $(PROJECT)

synthesize:
	@echo "Synthesize - XST"
	xst -intstyle ise -ifn "./$(PROJECT).xst" -ofn ./$(PROJECT).syr
translate:	
	@echo "Translate"
	ngdbuild -intstyle ise -dd _ngo -sd ../../rtl/xilinx -nt timestamp -uc ./$(PROJECT).ucf -p $(DEVICE)-$(PACKAGE)-$(SPEED) $(PROJECT).ngc $(PROJECT).ngd
map:
	@echo "Map"
	map -intstyle ise -p $(DEVICE)-$(PACKAGE)-$(SPEED) -w -logic_opt off -ol high -t 1 -xt 0 -register_duplication off -r 4 -global_opt off -mt off -ir off -pr off -lc off -power off -o $(PROJECT)_map.ncd $(PROJECT).ngd $(PROJECT).pcf
par:
	@echo "Place & Route"
	par -w -intstyle ise -xe n $(PROJECT)_map.ncd $(PROJECT).ncd $(PROJECT).pcf
trce:
	@echo "Generate Post-Place & Route Static Timing"
	trce -intstyle ise -v 3 -s 3 -n 3 -fastpaths -xml $(PROJECT).twx $(PROJECT).ncd -o $(PROJECT).twr $(PROJECT).pcf
bitgen:
	@echo "Generate Programming File"
	bitgen -intstyle ise -f $(PROJECT).ut $(PROJECT).ncd

clean:
	del *.bak *.lso *.syr *.xrpt *.xml *.ngc *.ngr *.map
	del *.mrp *.ncd *.pcf *.ngm *.twr *.twx *.xmsgs *.drc
	del *.bit *.bgn *.ut *.log *.html *.stx *.cmd_log

download:




