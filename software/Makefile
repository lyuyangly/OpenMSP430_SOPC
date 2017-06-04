#*
#***********************************************************************************************
#	Function  	: Makefile
#	Parameter 	: all clean debug flash
#	Author    	: Lyu Yang
#	Date      	: 2014
#	Description : Makefile for OpenMSP430
#***********************************************************************************************
#*

PROJECT = osmp

CROSS_COMPILE = msp430-

INCLUDE_DIRS = -I ./

LIBRARY_DIRS = -L ./

CC = $(CROSS_COMPILE)gcc
CFLAGS = $(INCLUDE_DIRS) -O2 -Wall -g -mcpu=430 -mivcnt=16 -mmpy=16

CXX = $(CROSS_COMPILE)g++
CXXFLAGS = $(INCLUDE_DIRS)

AS = $(CROSS_COMPILE)gcc
ASFLAGS = $(INCLUDE_DIRS) -c -x assembler-with-cpp

LD = $(CROSS_COMPILE)gcc
LDFLAGS = $(LIBRARY_DIRS) -mcpu=430 -T./omsp430.lds -Wl,-Map,$(PROJECT).map

OBJCP = $(CROSS_COMPILE)objcopy
OBJCPFLAGS = -O binary --gap-fill 0xff

AR = $(CROSS_COMPILE)ar
ARFLAGS = cr

DUMP = $(CROSS_COMPILE)objdump
DUMPFLAG = -DS

# User should list all object files
OBJS  = ./main.o

.PHONY: all clean debug flash
all: $(PROJECT).elf $(PROJECT).hex $(PROJECT).bin $(PROJECT).mif
	$(CROSS_COMPILE)size $(PROJECT).elf
	
$(PROJECT).elf: $(OBJS)
	$(LD) $(LDFLAGS) -o $@ $^

$(PROJECT).asm: $(PROJECT).elf
	$(DUMP) $(DUMPFLAG) $(PROJECT).elf > $(PROJECT).asm

$(PROJECT).hex: $(PROJECT).elf
	$(OBJCP) -O ihex --gap-fill 0xff $< $@
	
$(PROJECT).bin: $(PROJECT).elf
	$(OBJCP) $(OBJCPFLAGS) $< $@

$(PROJECT).mif: $(PROJECT).bin
	./mifwrite $^ $@

clean:
	@-rm -f $(PROJECT).elf $(PROJECT).hex $(PROJECT).asm *.log *.map *.mif *.bin
	@-rm -f *.o
	
debug:
	$(CROSS_COMPILE)gdb $(PROJECT).elf -x gdbinit.txt

gdbserver:
	$(CROSS_COMPILE)gdbproxy --debug msp430 --spy-bi-wire TIUSB

flash:
	$(CROSS_COMPILE)flash -n $(MCU) -w "$(PROJECT).hex" -v -g -z [VCC] 

download-uart: all
	openmsp430-loader.tcl -device /dev/ttyUSB0 -baudrate 115200 $(PROJECT).elf

download-jtag:
	$(CROSS_COMPILE)jtag -e $(PROJECT).elf

download-bsl:
	$(CROSS_COMPILE)bsl -e $(PROJECT).elf
