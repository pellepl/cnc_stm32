BINARY = cnc_stm32

############
#
# STM Device settings
#
############

FLAGS += -DSYSCLK_FREQ_72MHz=72000000
FLAGS += -DSTM32F10X_HD
STARTUP = startup_stm32f10x_hd.s

############
#
# Paths
#
############

sourcedir = src
builddir = build

basetoolsdir = /home/petera/toolchain/gcc/arm-elf-tools-4.8.1
#basetoolsdir = /home/petera/toolchain/gcc/arm-elf-tools-4.7.1
#basetoolsdir = /usr/local/gcc/arm-elf-tools-4.6.3
#/home/petera/toolchain/gcc/arm-elf-tools-4.6.2
#codir = ${basetoolsdir}/lib/gcc/arm-none-eabi/4.8.1/

hfile = ${sourcedir}/config_header.h

stmlibdir = STM32F10x_StdPeriph_Lib_V3.5.0/Libraries
stmdriverdir = ${stmlibdir}/STM32F10x_StdPeriph_Driver
stmcmsisdir = ${stmlibdir}/CMSIS/CM3/DeviceSupport/ST/STM32F10x
stmcmsisdircore = ${stmlibdir}/CMSIS/CM3/CoreSupport

tools = ${basetoolsdir}/bin

CPATH =
SPATH =
INC =
SFILES =
CFILES =
RFILES =

#############
#
# Build tools
#
#############

CROSS_COMPILE=${tools}/arm-none-eabi-
#CROSS_COMPILE=${tools}/arm-elf-
CC = $(CROSS_COMPILE)gcc $(COMPILEROPTIONS)
AS = $(CROSS_COMPILE)gcc $(ASSEMBLEROPTIONS)
LD = $(CROSS_COMPILE)ld
GDB = $(CROSS_COMPILE)gdb
OBJCOPY = $(CROSS_COMPILE)objcopy
OBJDUMP = $(CROSS_COMPILE)objdump
SIZE = $(CROSS_COMPILE)size
MKDIR = mkdir -p

###############
#
# Build configs
#
###############

INCLUDE_DIRECTIVES =
COMPILEROPTIONS = $(INCLUDE_DIRECTIVES) $(FLAGS) -mcpu=cortex-m3 -mno-thumb-interwork -mthumb -Wall -gdwarf-2
#-ffunction-sections -fdata-sections
COMPILEROPTIONS += -O2
# -nostartfiles -nostdlib 
ASSEMBLEROPTION = $(COMPILEROPTIONS)
LINKERSCRIPT = arm.ld
LINKEROPTIONS = --gc-sections -cref
OBJCOPYOPTIONS_HEX = -O ihex ${builddir}/$(BINARY).elf
OBJCOPYOPTIONS_BIN = -O binary ${builddir}/$(BINARY).elf

BUILD_NUMBER_FILE=build-number.txt

###############
#
# Files and libs
#
###############

# app files

CPATH 		+= ${sourcedir}
SPATH 		+= ${sourcedir}
INC			+= -I./${sourcedir}

SFILES 		+= stm32f10x_it_h.s

CFILES 		+= main.c
CFILES 		+= processor.c
CFILES 		+= cnc_control.c
CFILES 		+= system.c
CFILES 		+= led.c
CFILES 		+= nvstorage.c
CFILES 		+= config.c
CFILES 		+= timer.c
CFILES 		+= adc.c
CFILES 		+= cli.c
CFILES 		+= eval.c
CFILES 		+= comm_impl.c
CFILES 		+= comm_impl_uart.c
CFILES 		+= comm_impl_udp.c
CFILES 		+= comm_proto_sys.c
CFILES 		+= comm_proto_cnc.c
CFILES 		+= enc28j60_spi_eth.c
CFILES 		+= crc.c
CFILES 		+= comm_proto_file.c
CFILES 		+= spiffs_wrapper.c
CFILES 		+= heap.c
CFILES 		+= ringbuf.c

# comm files
include ../generic/comm/files.mk

# tinyheap files
include ../generic/tinyheap/files.mk
		
# enc28j60 driver files
include ../generic/enc28j60_stm32/files.mk
		
# spiffs files
include ../generic/spiffs/files.mk
		
# stm32 lib files
SPATH	+= ${stmdriverdir}/src ${stmcmsisdir} ${stmcmsisdir}/startup/gcc_ride7
SFILES 	+= $(STARTUP)

CPATH	+= ${stmdriverdir}/src ${stmcmsisdir} ${stmcmsisdircore}
INC		+= -I./${stmdriverdir}/inc
INC		+= -I./${stmcmsisdir}
INC		+= -I./${stmcmsisdircore}

CFILES	+= misc.c
CFILES	+= stm32f10x_adc.c
CFILES	+= stm32f10x_bkp.c
CFILES	+= stm32f10x_can.c
CFILES	+= stm32f10x_cec.c
CFILES	+= stm32f10x_crc.c
CFILES	+= stm32f10x_dac.c
CFILES	+= stm32f10x_dbgmcu.c
CFILES	+= stm32f10x_dma.c
CFILES	+= stm32f10x_exti.c
CFILES	+= stm32f10x_flash.c
CFILES	+= stm32f10x_fsmc.c
CFILES	+= stm32f10x_gpio.c
CFILES	+= stm32f10x_i2c.c
CFILES	+= stm32f10x_iwdg.c
CFILES	+= stm32f10x_pwr.c
CFILES	+= stm32f10x_rcc.c
CFILES	+= stm32f10x_rtc.c
CFILES	+= stm32f10x_sdio.c
CFILES	+= stm32f10x_spi.c
CFILES	+= stm32f10x_tim.c
CFILES	+= stm32f10x_usart.c
CFILES	+= stm32f10x_wwdg.c
		
# cmsis files
CFILES	+= system_stm32f10x.c
CFILES	+= core_cm3.c

# stm32 system
CFILES 	+= stm32f10x_it.c

# generic system configuration
include config.mk 

LIBS = 

BINARYEXT = .hex

############
#
# Tasks
#
############

vpath %.c $(CPATH)
vpath %.s $(SPATH)
INCLUDE_DIRECTIVES += $(INC)

SOBJFILES = $(SFILES:%.s=${builddir}/%.o)
OBJFILES = $(CFILES:%.c=${builddir}/%.o)
ROBJFILES = $(RFILES:%.c=${builddir}/%.o)

DEPFILES = $(CFILES:%.c=${builddir}/%.d)
DEPFILES += $(RFILES:%.c=${builddir}/%.d)

ALLOBJFILES  = $(SOBJFILES)
ALLOBJFILES += $(OBJFILES)
ALLOBJFILES += $(ROBJFILES)

DEPENDENCIES = $(DEPFILES) 

# link object files, create binary for flashing
$(BINARY): $(ALLOBJFILES)
	@echo "... build info"
	@if ! test -f $(BUILD_NUMBER_FILE); then echo 0 > $(BUILD_NUMBER_FILE); fi
	@echo $$(($$(cat $(BUILD_NUMBER_FILE)) + 1)) > $(BUILD_NUMBER_FILE)	
	@echo "... linking"
	@${LD} $(LINKEROPTIONS) $(BUILD_NUMBER_LDFLAGS) -T $(LINKERSCRIPT) -Map ${builddir}/$(BINARY).map -o ${builddir}/$(BINARY).elf $(ALLOBJFILES) $(LIBS)
	@echo "... objcopy"
	@${OBJCOPY} $(OBJCOPYOPTIONS_BIN) ${builddir}/$(BINARY).out
	@${OBJCOPY} $(OBJCOPYOPTIONS_HEX) ${builddir}/$(BINARY)$(BINARYEXT) 
	@echo "... disasm"
	@${OBJDUMP} -hd -j .text -j.data -j .bss -j .bootloader_text -j .bootloader_data -d -S ${builddir}/$(BINARY).elf > ${builddir}/$(BINARY)_disasm.s
	@echo "${BINARY}.out is `du -b ${builddir}/${BINARY}.out | sed 's/\([0-9]*\).*/\1/g '` bytes on flash"

-include $(DEPENDENCIES)	   	

# compile assembly files, arm
$(SOBJFILES) : ${builddir}/%.o:%.s
		@echo "... assembly $@"
		@${AS} -c -o $@ $<
		
# compile c files
$(OBJFILES) : ${builddir}/%.o:%.c
		@echo "... compile $@"
		@${CC} -c -o $@ $<

# compile c files deisgnated for ram
$(ROBJFILES) : ${builddir}/%.o:%.c
		@echo "... ram compile $@"
		@${CC} -c -o $@ $< 

# make dependencies
$(DEPFILES) : ${builddir}/%.d:%.c
		@echo "... depend $@"; \
		rm -f $@; \
		${CC} $(COMPILEROPTIONS) -M $< > $@.$$$$; \
		sed 's,\($*\)\.o[ :]*, ${builddir}/\1.o $@ : ,g' < $@.$$$$ > $@; \
		rm -f $@.$$$$

all: info mkdirs $(BINARY)

info:
	@echo "* Building to ${builddir}"
	@echo "* Compiler options:  $(COMPILEROPTIONS)" 
	@echo "* Assembler options: $(ASSEMBLEROPTIONS)" 
	@echo "* Linker options:    $(LINKEROPTIONS)" 
	@echo "* Linker script:     ${LINKERSCRIPT}"
	
mkdirs:
	-@${MKDIR} ${builddir}
	
clean:
	@echo ... removing build files in ${builddir}
	@rm -f ${builddir}/*.o
	@rm -f ${builddir}/*.d
	@rm -f ${builddir}/*.out
	@rm -f ${builddir}/*.hex
	@rm -f ${builddir}/*.elf
	@rm -f ${builddir}/*.map
	@rm -f ${builddir}/*_disasm.s
	@rm -f _stm32flash.script

install: $(BINARY)
	@sed 's/BUILDFILE/${builddir}\/${BINARY}.out/' stm32flash.script > _stm32flash.script
	@echo "script _stm32flash.script" | nc localhost 4444

debug: $(BINARY)
	@${GDB} ${builddir}/${BINARY}.elf -x debug.gdb
	
config-header:
	@echo "* Generating config header ${hfile}.."
	@echo "// Auto generated file, do not tamper" > ${hfile}
	@echo "#ifdef INCLUDE_CONFIG_HEADER" >> ${hfile}
	@echo "#ifndef _CONFIG_HEADER_H" >> ${hfile}
	@echo "#define _CONFIG_HEADER_H" >> ${hfile}
	@sed -nr 's/([^ \t]*)?[ \t]*=[ \t]*1/#define \1/p' config.mk >> ${hfile}
	@echo "#endif" >> ${hfile}
	@echo "#endif" >> ${hfile}

build-info:
	@echo "*** INCLUDE PATHS"
	@echo "${INC}"
	@echo "*** SOURCE PATHS"
	@echo "${CPATH}"
	@echo "*** ASSEMBLY PATHS"
	@echo "${SPATH}"
	@echo "*** SOURCE FILES"
	@echo "${CFILES}"
	@echo "*** ASSEMBLY FILES"
	@echo "${SFILES}"
	@echo "*** FLAGS"
	@echo "${FLAGS}"
	
	
############
#
# Build info
#
############

BUILD_NUMBER_LDFLAGS  = --defsym __BUILD_DATE=$$(date +'%Y%m%d')
BUILD_NUMBER_LDFLAGS += --defsym __BUILD_NUMBER=$$(cat $(BUILD_NUMBER_FILE))
