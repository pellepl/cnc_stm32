BINARY = cnc_stm32

############
#
# STM Device settings
#
############

FLAGS =  -DSTM32F10X_HD
#FLAGS += -DUSE_FULL_ASSERT
FLAGS += -DSYSCLK_FREQ_72MHz=72000000
STARTUP = startup_stm32f10x_hd.s

FLAGS += -DUSER_HARDFAULT=1

############
#
# Paths
#
############

sourcedir = src
builddir = build

#basetoolsdir = /home/petera/toolchain/gcc/arm-elf-tools-4.8.0
basetoolsdir = /home/petera/toolchain/gcc/arm-elf-tools-4.7.1
#/usr/local/gcc/arm-elf-tools-4.6.3
#/home/petera/toolchain/gcc/arm-elf-tools-4.6.2
codir = ${basetoolsdir}/lib/gcc/arm-none-eabi/4.7.1/
stmlibdir = STM32F10x_StdPeriph_Lib_V3.5.0/Libraries
stmdriverdir = ${stmlibdir}/STM32F10x_StdPeriph_Driver
stmcmsisdir = ${stmlibdir}/CMSIS/CM3/DeviceSupport/ST/STM32F10x
stmcmsisdircore = ${stmlibdir}/CMSIS/CM3/CoreSupport
tools = ${basetoolsdir}/bin

comm_dir = ../generic/comm
tinyheap_dir = ../generic/tinyheap
enc28j60_dir = ../generic/enc28j60_stm32

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

INCLUDE_DIRECTIVES = -I./${sourcedir} -I./${codir} -I./${stmdriverdir}/inc -I./${stmcmsisdir} -I./${stmcmsisdircore} -I./${comm_dir}/src -I./${tinyheap_dir}/src -I./${enc28j60_dir}/src
COMPILEROPTIONS = $(INCLUDE_DIRECTIVES) $(FLAGS) -mcpu=cortex-m3 -mno-thumb-interwork -mthumb -Wall -gdwarf-2
#-ffunction-sections -fdata-sections
COMPILEROPTIONS += -O3 
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

SFILES = 	$(STARTUP) \
			stm32f10x_it_h.s \
			memcpy.s \
			memset.s \
			variadic.s \
			sqrt.s \
			svc_handler.s

# app files
FILES = 	processor.c \
			shared_mem.c \
			main.c \
			bootloader_exec.c \
			system.c \
			led.c \
			nvstorage.c \
			config.c \
			uart.c \
			spi_driver.c \
			spi_dev.c \
			spi_flash.c \
			spi_flash_m25p16.c \
			spi_dev_os_generic.c \
			timer.c \
			cnc_control.c \
			cnc_comm.c \
			comm_impl.c \
			miniutils.c \
			taskq.c \
			heap.c \
			console.c \
			list.c \
			os.c \
			enc28j60_spi_eth.c \
			crc.c \
			comm_file.c
			
# comm files
include ${comm_dir}/files.mk

# tinyheap files
include ${tinyheap_dir}/files.mk
		
# enc28j60 driver files
include ${enc28j60_dir}/files.mk
		
# stm32 lib files
FILES += 	misc.c \
		stm32f10x_adc.c \
		stm32f10x_bkp.c \
		stm32f10x_can.c \
		stm32f10x_cec.c \
		stm32f10x_crc.c \
		stm32f10x_dac.c \
		stm32f10x_dbgmcu.c \
		stm32f10x_dma.c \
		stm32f10x_exti.c \
		stm32f10x_flash.c \
		stm32f10x_fsmc.c \
		stm32f10x_gpio.c \
		stm32f10x_i2c.c \
		stm32f10x_iwdg.c \
		stm32f10x_pwr.c \
		stm32f10x_rcc.c \
		stm32f10x_rtc.c \
		stm32f10x_sdio.c \
		stm32f10x_spi.c \
		stm32f10x_tim.c \
		stm32f10x_usart.c \
		stm32f10x_wwdg.c
		
# cmsis files
FILES += 	system_stm32f10x.c \
		core_cm3.c


# stm32 system
FILES +=	stm32f10x_it.c

# bootloader files
RFILES =	bootloader.c \
			bootloader_hal.c

LIBS = 

BINARYEXT = .hex

############
#
# Tasks
#
############

vpath %.c ${sourcedir} ${stmdriverdir}/src ${stmcmsisdir} ${stmcmsisdircore} ${comm_dir}/src ${tinyheap_dir}/src ${enc28j60_dir}/src
vpath %.s ${sourcedir} ${stmdriverdir}/src ${stmcmsisdir} ${stmcmsisdir}/startup/gcc_ride7

SOBJFILES = $(SFILES:%.s=${builddir}/%.o)
OBJFILES = $(FILES:%.c=${builddir}/%.o)
ROBJFILES = $(RFILES:%.c=${builddir}/%.o)

DEPFILES = $(FILES:%.c=${builddir}/%.d)
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

# compile relocatable c files
$(ROBJFILES) : ${builddir}/%.o:%.c
		@echo "... relocatable compile $@"
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
	
############
#
# Build info
#
############

BUILD_NUMBER_LDFLAGS  = --defsym __BUILD_DATE=$$(date +'%Y%m%d')
BUILD_NUMBER_LDFLAGS += --defsym __BUILD_NUMBER=$$(cat $(BUILD_NUMBER_FILE))
