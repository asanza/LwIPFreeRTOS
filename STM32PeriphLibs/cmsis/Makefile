# makefile
# (c) 2016 Diego F. Asanza

# Imported source files and paths
# CHIBIOS = ../../..
# Startup files.

CORE= Core/CM3
STARTUP= $(CORE)/startup/gcc

ASMSRC = $(STARTUP)

INCDIR = $(CORE)

##############################################################################
# Compiler settings
#

MCU  = cortex-m3

TRGT = arm-none-eabi-
CC   = $(TRGT)gcc
CPPC = $(TRGT)g++
LD   = $(TRGT)gcc
CP   = $(TRGT)objcopy
AS   = $(TRGT)as
AR   = $(TRGT)ar
OD   = $(TRGT)objdump
SZ   = $(TRGT)size
HEX  = $(CP) -O ihex
BIN  = $(CP) -O binary
CFLAGS +=  -c -mcpu=$(MCU) -mthumb

# Define C warning options here
CWARN = -Wall -Wextra -Wundef -Wstrict-prototypes

CSRC:= $(wildcard $(CORE)/*.c) 
COBJ:= $(patsubst %.c,%.o,$(CSRC))

ASMSRC:= $(wildcard $(STARTUP)/*cl.s)
ASMOBJ:= $(patsubst %.s,%.o,$(ASMSRC))

libstartup.a: $(COBJ) $(ASMOBJ)
	$(AR) rcs $@

%o: %.c
	$(CC) -c $(CFLAGS) $< -o $@

%o: %s
	$(AS) $(CFLAGS) $< -o $@

clean:
	-rm $(COBJ) $(ASMOBJ) libstartup.a