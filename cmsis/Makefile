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

include ../common.mk
CFLAGS +=  -c

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
