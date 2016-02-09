# makefile
# (c) 2016 Diego F. Asanza

# Imported source files and paths
# CHIBIOS = ../../..
# Startup files.

INCDIR= -Iinclude
SOURCE= .
PORTABLE= portable/GCC/ARM_CM3
MEMMANG= portable/MemMang

include ../common.mk
CFLAGS +=  -c $(INCDIR)

SRCS:= $(wildcard $(SOURCE)/*.c) \
	 			$(wildcard $(PORTABLE)/*.c) \
				$(wildcard $(MEMMANG)/*.c)

OBJS:= $(patsubst %.c,%.o,$(SRCS))

libfreertos.a: $(OBJS)
	$(AR) rcs libfreertos.a $(OBJS)

%o: %.c
	$(CC) -c $(CFLAGS) $< -o $@


clean:
	-rm $(OBJS) libfreertos.a
