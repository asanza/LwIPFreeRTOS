# makefile
# (c) 2016 Diego F. Asanza

# Imported source files and paths
# CHIBIOS = ../../..
# Startup files.

CMSIS= ../cmsis
SOURCE= src

INCDIR= -Iinc -I$(CMSIS)/Core/CM3

include ../common.mk
CFLAGS +=  -c $(INCDIR)

SRCS:= $(wildcard $(SOURCE)/*.c)
OBJS:= $(patsubst %.c,%.o,$(SRCS))

libplib.a: $(OBJS)
	$(AR) rcs $@

%o: %.c
	$(CC) -c $(CFLAGS) $< -o $@


clean:
	-rm $(OBJS) libplib.a
