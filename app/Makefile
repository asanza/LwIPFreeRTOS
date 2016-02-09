# makefile
# (c) 2016 Diego F. Asanza

# Imported source files and paths
# CHIBIOS = ../../..
# Startup files.

SOURCE= src
HWDIR= hw

INCDIR= -Iinc \
				-I../freertos/include \
				-I../plib/inc \
				-I../cmsis/Core/CM3

LDSCRIPT= gcc-flash-arm.ld

include ../common.mk
CFLAGS += $(INCDIR)
CFLAGS += -L../freertos -L../cmsis -L../plib
CFLAGS +=  -lstartup  -lplib  -lfreertos

SRCS:= $(wildcard $(SOURCE)/*.c)
OBJS:= $(patsubst %.c,%.o,$(SRCS))

app.elf: $(OBJS) #order matters!
	$(LD)  -o app.elf $(OBJS) $(CFLAGS) -T $(LDSCRIPT)

%o: %.c
	$(CC) -c $(CFLAGS) $< -o $@


clean:
	-rm $(OBJS) app.elf
