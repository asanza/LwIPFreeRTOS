#
#
#

### Subdirectories
SOURCEDIR = .
INCLUDES = -Iinclude \
						-Iport \
						-Iinclude/ipv4 \
						-I../freertos/include \
						-I../plib/inc \
						-I../cmsis/Core/CM3

SOURCES := $(shell find $(SOURCEDIR) -name '*.c')

include ../common.mk

CFLAGS += -c $(INCLUDES)

OBJS:=$(patsubst %.c,%.o,$(SOURCES))

liblwip.a: $(OBJS)
	$(AR) rcs liblwip.a $(OBJS)

%o: %c
	$(CC) -c $(CFLAGS) $< -o $@

clean:
	-rm $(OBJS) liblwip.a
