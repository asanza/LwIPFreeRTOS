# Common makefile definitions

### Build flags for all targets
CFLAGS	= -g -Wall -Wextra -Wundef #-Wstrict-prototypes

# Build tools
MCU  = cortex-m3

TRGT	= arm-none-eabi-
CC		= $(TRGT)gcc
CPPC	= $(TRGT)g++
LD		= $(TRGT)gcc
CP		= $(TRGT)objcopy
AS		= $(TRGT)as
AR		= $(TRGT)ar
OD		= $(TRGT)objdump
SZ		= $(TRGT)size
HEX		= $(CP) -O ihex
BIN		= $(CP) -O binary

CFLAGS += -mcpu=$(MCU) -mthumb
