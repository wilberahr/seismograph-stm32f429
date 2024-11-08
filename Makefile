CFLAGS = -DTEST
OBJS = clock.o console.o 
OBJS += gfx.o lcd-spi.o sdram.o font-7x12.o

BINARY = seismograph

LDLIBS += -lm
LDSCRIPT = ./stm32f429i-discovery.ld

#Incluir headers en el directorio include
DEFS += -I./include
OPENCM3_DIR = ./libopencm3

include ./Makefile.include
