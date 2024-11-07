CFLAGS = -DTEST
OBJS = clock.o console.o font-7x12.o gfx.o lcd-spi.o

BINARY = seismograph

LDSCRIPT = ./stm32f429i-discovery.ld

#Incluir headers en el directorio include
DEFS += -I./include
OPENCM3_DIR = ./libopencm3

include ./Makefile.include
