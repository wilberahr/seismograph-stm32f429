CFLAGS = -DTEST

# TODO: definir los obj de dependencias en directorio /lib
#OBJS = clock.o console.o

# Se agregan la direccion del directorio donde estan los ejemplos
EXAMPLES_DIR = ./lib/libopencm3-examples/examples

# Se define el la placa de desarrollo que se va utilizar los ejemplos
BOARD_NAME = stm32/f4

# Se define la variante del modelo de stm32f429i
MODEL = stm32f429i-discovery
OPENCM3_DIR = ./lib/libopencm3-examples/libopencm3

# TODO: definir los ejemplos a utilizar
# EXAMPLES = lcd-serial spi

# Se define de donde se va a obtener .o a partir de los ejemplos
# TODO: definir para cada archivo excepto los archivos con el main
# OBJS += $(EXAMPLES_DIR)/$(BOARD_NAME)/$(MODEL)/lcd-serial/*.o
# OBJS += $(EXAMPLES_DIR)/$(BOARD_NAME)/$(MODEL)/spi/*.o

# Nombre del binario a compilar
BINARY = seismograph

# LDSCRIPT tomado de los ejemplos
LDSCRIPT = ./stm32f429i-discovery.ld

# Include del Makefile.include que a su vez llama las reglas del make, tomado de los ejemplos
include ./Makefile.include




	