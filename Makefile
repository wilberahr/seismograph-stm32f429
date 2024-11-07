CFLAGS = -DTEST

# TODO: definir los obj de dependencias en directorio /lib
OBJ_DIR = build
BIN_DIR = bin
SRC_DIR = scr
INC_DIR = include
LIB_DIR = lib

OBJS = $(OBJ_DIR)/clock.o $(OBJ_DIR)/console.o

# Se agregan la direccion del directorio donde estan los ejemplos
#EXAMPLES_DIR = ./lib/libopencm3-examples/examples

# Se define el la placa de desarrollo que se va utilizar los ejemplos
#BOARD_NAME = stm32/f4

# Se define la variante del modelo de stm32f429i
#MODEL = stm32f429i-discovery
OPENCM3_DIR = $(LIB_DIR)/libopencm3-examples/libopencm3

#EXAMPLES_NAME = lcd-serial spi
# TODO: definir los ejemplos a utilizar
#EXAMPLES = $(EXAMPLES_DIR)/$(BOARD_NAME)/$(MODEL)/$(EXAMPLES_NAME)

#OBJS += $(LIB_DIR)/$(EXAMPLES_NAME)/build/*.o


#lcd-serial : $(EXAMPLES_DIR)/$(BOARD_NAME)/$(MODEL)/lcd-serial/*.o
# OBJS += $(EXAMPLES_DIR)/$(BOARD_NAME)/$(MODEL)/lcd-serial/*.o
#spi : $(EXAMPLES_DIR)/$(BOARD_NAME)/$(MODEL)/spi/*.o
# OBJS += $(EXAMPLES_DIR)/$(BOARD_NAME)/$(MODEL)/spi/*.o

# Nombre del binario a compilar
BINARY = seismograph

# LDSCRIPT tomado de los ejemplos
LDSCRIPT = ./stm32f429i-discovery.ld

# Include del Makefile.include que a su vez llama las reglas del make, tomado de los ejemplos
include ./Makefile.include

# Se define de donde se va a obtener .o a partir de los ejemplos
# TODO: definir para cada archivo excepto los archivos con el main
#$(LIB_DIR)/$(EXAMPLES_NAME)/build/%.o: $(EXAMPLES)/%.c
#	$(MAKE) -c $(EXAMPLES)
#	mkdir -p $(LIB_DIR)/$(EXAMPLES_NAME)
#	mkdir -p $(LIB_DIR)/$(EXAMPLES_NAME)/include
#	mkdir -p $(LIB_DIR)/$(EXAMPLES_NAME)/build
#	cp -p $(EXAMPLES)/{*.o *.d} $(LIB_DIR)/$(EXAMPLES_NAME)/build/
#	cp -p $(EXAMPLES)/{*.h} $(LIB_DIR)/$(EXAMPLES_NAME)/include/ 
#	rm -f $(EXAMPLES)/*.o $(EXAMPLES)/*.d


	