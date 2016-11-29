#Makefile to compile simple AVR project

#Project Name
P_NAME = mcz33991_example

#Object file
OBJECT_FILE = $(P_NAME).o

# Binary hax file for flashing
FLASH_FILE = $(P_NAME).hex

# Output directory
PROJECT.BIN = bin
TEST.BIN = bin/test

#AVR CPU constants
DF_CPU = 16000000UL
MMCPU = atmega168

# source files
PROJECT_SRC = \
	MCZ33991.c \
	spi.c \
	example.c

init:
	@mkdir -p $(PROJECT.BIN)

objects: init $(PROJECT_SRC)
	avr-gcc -mmcu=$(MMCPU) -DF_CPU=$(DF_CPU) \
	-std=c11 \
	-Os -o $(PROJECT.BIN)/$(OBJECT_FILE) $(PROJECT_SRC)

hex: objects
	avr-objcopy -j .text -j .data -O ihex $(PROJECT.BIN)/$(OBJECT_FILE) \
	$(PROJECT.BIN)/$(FLASH_FILE)

flash_it: objects hex
	avrdude -v -p atmega168 -carduino -P/dev/ttyUSB0 -b 19200 -D -U \
	flash:w:$(PROJECT.BIN)/$(FLASH_FILE):i

clean:
	@rm -rf $(PROJECT.BIN)
	@rm -f *.o *.hex
