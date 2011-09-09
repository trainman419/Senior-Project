DEVICE=atmega2560
CFLAGS=-mmcu=$(DEVICE) -Wall -Werror -save-temps
LDFLAGS=-mmcu=$(DEVICE) -lm
ASFLAGS=-mmcu=$(DEVICE)
CXXFLAGS=$(CFLAGS)

CC=avr-gcc 
CPP=avr-gcc -E
CXX=avr-g++ 
AS=avr-as

LD=avr-gcc

# include implicit rules for arduino
include ../Makefile.implicit

# include computer-specific file defining programmer port
include ../Makefile.device

all: TinyGPS.o
	avr-size TinyGPS.o

clean:
	rm TinyGPS.o
