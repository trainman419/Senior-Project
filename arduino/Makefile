DEVICE=atmega2560
CFLAGS=-mmcu=$(DEVICE) -Wall -Werror -O3
LDFLAGS=-mmcu=$(DEVICE)

CC=avr-gcc 
CPP=avr-gcc -E
CXX=avr-g++ 

LD=avr-gcc

# include implicit rules for arduino
include Makefile.implicit

TRG=main

all: $(TRG).hex

main.elf: main.o pwm.o

download: $(TRG).hex
	avrdude -pm2560 -P/dev/tty.usbmodem411 -cstk500 -u -U flash:w:$(TRG).hex
# I'm a little worried that I don't need to specify a baud rate, but it works
