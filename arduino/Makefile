DEVICE=atmega2560
CFLAGS=-mmcu=$(DEVICE) -Wall -Werror -O -save-temps
LDFLAGS=-mmcu=$(DEVICE) -lm
ASFLAGS=-mmcu=$(DEVICE)
CXXFLAGS=$(CFLAGS)

CC=avr-gcc 
CPP=avr-gcc -E
CXX=avr-g++ 
AS=avr-as

LD=avr-gcc

# include implicit rules for arduino
include Makefile.implicit

# include computer-specific file defining programmer port
include Makefile.device

TRG=main

all: $(TRG).hex

main.elf: main.o pwm.o motor.o serial.o power.o adc.o system.o servo.o gps.o sonar.o comm.o wheelmon.o speedman.o protocol.o compass.o bump.o

protocol.o: protocol.cpp protocol.h

program: $(TRG).hex
	avrdude -pm2560 -cusbtiny -u -U flash:w:$(TRG).hex
#	avrdude -pm2560 -P${COM} -cstk500v2 -u -U flash:w:$(TRG).hex
#  no need to specify baud rate with new Arduio UNO/Mega 2560 programmer

size: $(TRG).elf
	avr-size $(TRG).elf
