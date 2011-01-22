DEVICE=atmega2560
CFLAGS=-mmcu=$(DEVICE) -Wall -Werror -O3 -save-temps
LDFLAGS=-mmcu=$(DEVICE) -lm
ASFLAGS=-mmcu=$(DEVICE)

CC=avr-gcc 
CPP=avr-gcc -E
CXX=avr-g++ 

LD=avr-gcc

# include implicit rules for arduino
include Makefile.implicit

# include computer-specific file defining programmer port
include Makefile.device

TRG=main

all: $(TRG).hex

main.elf: main.o pwm.o motor.o new_serial.o power.o adc.o system.o servo.o gps.o sonar.o

download: $(TRG).hex
	avrdude -pm2560 -P${COM} -cstk500v2 -u -U flash:w:$(TRG).hex
#  no need to specify baud rate with new Arduio UNO/Mega 2560 programmer
