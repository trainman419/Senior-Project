DEVICE=atmega2560
CFLAGS=-mmcu=$(DEVICE) -Wall -Werror -O
LDFLAGS=-mmcu=$(DEVICE)
ASFLAGS=-mmcu=$(DEVICE)
CXXFLAGS=$(CFLAGS)

LDLIBS=-lm

CC=avr-gcc 
CPP=avr-gcc -E
CXX=avr-g++ 
AS=avr-as
AR=avr-ar
LD=avr-gcc

DRIVERS=adc.c bump.c power.c pwm.c serial.c serial-interrupt.c servo.c

all: libdrivers.a

libdrivers.a: $(DRIVERS:.c=.o)
	$(AR) rcs libdrivers.a $(DRIVERS:.c=.o)

%.mk: %.c
	$(CXX) -MM $^ -MF $@

include $(DRIVERS:.c=.mk)

clean:
	rm $(DRIVERS:.c=.o) $(DRIVERS:.c=.mk) || true
