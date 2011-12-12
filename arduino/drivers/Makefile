DEVICE=atmega2560
CFLAGS=-mmcu=$(DEVICE) -Wall -Werror -O
LDFLAGS=-mmcu=$(DEVICE)
ASFLAGS=-mmcu=$(DEVICE)
CXXFLAGS=$(CFLAGS)

LDLIBS=-lm

include ../Makefile.avr

DRIVERS=adc.c bump.c power.c pwm.c serial.c serial-interrupt.c servo.c

all: $(DRIVERS:.c=.o) compass.o

#libdrivers.o: $(DRIVERS:.c=.o) compass.o
#	$(LD) -r $(LDFLAGS) -o $@ $^
#	$(AR) rcs libdrivers.a $^

include ../Makefile.implicit

include $(DRIVERS:%.c=.%.mk)

clean:
	rm $(DRIVERS:.c=.o) $(DRIVERS:%.c=.%.mk) compass.o || true
