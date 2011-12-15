VPATH=../drivers:../ros_lib
DEVICE=atmega2560
CFLAGS=-mmcu=$(DEVICE) -O -Wall -Werror -I../ros_lib -I..
LDFLAGS=-mmcu=$(DEVICE)
ASFLAGS=-mmcu=$(DEVICE)
CXXFLAGS=$(CFLAGS)

SER=serial.o serial-interrupt.o
include ../Makefile.avr
include ../Makefile.implicit

.PHONY: all
all: serial_test.hex locking.hex burst_rx.hex burst_tx.hex

../drivers/libdrivers.o:
	$(MAKE) -C ../drivers

include .serial_test.mk
serial_test.elf: serial_test.o $(SER) servo.o

include .locking.mk
locking.elf: locking.o $(SER)

include .burst_rx.mk
burst_rx.elf: burst_rx.o $(SER)

include .burst_tx.mk
burst_tx.elf: burst_tx.o $(SER)

include .ros_test.mk
ros_test.elf: ros_test.o $(SER)
