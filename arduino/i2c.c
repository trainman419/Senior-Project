/*
 * i2c.c
 * An interrupt-driven I2C/TWI library
 *
 * Author: Austin Hendrix
 */
#include <stdint.h>

#include <avr/io.h>
#include <avr/interrupt.h>

/* design notes:
 * This library is designed to take the waiting out of dealing with I2C devices
 * In particular, it's aimed at the types of transactions that are needed to
 * interface with the devices on Sparkfun's 9DOF sensor stick.
 *
 * ADXL345 3-axis accelerometer:
 *    single-byte write:
 *       start, address, register, data, stop
 *    multibyte write:
 *       start, address, register, data[xN], stop
 *    single-byte read:
 *       start, address, register, (re)start, address(r), data, nack, stop
 *    multibyte read:
 *       start, address, register, (re)start, address(r), data[xN], nack, stop
 *
 * HMC5843 3-axis compass:
 *    I2C compliant. probably the same as ADXL345
 *
 * ITG-3200 3-axis accelerometer:
 *    same as ADXL345.
 *
 * Timing thoughts
 *    a multibyte read of 6 bytes will be 6*9 + 3*9 + 3 = 84 bits
 *    at 400khz this will take 3360 clock cycles or 210 us (bit-shifting)
 *    under 1 ms to read all three sensors in sequence. FAST!
 *    if we write and interrupt library, we'll be exceuting an interrupt for 2
 *       of every 9 bits we send. roughly 64 cycles per interrupt.
 *       at 16MHz, 400khz is 40 cycles per bit.
 *
 * Existing libraries implement master and slave functionality. we only need
 *  master functionality, and can trim a few bytes off our program size and
 *  memory footprint at the expense of more bugs and longer developent time
 *
 * Addresses:
 *    input addresses to function calls are 7 bits, left-aligned. the library
 *    functions override the LSB to provide the read/write address bit
 *
 * Read buffer:
 *    the read buffer must be supplied by the caller, and must be of adequate
 *    size. It will be passed to the callback when the read is complete.
 *
 * Callbacks:
 *    callbacks will be called when the associated read or write is complete.
 *    callbacks will be called with global interrupts enabled
 *    the I2C library will be ready for another I2C operation when the callback
 *       is called
 */

uint8_t i2c_idle;

void i2c_init() {
   // TODO: initialize stuff here
   i2c_idle = 0;
}

// the last thing that the I2C stack did.
volatile enum { IDLE, START, ADDR, REG, RSTART
} i2c_state;

// function pointer to use on data read completion
void (* i2c_w_callback)(uint8_t);
void (* i2c_r_callback)(uint8_t*);

// write a single byte to a register in an I2C device
void i2c_write(uint8_t addr, uint8_t reg, uint8_t data) {
   while(i2c_state != IDLE);
}

// write multiple bytes to a register in an I2C device and call a callback when
// done
void i2c_writem( uint8_t addr, uint8_t reg, uint8_t * data, uint8_t size, 
      void(*cb)(void)) {
   while(i2c_state != IDLE);
}

// read bytes from an I2C device into a buffer and call a callback when done
void i2c_read(uint8_t addr, uint8_t reg, uint8_t * buf, uint8_t size, 
      void(*cb)(uint8_t *)) {
   while(i2c_state != IDLE);
}

// the magic ISR that makes all of this go round
ISR(TWI_vect) {
   switch(i2c_state) {
      case START:
         // check TWSR to see that start was sent
         // send address (always?)
         break;
      case ADDR:
         // check that data sent and ack received
         // send register
         break;
      case REG:
         break;
      case RSTART:
         break;
   }
}
