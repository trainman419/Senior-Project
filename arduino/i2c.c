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
 *       of every 9 bits we send. roughly 64 cycles for interrupt overhead.
 *       at 16MHz, 400khz is 40 cycles per bit.
 *       at 16MHz, 100khz is 160 cycles.
 *    we can probably get 70% of our wait time back if we use interrupts
 *       70% * 16000 = 11200 cycles
 *
 *
 * Existing libraries implement master and slave functionality. we only need
 *  master functionality, and can trim a few bytes off our program size and
 *  memory footprint at the expense of more bugs and longer developent time
 *
 * The Wire/twi library from arduino and ARVlib both implement an interrupt
 *  driven I2C interface for AVR
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

volatile enum { IDLE, START, ADDR, REG, RSTART, DATA, STOP, RADDR, RDATA, NAK
} i2c_state; // the last thing that the I2C stack did.

void i2c_init() {
   // TODO: initialize stuff here
   i2c_idle = 0;
}

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

// state machine variables.
//
// state machine mode
volatile enum { READ = 1, WRITE = 0 } i2c_mode; 
uint8_t i2c_address;  // i2c slave address
uint8_t i2c_register; // i2c slave register
uint8_t * i2c_data;   // data to read/write from slave
uint8_t i2c_data_sz;  // size of data to write


// the magic ISR that makes all of this go round
ISR(TWI_vect) {
   switch(i2c_state) {
      case START:
         // check TWSR to see that start was sent
         if( (TWSR & 0xF8) == START ) {
            // send address
            // lowest bit set for read; unset for write
            TWDR = i2c_address | (i2c_mode & 1);
            TWCR = (1<<TWINT) | (1<<TWEN);
            i2c_state = ADDR;
         } else {
            // fail; return to idle state
            i2c_state = IDLE;
         }
         break;
      case ADDR:
         // check that data sent and ack received
         if( (TWSR & 0xF8) == MT_SLA_ACK ) {
            // send register
            TWDR = i2c_register;
            TWCR = (1<<TWINT) | (1<<TWEN);
            i2c_state = REG;
         } else {
            // fail; return to idle state
            i2c_state = IDLE;
         }
         break;
      case REG:
         // check that register was sent
         if( (TWSR & 0xF8) == MT_DATA_ACK ) {
            if( i2c_mode & 1 ) {
               // read mode
            } else {
               // write mode
               if( i2c_data_sz > 0 ) {
                  // if there's data in the buffer
                  TWDR = *i2c_data;
                  TWCR = (1<<TWINT) | (1<<TWEN);
                  i2c_data++;
                  i2c_data_sz--;
                  i2c_state = REG;
               } else {
                  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
                  i2c_state = STOP;
               }
            }
         } else {
            // fail; return to idle state
            i2c_state = IDLE;
         }
         break;
      case RSTART:
         break;
      case DATA:
         break;
      case STOP:
         break;
      case RADDR:
         break;
      case RDATA:
         break;
      case NAK:
         break;
      case IDLE:
         // if we're here, something is wrong
         break;
   }
}
