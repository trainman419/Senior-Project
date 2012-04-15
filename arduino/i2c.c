/*
 * i2c.c
 * An interrupt-driven I2C/TWI library
 *
 * Author: Austin Hendrix
 */
#include <stdint.h>

#include <avr/io.h>
#include <avr/interrupt.h>

#include "drivers/led.h"

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

// interface code is at the bottom of the file.

// state machine variables.
//
// state machine mode
volatile enum { READ = 1, WRITE = 0 } i2c_mode; 

uint8_t i2c_state = 0;
uint8_t i2c_twsr = 0;

uint8_t i2c_address;  // i2c slave address
uint8_t i2c_register; // i2c slave register
uint8_t * i2c_data;   // data to read/write from slave
uint8_t i2c_data_sz;  // size of data to write
uint8_t i2c_data_pos; // position in incoming data buffer

uint8_t i2c_errflag;
volatile uint8_t i2c_ready;

// function pointer to use on data read completion
void (* i2c_w_callback)(void) = 0;
void (* i2c_r_callback)(uint8_t*) = 0;

// i2c state machine functions
void i2c_none();
void i2cf_read();
void i2cf_address();
void i2cf_raddress();
void i2cf_wstop();
void i2cf_wdata();
void i2cf_rstart();
void i2cf_register();

void (*i2c_next)(void) = i2c_none;

// the magic ISR that makes all of this go round
ISR(TWI_vect) {
   //led_on();
   // disable TWI interrupt and enable general interrupts.
   //TWCR &= ~((1 << TWINT) | (1 << TWIE));
   //sei();

   i2c_twsr = TWSR;


   /*
   if( i2c_next ) {
      i2c_next();
   }
   */
   // validate i2c_next function pointer
   if( i2c_none == i2c_next) {
      i2c_none();
   } else if( i2cf_read == i2c_next) {
      i2cf_read();
   } else if( i2cf_address == i2c_next) {
      i2cf_address();
   } else if( i2cf_raddress == i2c_next) {
      i2cf_raddress();
   } else if( i2cf_wstop == i2c_next) {
      i2cf_wstop();
   } else if( i2cf_wdata == i2c_next) {
      i2cf_wdata();
   } else if( i2cf_rstart == i2c_next) {
      i2cf_rstart();
   } else if( i2cf_register == i2c_next) {
      i2cf_register();
   } else {
      led_on();
   }


   // re-enable TWI interrupts
   //TWCR = (TWCR | (1 << TWIE)) & ~(1 << TWINT);
   //if( ! i2c_errflag ) led_off();
   //led_off();
   /*
   if( TWCR & (1 << TWINT) ) {
      TWCR = TWCR; // reset interrupt flag if set
   }
   */
}

// do nothing
void i2c_none() {
   i2c_next = i2c_none;
   i2c_state = 1;
   TWCR = (1 << TWINT); // reset interrupt flag if set
   i2c_ready = 1;
}

// error
void i2c_err() {
   i2c_next = i2c_none;
   i2c_state = 2;
   i2c_errflag = 1;
   i2c_ready = 1;
}

void i2cf_address();

void i2cf_read() {
   if( (TWSR & 0xF8) == 0x40 ) {
      // if we sent an address, wait for data
      --i2c_data_sz;
      if( i2c_data_sz > 0 ) {
         // we want more data; transmit ack
         TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA) | (1<<TWIE);
      } else {
         // the next byte will be the last
         TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);
      }
      i2c_next = i2cf_read;
      i2c_state = 3;
   } else if( (TWSR & 0xF8) == 0x50 ) {
      // if we received data, wait for more data or nak
      i2c_data[i2c_data_pos++] = TWDR;
      --i2c_data_sz;
      if( i2c_data_sz > 0 ) {
         // we want more data; transmit ack
         TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA) | (1<<TWIE);
      } else {
         // the next byte will be the last
         TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);
      }
      i2c_next = i2cf_read;
      i2c_state = 4;
   } else if( (TWSR & 0xF8) == 0x58 ) {
      // we got our last byte and sent nak
      i2c_data[i2c_data_pos] = TWDR;
      TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO) | (1<<TWIE);
      i2c_ready = 1;
      i2c_next = i2c_none;
      i2c_state = 5;
      sei();
      if(i2c_r_callback) {
         void (*tmp_callback)(uint8_t*) = i2c_r_callback;
         i2c_r_callback = 0;
         tmp_callback(i2c_data);
      } else {
         i2c_err();
      }
   } else if( (TWSR & 0xF8) == 0x08 ) {
      // Start sent. shouldn't happen
      //led_on();
      i2c_err();
   } else if( (TWSR & 0xF8) == 0x10 ) {
      // repeated start sent. shouldn't happen
      //led_on();
      i2c_err();
   } else if( (TWSR & 0xF8) == 0x38 ) {
      // arbitration lost or NAK
      //led_on();
      i2c_err();
   } else if( (TWSR & 0xF8) == 0x48 ) {
      // SLA+R sent; NAK received
      // generate a stop condition
      //TWCR = (1<<TWSTA) | (1<<TWSTO) | (1<<TWINT);
      //i2c_next = i2cf_address;
      i2c_err();
      i2c_state = 6;
   } else {
      //led_on();
      i2c_err();
   }
}

// send i2c read address
void i2cf_raddress() {
   if( (TWSR & 0xF8) == 0x10 ) {
      TWDR = i2c_address | 1;
      TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);
      i2c_next = i2cf_read;
      i2c_state = 7;
   } else {
      i2c_err();
   }
}

// send repeated start condition
void i2cf_rstart() {
   if( (TWSR & 0xF8) == 0x28 ) {
      TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTA) | (1<<TWIE);
      i2c_next = i2cf_raddress;
      i2c_state = 8;
   } else {
      i2c_err();
   }
}

// send stop condition
void i2cf_wstop() {
   TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO) | (1<<TWIE);
   i2c_ready = 1;
   i2c_next = i2c_none;
   i2c_state = 9;
   sei();
   if(i2c_w_callback) {
      void (*tmp_cb)(void) = i2c_w_callback;
      i2c_w_callback = 0;
      tmp_cb();
   } 
}

// write i2c data
void i2cf_wdata() {
   if( (TWSR & 0xF8) == 0x28 ) {
      TWDR = *i2c_data;
      ++i2c_data;
      --i2c_data_sz;
      if( i2c_data_sz == 0 ) {
         i2c_next = i2cf_wstop;
         i2c_state = 10;
      }
      TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);
   } else {
      i2c_err();
   }
}

// write i2c register
void i2cf_register() {
   if( (TWSR & 0xF8) == 0x18 ) {
      TWDR = i2c_register;
      TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);
      if( i2c_mode == WRITE ) {
         i2c_next = i2cf_wdata;
         i2c_state = 11;
      } else {
         i2c_next = i2cf_rstart;
         i2c_state = 12;
      }
   } else {
      i2c_err();
   }
}

// write i2c address
void i2cf_address() {
   if( (TWSR & 0xF8) == 0x08 ) {
      TWDR = i2c_address;
      TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);
      i2c_next = i2cf_register;
      i2c_state = 13;
   } else {
      i2c_err();
   }
}

void i2c_init() {
   // set up clock prescaler for 400kHz operation

   // bit rate equation:
   // rate = CPU / (16 + (2 * TWBR * prescaler))
   // 400000 = 16000000 / (16 + (2 * 3 * 4))
   TWBR = 3; // 400kHz
   //TWBR = 18; // 100kHz
   TWSR = 1; // prescaler /4

   // set up I/O pins
   DDRD &= ~(3);
   PORTD &= ~(3);

   TWCR = 0;

   i2c_next = i2c_none;
   i2c_state = 14;
   i2c_ready = 1;
}

// overall bus functions should wait for TWSTO to become unset before writing
// to TWCR

// write a single byte to a register in an I2C device
void i2c_write(uint8_t addr, uint8_t reg, uint8_t data) {
   while(!i2c_ready); // wait for I2C to become ready
   while(TWCR & (1<<TWSTO) ); // wait for stop bit to become clear
   i2c_mode = WRITE;
   i2c_address = addr;
   i2c_register = reg;

   // use i2c_data_pos as temporary
   i2c_data_pos = data;
   i2c_data = &i2c_data_pos;
   i2c_data_sz = 1;

   //led_off();
   i2c_errflag = 0;
   i2c_ready = 0;

   i2c_next = i2cf_address;
   i2c_state = 15;

   // start I2C mode by sending start bit
   TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTA) | (1<<TWIE);
}

// write multiple bytes and call a callback when done
void i2c_writem( uint8_t addr, uint8_t reg, uint8_t * data, uint8_t size, 
      void(*cb)(void)) {
   while(!i2c_ready); // wait for I2C to become ready
   while(TWCR & (1<<TWSTO) ); // wait for stop bit to become clear
   i2c_mode = WRITE;
   i2c_address = addr;
   i2c_register = reg;
   i2c_data = data;
   i2c_data_sz = size;
   i2c_w_callback = cb;

   //led_off();
   i2c_errflag = 0;
   i2c_ready = 0;

   i2c_next = i2cf_address;
   i2c_state = 16;

   // start I2C mode by sending start bit
   TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTA) | (1<<TWIE);
}

uint8_t i2c_fail = 0;

// read bytes from an I2C device into a buffer and call a callback when done
int8_t i2c_read(uint8_t addr, uint8_t reg, uint8_t * buf, uint8_t size, 
      void(*cb)(uint8_t *)) {
   uint16_t timeout = 0x0FFF;
   // wait for I2C to become ready
   while(!i2c_ready && timeout > 0) {
      --timeout;
   }
   if(!i2c_ready) {
      led_on();
      ++i2c_fail;
      if( i2c_fail > 20 ) {
         i2c_fail = 0;
         i2c_ready = 1;
         return -1;
      } else {
         return 0;
      }
   }
   led_off();
   while(TWCR & (1<<TWSTO) ); // wait for stop bit to become clear
   i2c_mode = READ;
   i2c_address = addr;
   i2c_register = reg;
   i2c_data = buf;
   i2c_data_sz = size;
   i2c_data_pos = 0;
   i2c_r_callback = cb;

   //led_off();
   i2c_errflag = 0;
   i2c_ready = 0;

   i2c_next = i2cf_address;
   i2c_state = 17;

   // start I2C mode by sending start bit
   TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTA) | (1<<TWIE);
   return 0;
}

