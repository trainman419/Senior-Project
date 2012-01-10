/* i2c.h
 *
 * custom I2C library based on function pointers. mostly becuase I'm crazy. 
 *  Of course, it _might_ be more efficient. It's hard to say at this point.
 *
 * Author: Austin Hendrix
 */

#ifndef I2C_H
#define I2C_H

#include <stdint.h>

void i2c_init();

// write a single byte to a register in an I2C device
void i2c_write(uint8_t addr, uint8_t reg, uint8_t data);

// write multiple bytes and call a callback when done
void i2c_writem( uint8_t addr, uint8_t reg, uint8_t * data, uint8_t size, 
      void(*cb)(void));

// read bytes from an I2C device into a buffer and call a callback when done
void i2c_read(uint8_t addr, uint8_t reg, uint8_t * buf, uint8_t size, 
      void(*cb)(uint8_t *));

#endif
