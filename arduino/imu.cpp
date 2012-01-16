/* imu.c
 *
 * sparkfun 9dof IMU driver.
 * built on custon I2C library
 *
 * Also does appropriate sensor fusion to produce heading and orientation
 *
 * Accelerometer data: 3.9mg per LSB; 256 LSB / g
 * Compass data: 1300 counts/milli-gauss
 *
 * Author: Austin Hendrix
 */

#include <stdint.h>

#include "i2c.h"

#define I2C_ACCEL 0x1D
#define I2C_COMPASS 0x3C

void imu_init() {
   i2c_init();
   // set up accelerometer
   i2c_write(I2C_ACCEL, 0x2A, 0x00); // disable tap detection
   i2c_write(I2C_ACCEL, 0x2C, 0x09); // 50Hz data rate
   i2c_write(I2C_ACCEL, 0x31, 0x0B); // full res mode & +/- 16g range
   i2c_write(I2C_ACCEL, 0x38, 0x00); // disable FIFO; always get recent sample
   i2c_write(I2C_ACCEL, 0x2D, 0x80); // enable measurement mode

   // set up compass
   i2c_write(I2C_COMPASS, 0x00, 0x18); // 50Hz mode
   i2c_write(I2C_COMPASS, 0x01, 0x20); // gain: 1300 counts/milli-gauss
   i2c_write(I2C_COMPASS, 0x02, 0x00); // continuous-conversion mode
}

uint8_t compass_buf[6];

void compass_done(uint8_t * buf) {
   // ummm... done? publish?
}

// read the compass
void compass_read() {
   i2c_read(I2C_COMPASS, 0x03, compass_buf, 6, compass_done);
}

uint8_t accel_buf[6];

void accel_done(uint8_t * buf) {
   // ummm... done? publish?
}

// read the accelerometer
void accel_read() {
   i2c_read(I2C_ACCEL, 0x32, accel_buf, 6, accel_done);
}

// read the gyro
void gyro_read() {
}

/* Kalman filter notes:
 *
 * Fill this out when I actually write a kalman filter
 */ 
