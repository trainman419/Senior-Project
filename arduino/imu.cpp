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

extern "C" {
#include "i2c.h"
}

#include "ros.h"
#include <geometry_msgs/Vector3.h>


#define I2C_ACCEL 0x1D
#define I2C_COMPASS 0x3C

geometry_msgs::Vector3 compass_msg;
geometry_msgs::Vector3 accel_msg;
geometry_msgs::Vector3 gyro_msg;

ros::Publisher compass_pub("compass", &compass_msg);
ros::Publisher accel_pub("accel", &accel_msg);
ros::Publisher gyro_pub("gyro", &gyro_msg);

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
   // interpret and publish data
   int16_t * compass = (int16_t*)buf;
   compass_msg.x = compass[0]/1300.0;
   compass_msg.y = compass[1]/1300.0;
   compass_msg.z = compass[2]/1300.0;
   compass_pub.publish(&compass_msg);
}

// read the compass
void compass_read() {
   i2c_read(I2C_COMPASS, 0x03, compass_buf, 6, compass_done);
}

uint8_t accel_buf[6];

void accel_done(uint8_t * buf) {
   // interpret and publish data
   int16_t accel;
   accel = buf[0] | (buf[1] << 8);
   accel_msg.x = accel / 256.0;

   accel = buf[2] | (buf[3] << 8);
   accel_msg.y = accel / 256.0;

   accel = buf[4] | (buf[5] << 8);
   accel_msg.z = accel / 256.0;
   accel_pub.publish(&accel_msg);

   compass_read();
}

// read the accelerometer
void accel_read() {
   i2c_read(I2C_ACCEL, 0x32, accel_buf, 6, accel_done);
}

// read the gyro
void gyro_read() {
   accel_read();
   // TODO: write this
}

/* Kalman filter notes:
 *
 * Fill this out when I actually write a kalman filter
 */ 

void imu_read() {
   gyro_read();
}
