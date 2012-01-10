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


#define I2C_ACCEL 0xA6
#define I2C_COMPASS 0x3C
#define I2C_GYRO 0xD0

#define X 0
#define Y 1
#define Z 2

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
   i2c_write(I2C_ACCEL, 0x2D, 0x08); // enable measurement mode

   // set up compass
   i2c_write(I2C_COMPASS, 0x00, 0x18); // 50Hz mode
   i2c_write(I2C_COMPASS, 0x01, 0x20); // gain: 1300 counts/milli-gauss
   i2c_write(I2C_COMPASS, 0x02, 0x00); // continuous-conversion mode

   // set up gyro
   i2c_write(I2C_GYRO, 0x15, 19); // 50Hz sample rate
   i2c_write(I2C_GYRO, 0x16, 0x1C); // 20Hz low-pass filter
   i2c_write(I2C_GYRO, 0x3E, 0x01); // X gyro as clock reference
}

uint8_t common_buf[8];

// read compass, accelerometer and gyro and produce pose estimates
void update_imu() {
}

int16_t gyro_zero[3];
// number of gyro samples to take at startup
#define GYRO_COUNT 50
uint8_t gyro_start = GYRO_COUNT;
void gyro_done(uint8_t * buf) {
   int16_t gyro;
   gyro = (buf[0] << 8) | buf[1];
   if( gyro_start > 0 ) {
      gyro_zero[X] += gyro;
   } else {
      gyro -= gyro_zero[X];
   }
   gyro_msg.x = gyro;

   gyro = (buf[2] << 8) | buf[3];
   if( gyro_start > 0 ) {
      gyro_zero[Y] += gyro;
   } else {
      gyro -= gyro_zero[Y];
   }
   gyro_msg.y = gyro;

   gyro = (buf[4] << 8) | buf[5];
   if( gyro_start > 0 ) {
      gyro_zero[Z] += gyro;
   } else {
      gyro -= gyro_zero[Z];
   }
   gyro_msg.z = gyro;

   if( gyro_start > 0 ) {
      --gyro_start;
      if( gyro_start == 0 ) {
         gyro_zero[X] /= GYRO_COUNT;
         gyro_zero[Y] /= GYRO_COUNT;
         gyro_zero[Z] /= GYRO_COUNT;
      }
   }

   gyro_pub.publish(&gyro_msg);
}
// read the gyro
void gyro_read() {
   // TODO: read temperature sensor
   i2c_read(I2C_GYRO, 0x1D, common_buf, 6, &gyro_done);
}

void compass_done(uint8_t * buf) {
   // interpret and publish data
   int16_t compass = (buf[0] << 8) | buf[1];
   compass_msg.x = ((compass/1300.0) + compass_msg.x) / 2;

   compass = (buf[2] << 8) | buf[3];
   compass_msg.y = ((compass/1300.0) + compass_msg.y) / 2;

   compass = (buf[4] << 8) | buf[5];
   compass_msg.z = ((compass/1300.0) + compass_msg.z) / 2;
   compass_pub.publish(&compass_msg);

   gyro_read();
}

// read the compass
void compass_read() {
   i2c_read(I2C_COMPASS, 0x03, common_buf, 6, compass_done);
}

void accel_done(uint8_t * buf) {
   // interpret and publish data
   int16_t accel;
   accel = buf[0] | (buf[1] << 8);
   accel_msg.x = ((accel / 256.0) + 3*accel_msg.x) / 4;

   accel = buf[2] | (buf[3] << 8);
   accel_msg.y = ((accel / 256.0) + 3*accel_msg.y) / 4;

   accel = buf[4] | (buf[5] << 8);
   accel_msg.z = ((accel / 256.0) + 3*accel_msg.z) / 4;
   accel_pub.publish(&accel_msg);

   compass_read();
}

// read the accelerometer
void accel_read() {
   i2c_read(I2C_ACCEL, 0x32, common_buf, 6, accel_done);
}

/* Kalman filter notes:
 *
 * Fill this out when I actually write a kalman filter
 */ 

void imu_read() {
   accel_read();
}
