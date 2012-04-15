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
#include <stdlib.h>
#include <math.h>

extern "C" {
#include "i2c.h"
}

#include "twist.h"
#include "publish.h"

#define I2C_ACCEL 0xA6
#define I2C_COMPASS 0x3C
#define I2C_GYRO 0xD0

#define X 0
#define Y 1
#define Z 2

#define min(a, b) ((a)<(b))?(a):(b)
#define max(a, b) ((a)>(b))?(a):(b)

#define NORMALIZE(foo) while(foo > M_PI) foo -= M_PI*2.0; while(foo < -M_PI) foo += M_PI * 2.0;

uint8_t imu_enable = 0;

uint8_t i2c_fail = 0;
uint8_t i2c_resets = 0;

float imu_status = 0.0;
#define s(status) imu_status = max(status, imu_status)

Vector3 compass_msg;
Vector3 accel_msg;
Vector3 gyro_msg;

Vector3 compass_offset;
//Vector3 compass_min;
float compass_err;
Vector3 gyro_offset;

// IMU State
//  angles in Yaw Pitch Roll (ZYX) order
Twist imu_state;

Publisher<64> imu_pub('U');

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

   imu_state.linear.x = 0;
   imu_state.linear.y = 0;
   imu_state.linear.z = 0;
   imu_state.angular.x = 0;
   imu_state.angular.y = 0;
   imu_state.angular.z = 0;

   gyro_offset.x = 0;
   gyro_offset.y = 0;
   gyro_offset.z = 0;

   /* flat compass calibration */
   compass_offset.x = -0.036923;
   compass_offset.y = -0.107692;
   compass_offset.z = 0.03775;

   /* 3D compass calibration
   // Not as good on flat ground
   compass_offset.x = 0.0378;
   compass_offset.y = 0.0851;
   compass_offset.z = -0.0383;
   */

   compass_err = 0;

   imu_enable = 1;
   return;
}

Vector3 transform(Vector3 in,
      Vector3 rpy, bool inverse = false) {
   Vector3 out;
   // correlate math on paper with variables
   // theta: z
   // phi: y
   // psi: x

   // pre-compute sines and cosines
   float c_theta = cos(rpy.z);
   float s_theta = sin(rpy.z);
   float c_phi   = cos(rpy.y);
   float s_phi   = sin(rpy.y);
   float c_psi   = cos(rpy.x);
   float s_psi   = sin(rpy.x);

   // compute elements of rotation matrix
   
   // first row
   float r11 = c_phi * c_theta;
   float r12 = c_phi * s_theta;
   float r13 = s_phi;

   // second row
   float r21 = s_psi * s_phi * c_theta - c_psi * s_theta;
   float r22 = s_psi * s_phi * s_theta + c_psi * c_theta;
   float r23 = s_psi * c_phi;
   
   // third row
   float r31 = c_psi * s_phi * c_theta + s_psi * s_theta;
   float r32 = c_psi * s_phi * s_theta - s_psi * c_theta;
   float r33 = c_psi * c_phi;

   // matrix multiply
   if( !inverse ) {
      out.x = in.x * r11 + in.y * r12 + in.z * r13;
      out.y = in.x * r21 + in.y * r22 + in.z * r23;
      out.z = in.x * r31 + in.y * r32 + in.z * r33;
   } else {
      out.x = in.x * r11 + in.y * r21 + in.z * r31;
      out.y = in.x * r12 + in.y * r22 + in.z * r32;
      out.z = in.x * r13 + in.y * r23 + in.z * r33;
   }
   return out;
}

uint8_t common_buf[64];

// read compass, accelerometer and gyro and produce pose estimates
void update_imu() {
   s(7.0);
   // all estimates are absolute
   //Vector3 gyro_est;    // RPY angles
   Vector3 compass_est; // RPY angles
   Vector3 accel_est;   // RPY estimate from accelerometer

   Vector3 odom_est;    // XYZ estimate from odometry

   // RPY estimation from gyro
   // TODO: make sure scaling on gyro is accurate
   /*
   gyro_est.x = gyro_msg.x - gyro_offset.x;
   gyro_est.y = gyro_msg.y - gyro_offset.y;
   gyro_est.z = gyro_msg.z - gyro_offset.z;
   */
   /*
   gyro_est.x = gyro_msg.x + imu_state.angular.x - gyro_offset.x;
   gyro_est.y = gyro_msg.y + imu_state.angular.y - gyro_offset.y;
   gyro_est.z = gyro_msg.z + imu_state.angular.z - gyro_offset.z;
   */

   /*
   NORMALIZE(gyro_est.x);
   NORMALIZE(gyro_est.y);
   NORMALIZE(gyro_est.z);
   */

   // RPY estimation from compass
   // project compass onto the plane using old imu state
   accel_est = imu_state.angular;
   accel_est.z = 0.0;

   compass_est = compass_msg;
   //compass_est = transform(compass_msg, accel_est);
   //compass_est.z = atan2(compass_est.y, compass_est.x) - compass_err;
   compass_est.z = atan2(compass_est.x, compass_est.y);
   // no compass data on roll or pitch. use existing state
   //compass_est.y = imu_state.angular.y;
   //compass_est.x = imu_state.angular.x;

   // RPY estimate from accelerometer
   accel_est.y = (1.0 * M_PI / 2.0) + atan2(accel_msg.z, accel_msg.x);
   accel_est.x = (3.0 * M_PI / 2.0) + atan2(hypot(accel_msg.z, accel_msg.x),
         accel_msg.y);;
   // no z estimate from accelerometer
   //accel_est.z = imu_state.angular.z;
   accel_est.z = 0.0;
   NORMALIZE(accel_est.y);
   NORMALIZE(accel_est.x);

   // RPY estimate from odomery
   // TODO: do this properly
   odom_est.z = imu_state.angular.z;

   // combine sensor data
   //  this is entirely heuristic and based on intuition
   float x, y, z;

   // TODO: deal with angular wrap-around on everything

   // z: yaw/heading
   //  sources: gyro, compass, odometry
   //  start with weighted average
   //z = (1.0 * gyro_est.z + 1.0 * compass_est.z + 1.0 * odom_est.z) / 3.0;
   //z = (1.0 * gyro_est.z + 1.0 * odom_est.z) / 2.0;
   z = (1.0 * compass_est.z + 1.0 * odom_est.z) / 2.0;
   // if our compass error is too big, update the error value
   /*
   if( fabs( (compass_est.z - z) / z) > 0.1 ) {
      z = (1.0 * gyro_est.z + 1.0 * odom_est.z) / 2.0;
      compass_err += compass_est.z - z;
   } else {
      // else, try to drive the error metric back to zero
      compass_err *= 0.99;
   }
   */

   // y: pitch
   //  sources: gyro and accelerometer
   //  weighted average until something better comes along
   //y = (1.0 * gyro_est.y + 1.0 * accel_est.y) / 2.0;
   y = (2.0 * accel_est.y) / 2.0;


   // x: roll
   //  sources: gyro and accelerometer
   //x = (1.0 * gyro_est.x + 1.0 * accel_est.y) / 2.0;
   x = (2.0 * accel_est.y) / 2.0;

   imu_state.angular.x = x;
   imu_state.angular.y = y;
   imu_state.angular.z = z;

   //imu_pub.publish(&imu_state);
   if( imu_pub.reset() ) {
      //x = imu_state.angular.x * 180.0 / M_PI;
      x = imu_state.angular.x;
      //x = gyro_est.x * 180.0 / M_PI;
      //x = compass_msg.x;
      //x = compass_min.x;
      imu_pub.append(x);

      //y = imu_state.angular.y * 180.0 / M_PI;
      y = imu_state.angular.y;
      //y = gyro_est.y * 180.0 / M_PI;
      //y = compass_est.x;
      //y = compass_min.y;
      imu_pub.append(y);

      //z = imu_state.angular.z * 180.0 / M_PI;
      z = imu_state.angular.z;
      //z = gyro_est.z * 180.0 / M_PI;
      //z = compass_est.z * 180.0 / M_PI;
      //z = compass_est.z;
      //z = compass_min.z;
      imu_pub.append(z);
      imu_pub.finish();
      s(8.0);
   }

   return;
}

int16_t gyro_zero[3];
// number of gyro samples to take at startup
#define GYRO_COUNT 50
uint8_t gyro_start = GYRO_COUNT;
#define GYRO_DIV 20.0
#define GYRO_THRESHOLD 5

void gyro_done(uint8_t * buf) {
   s(6.0);

   int16_t gyro;
   // gyro Y is aligned with robot X
   gyro = (buf[2] << 8) | buf[3];
   if( gyro_start > 0 ) {
      gyro_zero[X] += gyro;
      gyro_msg.x = 0.0;
   } else {
      gyro -= gyro_zero[X];
      /*
      if( abs(gyro) < GYRO_THRESHOLD ) {
         gyro_zero[X] += gyro / 2;
         gyro = 0;
      }
      */
      gyro_msg.x = (gyro * 2000.0) / 0x7FFF;
      gyro_msg.x /= GYRO_DIV;
      gyro_msg.x *= M_PI / 180.0;
   }

   // gyro X is aligned with robot -Y
   gyro = (buf[0] << 8) | buf[1];
   if( gyro_start > 0 ) {
      gyro_zero[Y] += gyro;
      gyro_msg.y = 0.0;
   } else {
      gyro -= gyro_zero[Y];
      /*
      if( abs(gyro) < GYRO_THRESHOLD ) {
         gyro_zero[Y] += gyro / 2;
         gyro = 0;
      }
      */
      gyro_msg.y = -(gyro * 2000.0) / 0x7FFF;
      gyro_msg.y /= GYRO_DIV;
      gyro_msg.y *= M_PI / 180.0;
   }

   // gyro Z is aligned with robot Z
   gyro = (buf[4] << 8) | buf[5];
   if( gyro_start > 0 ) {
      gyro_zero[Z] += gyro;
      gyro_msg.z = 0.0;
   } else {
      gyro -= gyro_zero[Z];
      /*
      if( abs(gyro) < GYRO_THRESHOLD ) {
         gyro_zero[Z] += gyro / 2;
         gyro = 0;
      }
      */
      gyro_msg.z = (gyro * 2000.0) / 0x7FFF;
      gyro_msg.z /= GYRO_DIV;
      gyro_msg.z *= M_PI / 180.0;
   }

   if( gyro_start > 0 ) {
      --gyro_start;
      if( gyro_start == 0 ) {
         gyro_zero[X] /= GYRO_COUNT;
         gyro_zero[Y] /= GYRO_COUNT;
         gyro_zero[Z] /= GYRO_COUNT;
      }
   }

   update_imu();
   return;
}
// read the gyro
void gyro_read() {
   s(5.0);
   if( i2c_read(I2C_GYRO, 0x1D, common_buf, 6, gyro_done) ) {
      ++i2c_fail;
   }
   return;
}

void compass_done(uint8_t * buf) {
   s(4.0);

   // interpret and publish data
   int16_t compass = (buf[0] << 8) | buf[1];
   compass_msg.x = ((-compass/1300.0) - compass_offset.x + compass_msg.x) / 2;
   //compass_msg.x = (compass/1300.0);

   compass = (buf[2] << 8) | buf[3];
   compass_msg.y = ((-compass/1300.0) - compass_offset.y + compass_msg.y) / 2;
   //compass_msg.y = (compass/1300.0);

   compass = (buf[4] << 8) | buf[5];
   compass_msg.z = ((-compass/1300.0) - compass_offset.z + compass_msg.z) / 2;
   //compass_msg.z = (compass/1300.0);

   /*
   compass_min.x = max(compass_min.x, compass_msg.x);
   compass_min.y = max(compass_min.y, compass_msg.y);
   compass_min.z = max(compass_min.z, compass_msg.z);
   */

   update_imu();
   //gyro_read();
   return;
}

// read the compass
void compass_read() {
   s(3.0);
   if( i2c_read(I2C_COMPASS, 0x03, common_buf, 6, compass_done) ) {
      ++i2c_fail;
   }
   return;
}

void accel_done(uint8_t * buf) {
   s(2.0);
   // interpret and publish data
   int16_t accel;
   // accel -Y maps to robot X
   accel = -(buf[2] | (buf[3] << 8));
   accel_msg.x = ((accel / 256.0) + 3*accel_msg.x) / 4;

   // accel X maps to robot Y
   accel = (buf[0] | (buf[1] << 8));
   accel_msg.y = ((accel / 256.0) + 3*accel_msg.y) / 4;

   // accel -Z maps to robot Z
   accel = -(buf[4] | (buf[5] << 8));
   accel_msg.z = ((accel / 256.0) + 3*accel_msg.z) / 4;

   compass_read();
   return;
}

// read the accelerometer
void accel_read() {
   s(1.0);
   if( i2c_read(I2C_ACCEL, 0x32, common_buf, 6, accel_done) ) {
      ++i2c_fail;
   }
   return;
}

/* Kalman filter notes:
 *
 * Fill this out when I actually write a kalman filter
 */ 

void imu_read() {
   if( imu_enable ) {
      s(0.0);
      accel_read();
   } 
   if( i2c_fail > 5 ) {
      ++i2c_resets;
      imu_enable = 0;
      imu_init();
      i2c_fail = 0;
   }
   return;
}
