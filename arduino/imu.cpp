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
#include <math.h>

extern "C" {
#include "i2c.h"
}

#include "ros.h"
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>


#define I2C_ACCEL 0xA6
#define I2C_COMPASS 0x3C
#define I2C_GYRO 0xD0

#define X 0
#define Y 1
#define Z 2

#define min(a, b) ((a)<(b))?(a):(b)
#define max(a, b) ((a)>(b))?(a):(b)

geometry_msgs::Vector3 compass_msg;
geometry_msgs::Vector3 accel_msg;
geometry_msgs::Vector3 gyro_msg;

geometry_msgs::Vector3 compass_min;
geometry_msgs::Vector3 compass_max;
float compass_err;
geometry_msgs::Vector3 gyro_offset;

ros::Publisher compass_pub("compass", &compass_msg);
ros::Publisher accel_pub("accel", &accel_msg);
ros::Publisher gyro_pub("gyro", &gyro_msg);

// IMU State
//  angles in Yaw Pitch Roll (ZYX) order
geometry_msgs::Twist imu_state;

void imu_init() {
   i2c_init();
   // set up accelerometer
   i2c_write(I2C_ACCEL, 0x2A, 0x00); // disable tap detection
   i2c_wait();
   i2c_write(I2C_ACCEL, 0x2C, 0x09); // 50Hz data rate
   i2c_wait();
   i2c_write(I2C_ACCEL, 0x31, 0x0B); // full res mode & +/- 16g range
   i2c_wait();
   i2c_write(I2C_ACCEL, 0x38, 0x00); // disable FIFO; always get recent sample
   i2c_wait();
   i2c_write(I2C_ACCEL, 0x2D, 0x08); // enable measurement mode
   i2c_wait();

   // set up compass
   i2c_write(I2C_COMPASS, 0x00, 0x18); // 50Hz mode
   i2c_wait();
   i2c_write(I2C_COMPASS, 0x01, 0x20); // gain: 1300 counts/milli-gauss
   i2c_wait();
   i2c_write(I2C_COMPASS, 0x02, 0x00); // continuous-conversion mode
   i2c_wait();

   // set up gyro
   i2c_write(I2C_GYRO, 0x15, 19); // 50Hz sample rate
   i2c_wait();
   i2c_write(I2C_GYRO, 0x16, 0x1C); // 20Hz low-pass filter
   i2c_wait();
   i2c_write(I2C_GYRO, 0x3E, 0x01); // X gyro as clock reference
   i2c_wait();

   imu_state.linear.x = 0;
   imu_state.linear.y = 0;
   imu_state.linear.z = 0;
   imu_state.angular.x = 0;
   imu_state.angular.y = 0;
   imu_state.angular.z = 0;

   gyro_offset.x = 0;
   gyro_offset.y = 0;
   gyro_offset.z = 0;

   compass_min.x = 0;
   compass_min.y = 0;
   compass_min.z = 0;

   compass_max.x = 0;
   compass_max.y = 0;
   compass_max.z = 0;

   compass_err = 0;
}

geometry_msgs::Vector3 transform(geometry_msgs::Vector3 in,
      geometry_msgs::Vector3 rpy, bool inverse = false) {
   geometry_msgs::Vector3 out;
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

uint8_t common_buf[8];

// read compass, accelerometer and gyro and produce pose estimates
void update_imu() {
   // all estimates are absolute
   geometry_msgs::Vector3 gyro_est;    // RPY angles
   geometry_msgs::Vector3 compass_est; // RPY angles
   geometry_msgs::Vector3 accel_est;   // RPY estimate from accelerometer

   geometry_msgs::Vector3 odom_est;    // XYZ estimate from odometry

   // RPY estimation from gyro
   // TODO: make sure scaling on gyro is accurate
   gyro_est.x = gyro_msg.x + imu_state.angular.x - gyro_offset.x;
   gyro_est.y = gyro_msg.y + imu_state.angular.y - gyro_offset.y;
   gyro_est.z = gyro_msg.z + imu_state.angular.z - gyro_offset.z;

   // RPY estimation from compass
   // project compass onto the plane using old imu state
   compass_est = transform(compass_msg, imu_state.angular);
   compass_est.z = atan2(compass_est.y, compass_est.x) - compass_err;
   // no compass data on roll or pitch. use existing state
   //compass_est.y = imu_state.angular.y;
   //compass_est.x = imu_state.angular.x;

   // RPY estimate from accelerometer
   accel_est.y = (3.0 * M_PI / 2.0) - atan2(accel_msg.z, accel_msg.x);
   accel_est.x = (3.0 * M_PI / 2.0) - atan2(hypot(accel_msg.z, accel_msg.x),
         accel_msg.y);;
   // no z estimate from accelerometer
   //accel_est.z = imu_state.angular.z;

   // RPY estimate from odomery
   // TODO: do this properly
   odom_est.z = imu_state.angular.z;

   // combine sensor data
   //  this is entirely heuristic and based on intuition
   float x, y, z;

   // TODO: deal with angular wrap-around on all of these

   // z: yaw/heading
   //  sources: gyro, compass, odometry
   //  start with weighted average
   z = (1.0 * gyro_est.z + 1.0 * compass_est.z + 1.0 * odom_est.z) / 3.0;
   // if our compass error is too big, update the error value
   if( fabs( (compass_est.z - z) / z) > 0.1 ) {
      z = (1.0 * gyro_est.z + 1.0 * odom_est.z) / 2.0;
      compass_err += compass_est.z - z;
   } else {
      // else, try to drive the error metric back to zero
      compass_err *= 0.99;
   }

   // y: pitch
   //  sources: gyro and accelerometer
   //  weighted average until something better comes along
   y = (1.0 * gyro_est.y + 1.0 * accel_est.y) / 2.0;


   // x: roll
   //  sources: gyro and accelerometer
   x = (1.0 * gyro_est.x + 1.0 * accel_est.y) / 2.0;

   imu_state.angular.x = x;
   imu_state.angular.y = y;
   imu_state.angular.z = z;
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
   i2c_read(I2C_GYRO, 0x1D, common_buf, 6, &gyro_done);
}

void compass_done(uint8_t * buf) {
   // interpret and publish data
   int16_t compass = (buf[0] << 8) | buf[1];
   compass_msg.x = compass/1300.0;

   compass = (buf[2] << 8) | buf[3];
   compass_msg.y = compass/1300.0;

   compass = (buf[4] << 8) | buf[5];
   compass_msg.z = compass/1300.0;

   // compass calibration; keep track of minimum and maximum values
   compass_max.x = max(compass_msg.x, compass_max.x);
   compass_max.y = max(compass_msg.y, compass_max.y);
   compass_max.z = max(compass_msg.z, compass_max.z);

   compass_min.x = min(compass_msg.x, compass_min.x);
   compass_min.y = min(compass_msg.y, compass_min.y);
   compass_min.z = min(compass_msg.z, compass_min.z);

   // subtract out zero point
   compass_msg.x -= (compass_max.x + compass_min.x)/2.0;
   compass_msg.y -= (compass_max.y + compass_min.y)/2.0;
   compass_msg.z -= (compass_max.z + compass_min.z)/2.0;
//   compass_pub.publish(&compass_msg);

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
   accel_msg.x = accel / 256.0;

   accel = buf[2] | (buf[3] << 8);
   accel_msg.y = accel / 256.0;

   accel = buf[4] | (buf[5] << 8);
   accel_msg.z = accel / 256.0;
//   accel_pub.publish(&accel_msg);

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
