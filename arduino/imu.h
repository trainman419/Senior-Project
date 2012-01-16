/* imu.h
 * 
 * Interface to 9dof IMU driver
 * 
 * Author: Austin Hendrix
 */

#ifndef IMU_H
#define IMU_H

#include "ros.h"

extern ros::Publisher compass_pub;
extern ros::Publisher accel_pub;
extern ros::Publisher gyro_pub;

void imu_init();

void imu_read();
#endif
