/* gps_odometry.cpp
 *
 * A ROS node to fuse GPS and raw odometry data with a kalman filter to produce
 *  better location estimates.
 *
 * Author: Austin Hendrix
 */

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "gps_common/GPSFix.h"
#include "global_map/Offset.h"

#include "matrix.h"

matrix<1, 3> position(0.0);
matrix<3, 3> covariance(0.0);

// receive a GPS update
void gpsCallback(const gps_common::GPSFix::ConstPtr &gps) {
   ROS_INFO("Got gps message");

   // Kalman filter update step
}

// receive an odometry update
void odometryCallback(const nav_msgs::Odometry::ConstPtr &odo) {
   ROS_INFO("Got odometry message");

   // Kalman filter prediction step

   // FIXME: mostly garbage code to test that the matrix class compiles
   matrix<1, 3> update(0.0);
   position = update + position;

   update = position * covariance;
   covariance = covariance.T();
   covariance = invert(covariance);
}

int main(int argc, char ** argv) {

   ros::init(argc, argv, "gps_odometry");

   // subscribe to extended fix data from gps node.
   ros::NodeHandle n;

   ros::Subscriber gps_sub = n.subscribe("extended_fix", 10, gpsCallback);
   ros::Subscriber odo_sub = n.subscribe("base_odometry", 10, odometryCallback);

   ros::spin();

   return 0;
}
